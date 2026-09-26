# Выданный датасет: совместимость, инструкция и план подключения

Дата: 25 сентября 2026. Состояние кода: `536e5d2`. Сначала прочитать [план соответствия требованиям](15-organizer-requirements-and-roadmap.md); список всех bag — в [инвентаризации](17-dataset-inventory.md).

## 1. Можно ли работать с датасетом сейчас?

**Изучать и разбирать данные — да. Запустить готовую штатную команду Lab на `.db3` или сразу сдать текущую ROS-ноду — нет.**

| Действие | Готовность |
|---|---|
| Прочитать metadata и таблицы SQLite без ROS | Доступно сейчас |
| Нормализовать `position` и `velocity` существующим Normalizer | Проверено на реальных сообщениях, нужны профили |
| Передать нормализованные Python-объекты в baseline/EKF | Проверено диагностическим офлайн-прогоном |
| `odometry-lab run --source dataset/data/<bag>` | Не поддерживается: reader знает CSV/JSON/JSONL/WebSocket |
| `odometry-lab probe <file.db3>` | Не поддерживается: probe знает текстовые форматы |
| `ros2 bag play` → текущий `SourceAdapter` | Возможен после сборки custom types и подготовки профилей |
| Проверенный live-прогон EKF на реальном bag | Требует исправления startup и обработки времени |
| Автоматическая оценка по GNSS из bag | Требует отдельного truth reader и преобразований |
| Отправка результата жюри | Нет нужных выходных топиков и согласованной XYZ-геометрии |

`docker compose --profile demo up --build` по-прежнему запускает синтетический сценарий. Наличие папки `dataset` автоматически его не переключает. `--profile ros` тоже использует сгенерированные события.

## 2. Фактический состав, а не только описание README

SQL-подсчет сообщений и осмотр метаданных всех 122 файлов дали:

| Параметр | Результат |
|---|---:|
| Bag-каталоги | 122 |
| Префиксы | 30618: 103; 30639: 19 |
| Суммарная длительность по metadata | 36,7455 ч |
| Число сообщений в SQLite | 8 788 325 |
| Объем `.db3` | 863 756 288 байт, около 823,74 MiB |
| Минимальная длительность записи | 1,2818 с |
| Медианная длительность | 1 190,77 с |
| Максимальная длительность | 1 897,48 с |
| Записи с сообщениями обоих GNSS-приемников | 86 |
| Только rover GNSS | 1: `30639_e4379d7f` |
| Без сообщений GNSS | 35 |

Во всех bag объявлены семь топиков, но в 35 GNSS-топики пусты. Следовательно, «топик присутствует» не означает «эталон доступен». В этих 35 записях нет и начальных GNSS-измерений; предусмотреть заданное начальное состояние или явно относительную одометрию.

Для начала использовать:

| Bag | Длительность | Назначение |
|---|---:|---|
| `30618_1551d0a9` | 1,28 с | Формат и стартовый backlog; колеса стоят, для оценки качества модели непригоден |
| `30618_0d865417` | 11,84 с | Короткая динамика, скорость до 3,90 м/с, ручка −11…6 |
| `30639_0ab96c59` | 79,79 с | Короткая запись второго префикса, скорость до 3,76 м/с |
| `30618_01f73500` | 1 211,98 с | Длительная запись с обоими GNSS |
| `30618_0259fe53` | 1 228,31 с | Работа без GNSS; проверять доступность и устойчивость, не истинную ошибку |
| `30618_27e994fc` | 1 897,48 с | Самый длинный прогон, память и накопление ошибки |

### 2.1. Значения сигналов

- Ручка действительно содержит все целые позиции от −15 до +15. Нейтраль — 0; знак тяги/торможения следует README.
- Передняя скорость: −0,3783…53,7297 м/с; задняя: −0,3890…53,9195 м/с. Верхние значения — наблюдаемые показания датчиков, не подтвержденная скорость трамвая.
- Найдено 15 отрицательных передних и 13 отрицательных задних отсчетов. В текущем Normalizer они будут отклонены. Причину нужно установить по соседним отсчетам и truth, а не автоматически менять знак.
- В просмотренных колесных payload всех записей нет NaN/Inf; это не отменяет необходимости runtime-валидации.
- Источников колесной скорости два, это скорости тележек. Нет отдельных левых/правых колес для вычисления yaw.

### 2.2. Время — главный интеграционный риск

SQLite `messages.timestamp` и `msg.header.stamp` — разные величины. В аудите они анализировались отдельно, базы открывались через `mode=ro`. Header и значения трех custom-потоков диагностически извлекались из CDR согласно приложенным `.msg`; это ограниченный аудиторский декодер, не универсальный production reader.

| Поток | Количество | Header позади записи >20 мс | Медиана record−header | p95 record−header |
|---|---:|---:|---:|---:|
| Ручка | 2 652 391 | 50 245, около 1,89% | 1,01 мс | 6,96 мс |
| Передняя тележка | 1 239 220 | 949 790, около 76,64% | 49,34 мс | 94,61 мс |
| Задняя тележка | 1 237 408 | 949 896, около 76,76% | 49,04 мс | 94,01 мс |
| GNSS master velocity | 895 750 | 885 421 | 84,67 мс | 97,83 мс |
| GNSS rover velocity | 915 137 | 905 611 | 88,55 мс | 102,17 мс |

Количество и пороги посчитаны по всем сообщениям. Квантили выше — по систематической выборке каждого 113-го сообщения соответствующего топика по совокупности bag; это не точные квантили полного массива. Record−header нельзя автоматически называть сетевой задержкой: возможны различия часов.

Есть отрицательные разности, backlog порядка нескольких секунд и возвраты header-времени внутри потока: ручка 75, передняя 25, задняя 27. Поэтому единственный постоянный сдвиг не является готовым решением. Стартовые накопленные сообщения объясняют, почему интервал header-времени может быть длиннее duration из metadata.

У передней/задней тележек 1 236 228 общих уникальных header timestamp в пределах соответствующих bag. Большинство пар синхронны по измерительному времени, но приходят отдельными сообщениями. Нужно сохранить эту пару до закрытия кадра, иначе нынешнее голосование по каналам может не сработать.

Текущий topics-таймер использует `/clock − 20 мс`, а ядро отклоняет события с timestamp ≤ уже опубликованного времени. Из статистики следует риск массового отбрасывания колес при штатном replay. Приведенные 76–77% — доля сообщений с таким расхождением часов, **не измеренная доля потерь готовой ROS-системы**; последнюю нужно измерить после подключения.

## 3. Что уже проверено экспериментально

Для `30618_0d865417` временным диагностическим скриптом разобраны 524 сообщения трех входных топиков. После существующего Normalizer и сортировки по header оба оценивателя выполнили `run_events`:

| Результат | wheel-hold | adaptive-ekf |
|---|---:|---:|
| Кадры выхода | 764 | 764 |
| Действительные кадры | 713 | 764 |
| Конечный s | 6,858 м | 10,394 м |
| Конечная скорость | 0 м/с | 1,805 м/с |

Ошибок нормализации нет. Число кадров определяется диапазоном header, который включает накопленные в начале сообщения; оно не обязано быть `duration_metadata × 50`.

Это подтверждает совместимость полей с Python API, но не live-причинность, точность или победу EKF. GNSS truth для этого запуска не строился. В частности, ненулевая конечная скорость EKF при нулевых колесах заслуживает проверки gating/freeze/recovery и модели торможения.

Отдельно воспроизведены две ошибки:

1. `AdaptiveOdometryEstimator().advance_to(1_000_000_000)` вызывает `TypeError`: сравниваются int и None. В topics-режиме это возможно при ненулевом `/clock` до первого vehicle-сообщения.
2. Проверка выданного `tram_vehicle_msgs/package.xml` штатным `catkin_pkg` внутри Humble-контейнера завершается `Package 'tram_vehicle_msgs' must declare at least one maintainer`.

Локальные материалы аудита находятся в `artifacts/organizer-audit/`: `inventory.json`, `current-probe.json`, `audit.py`, `probe_current.py`. Каталог игнорируется Git; проверяемая сводка зафиксирована в этих Markdown-документах. Для командной воспроизводимости полноценный reader и тесты нужно добавить в проект отдельным этапом.

## 4. Что можно запускать прямо сейчас

### 4.1. Инвентаризация без ROS

В корне репозитория, PowerShell:

```powershell
Get-ChildItem dataset/data -Directory | Measure-Object
Get-Content dataset/data/30618_0d865417/metadata.yaml
Get-Content dataset/tram_vehicle_msgs/msg/VelocitySensor.msg
Get-Content dataset/tram_vehicle_msgs/msg/DriverControllerCommand.msg
```

Подсчет реально существующих сообщений стандартным Python SQLite, только чтение:

```powershell
.venv/Scripts/python.exe -c "from pathlib import Path; import sqlite3; p=Path('dataset/data/30618_0d865417/30618_0d865417_0.db3'); c=sqlite3.connect(p.resolve().as_uri()+'?mode=ro',uri=True); print(c.execute('SELECT t.name,t.type,count(m.id) FROM topics t LEFT JOIN messages m ON m.topic_id=t.id GROUP BY t.id').fetchall()); c.close()"
```

Не редактировать базы через SQLite-браузер. Не путать объявление топика в таблице `topics` с ненулевым числом его сообщений.

### 4.2. Подготовка ROS custom types

Следующие команды рассчитаны на **Linux/WSL Ubuntu 22.04 с ROS 2 Humble**, где уже доступны colcon и стандартные зависимости. Это ручная подготовка рабочего окружения; в текущем Dockerfile датасет и пакет организаторов не копируются.

```bash
source /opt/ros/humble/setup.bash
REPO=/absolute/path/to/odometry-service
DATASET_WS=$(mktemp -d /tmp/odometry-dataset-ws.XXXXXX)
mkdir -p "$DATASET_WS/src"
cp -R "$REPO/dataset/tram_vehicle_msgs" "$DATASET_WS/src/"
```

В **скопированном** `package.xml` добавить внутри `<package>` maintainer с реальным адресом команды, например по синтаксису:

```xml
<maintainer email="team@example.org">Odometry team</maintainer>
```

Адрес примера заменить. `.msg` не менять, исходную папку `dataset` оставить неизменной. Затем:

```bash
cd "$DATASET_WS"
colcon build --packages-select tram_vehicle_msgs
source "$DATASET_WS/install/setup.bash"
ros2 interface show tram_vehicle_msgs/msg/VelocitySensor
ros2 interface show tram_vehicle_msgs/msg/DriverControllerCommand
ros2 bag info "$REPO/dataset/data/30618_0d865417"
```

Этот полный build в ходе аудита не выполнялся; проверен конкретный дефект manifest. Если workspace временный, сохранить его путь и source-ить overlay в каждом терминале. Для основной разработки пакет после исправления рабочей копии нужно включить в штатную ROS-сборку.

### 4.3. Просмотр raw-потоков

В терминале player после `source` ROS и custom overlay:

```bash
ros2 bag play "$REPO/dataset/data/30618_0d865417" --clock 100 --delay 2
```

В других терминалах с теми же overlay:

```bash
ros2 topic echo /vehicle/driver_position_cmd
ros2 topic echo /vehicle/front_bogie_velocity
ros2 topic echo /vehicle/rear_bogie_velocity
```

Запускать echo до короткого replay. Каждый новый прогон — новый оцениватель; не использовать loop до проверки clock reset. На всем ROS-графе согласовать `ROS_DOMAIN_ID`. Единственный издатель `/clock` — bag player; наш файловый `replay` одновременно не запускать.

Для теста без GNSS можно ограничить player тремя `/vehicle/*` через `--topics`; это лишит bootstrap начальной GNSS-выставки, поэтому нужен заранее заданный initial state. Сам оцениватель не должен зависеть от публикации всех семи топиков.

## 5. Профили для существующего SourceAdapter

Ниже **содержимое будущих файлов**, а не уже установленные конфигурации. Их можно создать без изменения алгоритма нормализации. Используется `layout: wide`, потому что в custom Wheel message нет поля wheel_id: ID задается профилем самого топика. Wrapper `kind` здесь не нужен.

`contracts/profiles/organizer-front.yaml`:

```yaml
version: "0.1"
layout: wide
time: {field: header.stamp, unit: ros, clock: organizer-bag}
control: {field: __absent__, mapping: linear, scale: 1}
wheels:
  - {id: front, field: velocity, unit: m/s}
```

`contracts/profiles/organizer-rear.yaml`:

```yaml
version: "0.1"
layout: wide
time: {field: header.stamp, unit: ros, clock: organizer-bag}
control: {field: __absent__, mapping: linear, scale: 1}
wheels:
  - {id: rear, field: velocity, unit: m/s}
```

`contracts/profiles/organizer-control.yaml`:

```yaml
version: "0.1"
layout: wide
time: {field: header.stamp, unit: ros, clock: organizer-bag}
control: {field: position, mapping: linear, scale: 0.06666666666666667}
wheels:
  - {id: unused, field: __absent__, unit: m/s}
```

`__absent__` — отсутствующее поле, его пропускает текущий wide-normalizer. Искусственный wheel ID `unused` не публикуется. Это совместимый временный прием, поскольку текущая схема требует и `control`, и непустой список `wheels` в каждом профиле. В дальнейшем лучше поддержать явные control-only/wheel-only профили. Отсутствующие `seq_field` и `valid_field` означают генерацию seq адаптером и valid=true для прошедшей нормализацию записи; это не оценка исправности сенсора.

`contracts/profiles/organizer-topics.yaml`:

```yaml
topics:
  - name: /vehicle/front_bogie_velocity
    type: tram_vehicle_msgs/msg/VelocitySensor
    profile: organizer-front.yaml
    qos: sensor
  - name: /vehicle/rear_bogie_velocity
    type: tram_vehicle_msgs/msg/VelocitySensor
    profile: organizer-rear.yaml
    qos: sensor
  - name: /vehicle/driver_position_cmd
    type: tram_vehicle_msgs/msg/DriverControllerCommand
    profile: organizer-control.yaml
    qos: sensor
```

Качество исходных сообщений не закодировано специальным valid-полем. Порог отрицательной скорости, freeze и разногласие тележек проверяются отдельно. Счетчики seq независимы по топикам; ID front/rear сохраняет различимость колес.

После создания профилей и сборки основных пакетов можно проверить **сам адаптер**:

```bash
source /opt/ros/humble/setup.bash
source "$REPO/ros2_ws/install/setup.bash"
source "$DATASET_WS/install/setup.bash"
ros2 run odometry_node source_adapter --ros-args \
  -p config:="$REPO/contracts/profiles/organizer-topics.yaml" \
  -p use_sim_time:=true
```

Проверять `/tram/control` и `/tram/wheel_speed` через `ros2 topic echo`. Убедиться, что позиции ±15 дают ±1, а скорость не пересчитывается повторно. Если Python-пакеты `odometry_core`/`odometry_io` не установлены в этом окружении, одна colcon-сборка нынешней ROS-оболочки их не установит: использовать подготовленное окружение проекта или исправить упаковку по P1-C.

### 5.1. Следующий запуск после исправлений P0

После guard старта, согласования времени и добавления publisher контракта:

```bash
ros2 run odometry_node estimator --ros-args \
  -p estimator:=adaptive-ekf -p input_mode:=topics \
  -p use_sim_time:=true -p run_id:=organizer-30618-0d865417
```

Эти параметры существуют уже сейчас, но **команда пока не подтверждает готовность к данным**: текущая версия может упасть до первого измерения, отсечь поздние колеса и не публикует `/result/*`. Поэтому порядок внедрения важен: сначала исправления P0, затем acceptance-прогон. `processing_delay_ms` выбирать по проверенному временному протоколу, а не механически ставить большое значение.

Во время проверки после исправлений:

```bash
ros2 topic info /result/velocity --verbose
ros2 topic info /result/position --verbose
ros2 topic hz /result/velocity
ros2 topic hz /result/position
ros2 topic echo /odometry/diagnostics
```

Частоту надо считать во время активного bag, исключая паузу и остановку `/clock`. Проверка задержки требует отдельного измерителя от callback входа до публикации, а не только `topic hz`.

## 6. Офлайн-путь для DS: что реализовать

Штатный новый reader предлагается разместить в `odometry_io`, CLI экспорта — в `odometry_lab`. Использовать `rosbag2_py` в Humble или библиотеку `rosbags` с фиксированной совместимой версией для Python 3.10. В текущих зависимостях `rosbags` отсутствует. Пример AnyReader из dataset README нужно дополнить регистрацией обоих custom `.msg`; произвольное открытие файла без typestore недостаточно.

Не копировать аудиторский struct-разбор как универсальный reader: полноценная библиотека должна проверять типы и CDR.

Один запуск экспорта должен создавать:

```text
artifacts/datasets/<bag_id>/
  manifest.json        # входные файлы/хеши, типы, времена, счетчики, качество, версия экспорта
  inputs.jsonl         # только три разрешенных входа, raw notch и обе временные шкалы
  truth.jsonl          # отдельно GNSS, затем очищенный метрический эталон
  quality.json         # late/negative/reset/GNSS gaps, правила инициализации
  profile.yaml         # конкретный mapping для наших событий
```

Планируемые, **пока не существующие**, команды:

```text
odometry-lab inspect-bag --source dataset/data/<bag>
odometry-lab export-bag --source dataset/data/<bag> --output artifacts/datasets/<bag>
odometry-lab replay-bag --source dataset/data/<bag> --estimator adaptive-ekf
```

Требования к reader:

1. Регистрировать ровно приложенные custom types; проверять тип топика и входной allowlist.
2. Итеративно читать записи, не держать все 8,8 млн сообщений и выходы одновременно в RAM.
3. Сохранять bag ID, topic, source seq, header ns, recorded ns, исходный position и нормализованное u.
4. Не подмешивать GNSS в inputs; разметку и truth передавать только evaluator.
5. Отдельные режимы `analysis_sorted` и `causal_replay`: первый удобен для EDA, второй сохраняет очередность фактического поступления и используется для проверки online-алгоритма.
6. Обрабатывать несколько clock-эпох, отсутствующие/пустые топики и partial truth. Версионировать все правила исправления данных; не переписывать оригинальные bag.
7. Формировать проверенный профиль именно с `front/rear`: существующий `events.yaml` разрешает `left/right`, поэтому экспорт с новыми ID через этот профиль будет отклонен.

После появления экспорта в нашем каноническом формате **существующие** команды Lab пригодны для исследовательского offline-прогона:

```bash
uv run odometry-lab run \
  --source artifacts/datasets/<bag>/inputs.jsonl \
  --profile artifacts/datasets/<bag>/profile.yaml \
  --output artifacts/runs/<bag>-M1 --run-id <bag>-M1 --estimator adaptive-ekf
uv run odometry-lab evaluate \
  --estimates artifacts/runs/<bag>-M1/estimates.jsonl \
  --truth artifacts/datasets/<bag>/truth.jsonl \
  --output artifacts/runs/<bag>-M1/report.json
```

Подставлять настоящие bag ID вместо `<bag>`. Этот evaluator ожидает готовые `stamp_ns/s_m/v_mps`, не raw lat/lon. Поскольку текущий file reader сортирует события по header, такой запуск нельзя объявлять причинным ROS-replay. Для записи без GNSS опустить `--truth` и получить unavailable-метрики, сохранив доступность и режимы.

## 7. Чек-лист первого реального эксперимента

1. На `30618_0d865417` проверить package types, три профиля и значения; сохранить счетчики.
2. Исправить запуск EKF до первых входов; проверить старт до `/clock` и до первого vehicle-сообщения.
3. Провести причинный replay, сравнить задержки и потери front/rear/control. Зафиксировать политику late data.
4. Из того же bag отдельно построить GNSS truth; проверить оси, units, качество и временную привязку. До этого не называть разницу baseline/EKF точностью.
5. Сравнить B0/B2/M1/M2 в одинаковых условиях и с одинаковым bootstrap. Проверить остановку в конце короткой записи.
6. Повторить на `30639_0ab96c59`, затем на длинном `30618_01f73500` и записи без GNSS `30618_0259fe53`.
7. Сделать holdout целыми непересекающимися прогонами. Не перемешивать соседние отсчеты одного пути между train и test.
8. Подключить `/result/*`, сопоставление по правилам системы оценки и согласованную геометрию. Проверить работу без API/Vue, offline colcon-сборку и ограничения ресурсов.

## 8. Границы проведенной проверки

Проверены PDF, README, весь состав SQLite, временные заголовки, значения трех внутренних потоков, совместимость Normalizer на коротком bag, офлайн-вызовы ядра и manifest-validation в ROS-контейнере. Полный набор GNSS координат/качества еще не проанализирован, истинные ошибки на реальных записях не рассчитаны, пакет организаторов целиком не собран, online ROS-probe реального bag не выполнен. Поэтому вывод — «интеграция технически возможна, выявлены конкретные препятствия», а не «решение уже готово к сдаче».
