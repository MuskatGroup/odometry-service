# Запуск рабочего каркаса

## Что реализовано

В одном репозитории работают: transport-independent Python-ядро с baseline `wheel-hold-v1` и `ekf-robust-v1`, профили и адаптеры CSV/JSON/JSONL/WebSocket, ROS-адаптер, replay, Failure Lab, HTTP/SignalR API и Vue.

Рабочий EKF использует состояние `[s, v, a_act, d]`, нелинейную модель тяги/торможения, NIS-гейт, поканальное голосование, детектор зависания, гистерезис возврата и повторный захват. Адаптация возмущения доступна экспериментальным переключателем, но по умолчанию отключена. Автокалибровка параметров, ML-классификатор и доказанная точность на реальном трамвае пока **не реализованы**.

Baseline использует медиану свежих каналов, затем удерживает скорость до тайм-аута качества. Продолжение численного прогноза с `valid=false` предназначено для анализа ошибок, а не использования в управлении. `FUSED` в этом baseline означает наличие свежих колес и управления; это не заявление о работающем EKF.

Baseline не публикует неопределенность. EKF публикует `sigma` и ковариацию 4×4 с обязательным признаком `UNCERTAINTY_UNCALIBRATED`. ROS публикует `/odometry/estimate`, диагностику и совместимое `/odometry/filtered` для действительной EKF-оценки.

## Быстрый запуск всей демонстрации

Требуется работающий Docker Engine в режиме Linux containers. Из корня:

```bash
docker compose --profile demo up --build
```

После завершения контейнера `demo` откройте [http://localhost:8080](http://localhost:8080). Там появится `demo-001`, синтетический отказ колес, графики и отчет. Контейнер demo должен завершиться с кодом 0; console продолжает работать. Данные остаются в `artifacts/demo`, каталог API — в Docker volume `console-data`.

Повторный импорт идентичных кадров дедуплицируется. Для другого эксперимента используйте новый `run_id`: сервер не заменяет уже сохраненный кадр с тем же seq.

Остановить сервисы без удаления сохраненных данных:

```bash
docker compose --profile demo --profile ros down
```

## Разработка Python без ROS

Нужны uv и Python 3.10 (uv может установить интерпретатор):

```bash
uv sync --locked --python 3.10
uv run odometry-lab demo --output artifacts/demo
uv run pytest -q
uv run ruff check python tests ros2_ws/src/odometry_node
```

В Windows без uv команды доступны через `.venv\Scripts\python.exe -m odometry_lab.cli` после установки окружения. На Linux — через `.venv/bin/python`.

Отдельные шаги:

```bash
uv run odometry-lab generate --output artifacts/experiment --fault slip
uv run odometry-lab run --source artifacts/experiment/events.jsonl --profile contracts/profiles/events.yaml --output artifacts/experiment --run-id experiment-001 --estimator adaptive-ekf
uv run odometry-lab evaluate --estimates artifacts/experiment/estimates.jsonl --truth artifacts/experiment/truth.jsonl --faults artifacts/experiment/faults.jsonl --output artifacts/experiment/report.json
uv run odometry-lab import-report --directory artifacts/experiment --api http://localhost:8080
```

Без `--truth` evaluator возвращает null для фактических ошибок и причину `NO_INDEPENDENT_TRUTH`. Разделение train/test и идентификация модели относятся к следующему этапу DS.

Генератор поддерживает измерительные `slip|common_slip|slide|lock|freeze|dropout|scale`, физические `grade|traction_scale` и комбинированный `grade_in_dropout` отказы, `--seed`, `--duration`, `--fault-start`, `--fault-end`, `--wheels`. Физика включает лаг привода. Разметка сохраняется в `faults.jsonl` и не поступает в оцениватель.

Сравнение B0/B1/B2/M1/M2 на сценариях запускается так:

```bash
uv run odometry-lab benchmark --output artifacts/benchmark --seeds 17,23
```

Результаты — `benchmark.json`, `benchmark.md` и отдельные отчеты. Это синтетический regression benchmark, а не доказательство точности.

## Подключение файлов

```bash
uv run odometry-lab run --source tests/fixtures/events.csv --profile contracts/profiles/events.yaml --output artifacts/csv --run-id csv-example
uv run odometry-lab run --source tests/fixtures/events.json --profile contracts/profiles/events.yaml --output artifacts/json --run-id json-example
```

`events.yaml` описывает строки отдельных событий. `wide.yaml` — строки с ручкой и несколькими колесами, включая преобразование km/h и RPM.

Для неизвестного файла можно получить черновик профиля:

```bash
uv run odometry-lab probe sample.csv --output artifacts/profile-draft.yaml
```

`probe` не утверждает, что угадал единицы или шкалу времени: созданный YAML содержит предупреждения `REVIEW_REQUIRED` и должен быть проверен человеком.

Профиль определяет поля через пути `outer.inner.0.value`, временные единицы ns/us/ms/s/ros, явный `offset_ns`, кодировку ручки и колеса. JSON-объект с вложенным массивом поддерживается через `records_path`. В CSV доступен `delimiter`. Отсутствующий отсчет не заменяется нулем.

Входной файл ограничен 100 MB по умолчанию: `max_file_bytes`. Он загружается и упорядочивается перед replay. Для больших записей пока нужно разделение файлов. Поврежденные строки учитываются в `run.json.diagnostics`.

## WebSocket

Терминал 1:

```bash
uv run odometry-lab serve-ws --source artifacts/demo/events.jsonl
```

Терминал 2:

```bash
uv run odometry-lab run --source ws://127.0.0.1:8765 --profile contracts/profiles/events.yaml --output artifacts/ws --run-id ws-example --duration 24
```

Соединение принимает один JSON-объект на сообщение. Встроенный сервер заканчивает поток после записи; наблюдатель продолжает работать до `--duration`, поэтому видны истечение свежести и INVALID. Переподключение не выполняется.

Для WebSocket → ROS используйте сервер с `--system-time` и профиль `events-system.yaml`. Часы должны быть согласованы с ROS system time. При расхождении старта более 5 секунд источник останавливается с диагностикой. Офлайн Lab использует шкалу источника с монотонным отсчетом времени воспроизведения.

## ROS-демонстрация

```bash
docker compose --profile ros up --build
```

Сначала seed создает запись. Replay публикует атомарные `/tram/events`, отдельно `/tram/control` и `/tram/wheel_speed`, а также единственный `/clock`. Estimator использует batch-режим и обрабатывает только `/tram/events`; отдельные колесные топики не применяются второй раз.

Replay идет с шагом 20 мс и завершает продвижение времени через 3 секунды после последнего события. Bridge передает прореженную телеметрию до 10 Гц. После завершения replay время стоит, на UI появляется признак отсутствия свежей live-телеметрии; сохраненный запуск остается доступен.

Для живых внешних топиков внутри собранного ROS-контейнера:

```bash
ros2 run odometry_node source_adapter --ros-args -p config:=/workspace/contracts/profiles/ros-topics.yaml
ros2 run odometry_node estimator --ros-args -p input_mode:=topics -p run_id:=external-001
```

Доступные типы сообщений выбираются через `type: package/msg/Name`; соответствующий пакет должен быть установлен. Пример профиля использует внешние топики наших ControlSample/WheelSample. Для организаторского формата заменяются типы, имена и mapping полей. Сам алгоритм не меняется.

Topics-режим имеет явно заданную задержку обработки `processing_delay_ms=20` для доставки событий. Это начальная настройка целостности данных, а не достижение прежней цели задержки 10 мс. Поздние события отклоняются; время расчета ядра измеряется отдельно.

## Запись и повтор ROS

В ROS-окружении можно записать атомарный вход:

```bash
ros2 bag record -o /artifacts/recording /tram/events
```

Для проверки остановите первоначальный источник и запустите новый экземпляр estimator (или новый запуск), затем:

```bash
ros2 bag play /artifacts/recording --clock
```

При этом единственный источник `/clock` — проигрыватель. Не воспроизводите одновременно записанный `/clock` и генерируемые часы. В batch-режиме timestamp заголовка группы задает момент продвижения ядра; его содержимое воспроизводится без перестановок между отдельными топиками.

Интеграционный тест записывает реальные ROS-сообщения, воспроизводит bag и сравнивает значения с офлайн-ядром:

```bash
docker compose --profile test build ros-tests
docker compose --profile test run --rm --no-deps ros-tests
```

## API и frontend локально

Терминал API:

```bash
dotnet run --project apps/api --urls http://localhost:8080
```

Терминал UI, рабочая директория `apps/web`:

```bash
npm ci
npm run dev
```

Откройте [http://localhost:5173](http://localhost:5173). Vite проксирует API и SignalR. Затем импортируйте отчет командой выше. Для release-сборки используйте `npm run build`; Docker копирует статику в API.

API предоставляет исходные endpoints из контракта и `GET /api/runs/{id}/telemetry?afterSeq=-1&limit=2000`. Кадры дедуплицируются по run_id/seq. Предел — 500 кадров в батче, 32 KB на кадр, 8 MB на HTTP-запрос. SQLite хранит каталог и экранную телеметрию; отчеты — JSON-файлы. Полные входы/оценки остаются артефактами Lab.

Контур предназначен для локального стенда; Compose привязывает web-порт к loopback. Аутентификация и внешняя публикация не входят в каркас.

## Проверки API

После `dotnet build apps/api/Odometry.Api.csproj`:

PowerShell:

```powershell
$env:RUN_API_TESTS = '1'
uv run pytest -q tests/test_api.py
```

Linux:

```bash
RUN_API_TESTS=1 uv run pytest -q tests/test_api.py
```

Тесты поднимают настоящий сервер на свободном локальном порту и проверяют дедупликацию, timestamp без потери точности, сохранение отчета и ошибки контракта.

## Следующий шаг команды

DS калибрует параметры EKF и сравнивает его с baseline на реальных данных; автоматизатор расширяет независимые сценарии и проверку доверия. Архитектор подключает формат организаторов. Backend/frontend могут развиваться на сохраненных отчетах и SignalR независимо от алгоритма. Rust, ML, LLM и управление запуском из UI остаются вне этого этапа.
