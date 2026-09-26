# План реализации для двух разработчиков

Дата фиксации: 26 сентября 2026.

Статус: **единственный действующий план реализации**.

Этот документ достаточен для начала работы. Другие планы не являются техническим
заданием. Технические исследования из старых документов открываются только по ссылкам,
когда нужна дополнительная информация.

## 1. Цель итерации

За 3–4 дня два разработчика должны получить воспроизводимый вертикальный срез:

```text
/vehicle/front_bogie_velocity ─┐
/vehicle/rear_bogie_velocity  ─┼─► normalization ─► robust EKF ─► Pathgraph
/vehicle/driver_position_cmd  ─┘                         │
                                                        ├─► /result/velocity
optional GNSS ─► base_link/map matching ─► correction ──┘   /result/position
```

Результат должен:

- работать в ROS 2 Humble без GNSS;
- принимать реальные топики организаторов;
- переводить скорость колёс из км/ч в м/с;
- оценивать ускорение, скорость и положение;
- учитывать проскальзывание и отказы колёсных каналов;
- использовать Pathgraph и continuous MGRS `37UCB`;
- поддерживать начальную и периодическую GNSS-коррекцию;
- воспроизводиться на выданных rosbag;
- выдавать метрики относительно GNSS-derived reference;
- иметь документацию сборки, запуска, модели и ограничений.

### 1.1. Фактическое состояние на момент старта

| Компонент | Состояние сейчас | Что ещё требуется |
|---|---|---|
| `wheel-hold` baseline | Реализован и протестирован | Оставить для сравнения |
| Robust EKF `[s,v,a,d]` | Есть рабочий прототип с covariance | Обобщить correction API и откалибровать |
| Модель тягового привода | Есть только формула `kt/kb/dead-zone/v_sat` с начальными коэффициентами | Идентифицировать `tau` и traction/brake tables по данным |
| Продольная динамика | Есть лаг привода и сопротивление `c1/c2` | Добавить `grade(s)`, vehicle profiles и калибровку noise |
| Проскальзывание/отказы | Есть innovation gate, median, freeze, jump и reacquisition | Разделить состояния slip/slide/fault и откалибровать пороги |
| Универсальный IO | CSV/JSON/JSONL/WebSocket и profile normalization реализованы | Подключить реальные organizer topics и `/3.6` |
| Dataset reader | Читает wheel/control и четыре GNSS-топика | Построить синхронизированный reference builder |
| Pathgraph | Есть pandas-инструменты анализа и viewer | Нет production-библиотеки `pose_at/project` |
| ROS | Есть replay, adapter, estimator и telemetry bridge на внутренних `/tram/*` | Нет прямых `/vehicle/*`, `/result/*` и GNSS policies |
| Failure Lab | Есть синтетика, evaluator и benchmark пяти вариантов | Нет split реальных bag, GNSS masking и map metrics |
| API/web | Базовый стенд реализован | Добавить карту, health, corrections и новые метрики |
| Идентификация/калибровка | Не реализована | Нужны identification/validation/test и versioned model YAML |

Иными словами, текущий код — хороший алгоритмический каркас, но не готовая модель
конкретного трамвая. Числа `kt`, `kb`, `tau`, `c1`, `c2`, noise и gates пока являются
стартовыми предположениями, а не результатом идентификации по датасету организаторов.

## 2. Правила параллельной работы

Создать две ветки от одного commit:

```text
feature/runtime-estimator
feature/geometry-reference
```

Разработчик 1 является интегратором. Только он меняет корневые build-файлы и выполняет
финальное подключение компонентов.

```text
pyproject.toml
uv.lock
compose.yaml
infra/**
README.md
docs/23-parallel-implementation-handoff.md
```

Разработчик 2 не редактирует эти файлы. Если нужна новая зависимость или Compose-сервис,
он указывает требование в PR. Разработчик 1 добавляет его при интеграции.

Оба разработчика:

- не запускают форматтер по чужим каталогам;
- не коммитят `.db3`, notebook outputs, большие отчёты и изображения;
- не переименовывают согласованные публичные типы без синхронизации;
- делают небольшие коммиты с проходящими тестами своей области;
- перед merge обновляют свою ветку относительно общей базы.

## 3. Разработчик 1 — realtime estimator и ROS

Ветка: `feature/runtime-estimator`.

### 3.1. Исключительная зона владения

```text
python/odometry_core/**
python/odometry_io/**
ros2_ws/**
contracts/profiles/**
configs/ros/**
docs/delivery/MODEL.md
docs/delivery/ASSUMPTIONS.md
docs/delivery/BUILD.md
docs/delivery/RUN.md
tests/test_core.py
tests/test_adaptive.py
tests/test_sources.py
tests/integration/test_ros.py
```

Кроме этой зоны разработчик 1 владеет перечисленными в разделе 2 интеграционными
файлами. Он не меняет реализацию geometry, аналитику датасета, Lab, API и web.

### 3.2. Задачи A1 — реальные ROS-контракты

1. Перенести `dataset/tram_vehicle_msgs` в `ros2_ws/src/tram_vehicle_msgs` без изменения
   выданных `.msg`.
2. Реализовать `reserve_odometry_node` с подписками:

```text
/vehicle/front_bogie_velocity
/vehicle/rear_bogie_velocity
/vehicle/driver_position_cmd
```

3. На границе ROS выполнить `wheel_mps = wheel_raw_kmh / 3.6`.
4. Публиковать:

```text
/result/velocity  tram_vehicle_msgs/msg/VelocitySensor
/result/position  nav_msgs/msg/Odometry
/odometry/estimate  odometry_msgs/msg/LongitudinalEstimate
```

5. Использовать timestamp события/ROS time, а не wall clock.
6. Обновить сообщения `odometry_msgs`, добавив как минимум `a_mps2`, режим, признаки
   валидности и диагностические счётчики.

Готово, когда реальный bag публикует `/result/velocity` и `/result/position` с правильными
типами, единицами и частотой не ниже 10 Гц.

### 3.3. Задачи A2 — модель и robust EKF

Этот блок обязан дать все три модели, требуемые заданием. Это не три независимые
нейросети, а единая gray-box модель: физическая структура известна, её параметры и
таблицы идентифицируются по identification-bag, а EKF оценивает состояние во время движения.

#### A2.1. Модель тягового привода

Входы: нормализованная позиция контроллера `u`, скорость `v`, `vehicle_id`. Выход:
запрошенное продольное ускорение `a_cmd` до динамического лага.

```text
a_cmd = traction_table(u, v, vehicle_id),  u > dead_zone
a_cmd = 0,                                 |u| <= dead_zone
a_cmd = brake_table(u, v, vehicle_id),     u < -dead_zone
tau·da_act/dt = a_cmd - a_act
```

Первая обязательная версия — монотонные lookup-таблицы с интерполяцией. Параметрическая
формула `kt/kb/v_sat` остаётся baseline. Таблицы и `tau` идентифицируются офлайн
разработчиком 2 по GNSS-derived `a_ref`, затем сохраняются в versioned YAML. Во время
закрытого теста повторной идентификации нет: runtime только читает готовый профиль.

#### A2.2. Модель продольной динамики

Состояние:

```text
x = [s, v, a_act, d]
```

Модель:

```text
ds/dt     = v
tau·da/dt = a_cmd(u, v, vehicle_id) - a_act
dv/dt     = a_act - c1·v - c2·v·|v| - g·grade(s) + d
dd/dt     = process_noise
```

Идентифицируемые по identification-bag параметры: `tau`, `c1`, `c2`, значения traction/brake tables,
process noise и measurement noise. `grade(s)` поступает из Pathgraph. Масса не должна
оцениваться как надёжно наблюдаемая величина по имеющимся сигналам; её влияние входит в
профиль тяги и медленное возмущение `d`.

Структура уравнений задаётся инженером и не «обучается» как black-box ML. По данным
решается задача идентификации параметров с ограничениями, затем параметры проверяются на
validation-bag и только один раз оцениваются на test-bag.

#### A2.3. Модель/эвристика проскальзывания

Обязательная версия — интерпретируемый state machine по каждому колёсному каналу. Его
признаки: wheel-model innovation, расхождение тележек, производная колёсной скорости,
`u`, знак модельного ускорения, freshness и длительность постоянного значения.

Пороги gate, hysteresis и reacquisition калибруются на identification-bag и синтетических отказах.
GNSS разрешён при подготовке меток и проверке, но не является runtime-признаком. Если
позже появится достаточно надёжных меток, классификатор может заменить только вычисление
health/slip probability; safety gates и fallback `MODEL_ONLY` сохраняются.

Реализовать:

- scalar measurement update с произвольным `H`;
- wheel/GNSS velocity update: `H=[0,1,0,0]`;
- GNSS along-track position update: `H=[1,0,0,0]`;
- раздельные таблицы тяги и торможения по контроллеру и скорости;
- профиль `vehicle_id` с fallback на `default`;
- конечную положительную covariance;
- ограничение состояния возмущения `d`;
- последовательную обработку событий одного timestamp.

Порядок событий одного timestamp:

```text
control → wheels by wheel_id → GNSS velocity → GNSS position → publication
```

### 3.4. Задачи A3 — качество колёс

Для каждого канала рассчитывать:

- freshness и монотонность timestamp;
- freeze, dropout и spike;
- физически допустимую производную;
- innovation относительно модели;
- расхождение тележек;
- контекст тяги/торможения;
- health score с hysteresis и reacquisition.

Состояния:

```text
normal
positive_slip
braking_slide
frozen
dropout
inconsistent
unknown
```

Один плохой канал исключается независимо. При недостоверности обоих каналов estimator
переходит в `MODEL_ONLY`, продолжает прогноз, увеличивает covariance и после
`max_model_only_s` помечает положение недействительным, не заменяя его нулём.

### 3.5. Задачи A4 — GNSS-политики и интеграция geometry

Реализовать параметры:

```text
disabled
initialization_only
intermittent
```

- `disabled` — default, GNSS-подписки не создаются, обязательны `route_id` и `s0`.
- `initialization_only` — GNSS определяет route и `s0`, затем updates блокируются.
- `intermittent` — валидные GNSS могут корректировать `s` и `v` через gates.

Разработчик 1 подключает готовый `odometry_geometry` только через API раздела 5. ROS
сообщения сначала преобразуются в библиотечные типы; geometry не должна знать о ROS.

До merge geometry разработчик 1 продолжает A1–A3 с fake `GeometryPort` в unit-тестах.
Не создавать параллельную реализацию Pathgraph или UTM.

### 3.6. Задачи A5 — runtime и документация

Подготовить:

- launch-файлы и YAML для трёх политик;
- параметры `vehicle_id`, `route_id`, `s0`, путей к модели и Pathgraph;
- bounded queues и счётчики переполнения;
- latency p95/p99/max, CPU и RSS;
- offline `colcon build`;
- `MODEL.md`, `ASSUMPTIONS.md`, `BUILD.md`, `RUN.md`.

Конкурсный launch содержит только обязательную ноду. API, web, evaluator и reference не
попадают в realtime-контур.

## 4. Разработчик 2 — geometry, reference, испытания и viewer

Ветка: `feature/geometry-reference`.

### 4.1. Исключительная зона владения

```text
python/odometry_geometry/**
python/odometry_lab/**
dataset/analysis/**
apps/api/**
apps/web/**
contracts/estimate.schema.json
contracts/report.schema.json
configs/models/**
docs/delivery/VALIDATION.md
docs/delivery/DEMO.md
tests/test_geometry.py
tests/test_gnss_reference.py
tests/test_lab.py
tests/test_api.py
tests/fixtures/**
```

Разработчик 2 не меняет core, IO, ROS, root build/lock/Compose и документы разработчика 1.

Работа выполняется строго в порядке B1 → B2 → B3 → B4. Viewer начинается только после
готовности geometry/reference и базовых метрик.

### 4.2. Задачи B1 — production geometry

Создать пакет `odometry_geometry`:

- runtime только на Python 3.10 standard library;
- без ROS, pandas, numpy и pyproj;
- загрузка двух JSON из `dataset/Pathgraph`;
- стабильные ASCII `route_id`;
- накопленная трёхмерная координата `s`;
- интерполяция `x/y/z/yaw/curvature/grade`;
- `pose_at` и проекция XY на ближайший сегмент;
- учёт heading при различении соседних путей;
- bounded spatial index;
- статусы `MATCHED`, `AMBIGUOUS`, `OUT_OF_GRAPH`.

`pose_at` вне границ бросает `OutOfGraphError`. `project` вне gate возвращает
`OUT_OF_GRAPH` и не прижимает координату к концу маршрута.

### 4.3. Задачи B2 — MGRS и GNSS extrinsics

Реализовать pure-Python WGS84 → UTM zone 37N:

```text
x = easting  - 300000
y = northing - 6100000
```

Modulo `100000` запрещён. Контрольная точка:

```text
lat = 55.8088325462547
lon = 37.4602768500852
x   = 103501.6309 ± 0.02 м
y   =  85876.1201 ± 0.02 м
```

Геометрия:

```text
front bogie = [ 0.000, 0, 0]
rear bogie  = [-7.550, 0, 0]
antenna 1   = [-9.873, 0, 3]
antenna 2   = [ 2.563, 0, 3]
baseline    = 12.436 м
```

Реализовать dual-antenna yaw/base_link, проверку baseline, single-antenna fallback при
известном yaw и body yaw по хорде тележек. Геоидную поправку не применять.

### 4.4. Задачи B3 — GNSS-derived reference и калибровка

В `dataset/analysis/gnss_reference.py` реализовать `SpeedReferenceBuilder`:

1. синхронизация двух GNSS velocity;
2. перевод антенн в `base_link`;
3. проекция скорости на продольную ось/Pathgraph;
4. устойчивый consensus двух источников;
5. проверка производной сглаженной GNSS-позиции;
6. выдача `v_ref`, `s_ref`, uncertainty, coverage и reason codes.

Колёсные данные не использовать как target. Подготовить таблицу:

```text
stamp, vehicle_id, route_id, s_ref, v_ref, a_ref,
controller, front_wheel_mps, rear_wheel_mps, grade, curvature,
reference_available, reference_uncertainty
```

Разделить identification/validation/test целыми bag, не временными строками. На
identification subset определить
`tau`, таблицы тяги/торможения, сопротивление и process-noise. Сохранить:

```text
configs/models/default.yaml
configs/models/30618.yaml
configs/models/30639.yaml
```

Каждый файл содержит версию, метод, identification bags и метрики validation. Test bags не
используются для подбора параметров.

### 4.5. Задачи B4 — Failure Lab и метрики

Расширить CLI и Lab:

- режимы GNSS `never`, `initial`, intermittent 30/60/120 секунд;
- freeze/dropout/spike/slip и GNSS jump;
- один и два отказавших канала;
- выход за Pathgraph и ложный соседний путь;
- batch-run пяти вариантов алгоритма;
- latency/RSS/CPU и incident report.

Сравнить:

1. wheel median после `/3.6`;
2. model-only;
3. EKF без robust gates;
4. robust EKF;
5. robust EKF с GNSS corrections.

Метрики: velocity MAE/RMSE/bias/p95, along-track/XYZ error, final drift, availability,
model-only duration, recovery time, correction jump и reference coverage. При отсутствии
reference метрика имеет статус `unavailable`, а не значение `0`.

### 4.6. Задачи B5 — API, web и документация

После B1–B4 обновить schema до `0.3` и viewer:

- Pathgraph, GNSS reference и estimate XY;
- контроллер, два колеса, скорость, ускорение и путь;
- uncertainty, filter mode и wheel health;
- принятые/отклонённые GNSS corrections;
- incidents и итоговые метрики.

Web только читает сохранённые и live-результаты. Он не запускает процессы и не входит в
ROS launch. Подготовить `VALIDATION.md` и `DEMO.md`.

## 5. Замороженный контракт между разработчиками

Разработчик 2 экспортирует из `odometry_geometry`:

```python
@dataclass(frozen=True)
class ProjectionConfig:
    utm_zone: int = 37
    northern: bool = True
    false_easting_m: float = 300000.0
    false_northing_m: float = 6100000.0

@dataclass(frozen=True)
class TramGeometry:
    bogie_base_m: float = 7.55
    antenna_1_xyz_m: tuple[float, float, float] = (-9.873, 0.0, 3.0)
    antenna_2_xyz_m: tuple[float, float, float] = (2.563, 0.0, 3.0)

@dataclass(frozen=True)
class MapPoint:
    x_m: float
    y_m: float
    z_m: float

@dataclass(frozen=True)
class BaseLinkPose:
    x_m: float
    y_m: float
    z_m: float
    yaw_rad: float
    baseline_error_m: float | None

@dataclass(frozen=True)
class TrackPose:
    route_id: str
    s_m: float
    x_m: float
    y_m: float
    z_m: float
    yaw_rad: float
    curvature_inv_m: float
    grade: float

@dataclass(frozen=True)
class TrackMatch:
    route_id: str | None
    s_m: float | None
    cross_track_m: float | None
    heading_error_rad: float | None
    status: str
```

Функции:

```python
project_wgs84(latitude_deg, longitude_deg, altitude_m, config) -> MapPoint

PathGraph.from_directory(path) -> PathGraph
PathGraph.route_ids -> tuple[str, ...]
PathGraph.length_m(route_id) -> float
PathGraph.pose_at(route_id, s_m) -> TrackPose
PathGraph.body_pose_at(route_id, front_s_m, bogie_base_m=7.55) -> TrackPose
PathGraph.project(x_m, y_m, yaw_rad=None, route_id=None) -> TrackMatch

base_link_from_dual_antenna(point_1, point_2, geometry) -> BaseLinkPose
base_link_from_single_antenna(point, yaw_rad, antenna_xyz_m) -> MapPoint
```

Ошибки: неверный JSON/config/числа → `ValueError`; неизвестный route → `KeyError`;
`pose_at` вне диапазона → `OutOfGraphError`; проекция вне gate → статус
`OUT_OF_GRAPH`.

Разработчик 1 экспортирует из core:

```python
@dataclass(frozen=True)
class AlongTrackPositionCorrection:
    stamp_ns: int
    seq: int
    route_id: str
    s_m: float
    variance_m2: float
    source: str

@dataclass(frozen=True)
class LongitudinalVelocityCorrection:
    stamp_ns: int
    seq: int
    v_mps: float
    variance_m2ps2: float
    source: str

estimator.ingest_position_correction(sample)
estimator.ingest_velocity_correction(sample)
```

Geometry не импортирует core. Core не импортирует geometry. Их связывает только ROS-нода
разработчика 1.

### 5.1. Контракт артефакта параметров модели

Разработчик 2 создаёт YAML, разработчик 1 реализует его строгую загрузку. Неизвестные
поля, несовпадающие размеры таблиц и нечисловые значения являются ошибкой запуска.

```yaml
schema_version: 1
model_version: tram-graybox-v1
vehicle_id: default
identified_at_utc: null  # заполняется фактическим временем идентификации
identification_method: constrained_least_squares
identification_bags: []
validation_bags: []

longitudinal:
  tau_s: 0.3
  c1_inv_s: 0.01
  c2_inv_m: 0.0
  disturbance_limit_mps2: 1.5

traction:
  controller_u: [0.1, 0.5, 1.0]
  speed_mps: [0.0, 5.0, 10.0, 20.0]
  acceleration_mps2: []  # matrix len(controller_u) x len(speed_mps)

braking:
  controller_u: [-0.1, -0.5, -1.0]
  speed_mps: [0.0, 5.0, 10.0, 20.0]
  acceleration_mps2: []  # отрицательные значения, та же размерность

noise:
  q_v: 0.01
  q_a: 0.05
  q_d: 0.001
  wheel_variance_floor: 0.04

wheel_health:
  gate_normal: 9.0
  gate_reject: 36.0
  max_wheel_accel_mps2: 6.0
  freeze_s: 1.0
  reacquire_s: 10.0
  recover_updates: 5

metrics:
  reference_coverage: null
  validation_velocity_rmse_mps: null
```

Интерполяция таблиц билинейная. За границами скорости используется ближайшая граничная
ячейка, а позиция контроллера предварительно ограничивается допустимым диапазоном. Знак
ускорения в braking table отрицательный. Если профиль конкретного `vehicle_id` отсутствует,
загружается `default`; молчаливый переход на hardcoded коэффициенты запрещён.

## 6. Что оба начинают делать сейчас

### Разработчик 1 — первые четыре коммита

1. `ros: vendor tram_vehicle_msgs and subscribe organizer topics`
2. `io: normalize organizer wheel velocity from kmh to mps`
3. `ros: publish required result velocity and position topics`
4. `core: add generic scalar correction API and tests`

После четвёртого коммита продолжить wheel health и модель, не ожидая geometry.

### Разработчик 2 — первые четыре коммита

1. `geometry: scaffold package and load directed pathgraph routes`
2. `geometry: implement pose interpolation and nearest projection`
3. `geometry: add continuous mgrs projection and organizer control point`
4. `geometry: add antenna transforms and synthetic tests`

После четвёртого коммита открыть первый PR. Пока разработчик 1 интегрирует geometry,
разработчик 2 продолжает reference и Lab в своей ветке или новой ветке от merged main.

## 7. Порядок merge без конфликтов

### Merge 1 — geometry foundation

Первым вливается PR разработчика 2 только с:

```text
python/odometry_geometry/**
tests/test_geometry.py
```

Разработчик 1 добавляет пакет в root workspace/lock и проверяет импорт в ROS container.

### Merge 2 — organizer ROS vertical slice

Вливается PR разработчика 1: реальные inputs, `/3.6`, `/result/*`, core correction API и
smoke-test без GNSS.

### Merge 3 — GNSS integration

Разработчик 1 ребейзит свою integration-ветку и соединяет ROS GNSS → geometry → core.
Разработчик 2 не редактирует интеграционный adapter, но исправляет geometry при падении
его контрактных тестов.

### Merge 4 — reference, calibration и Lab

Вливаются артефакты разработчика 2: reference builder, model profiles, split, сценарии и
метрики. Разработчик 1 проверяет загрузку model YAML в runtime.

### Merge 5 — viewer и сдача

Разработчик 2 вливает API/web/validation/demo. Разработчик 1 завершает runtime docs,
offline build, launch и общую репетицию.

## 8. План на 3–4 дня

| День | Разработчик 1 | Разработчик 2 | Общий результат |
|---|---|---|---|
| 1 | Реальные ROS inputs, `/3.6`, `/result/*`, correction API | Pathgraph, `pose_at/project`, UTM/MGRS, extrinsics | Два независимых зелёных PR |
| 2 | Модель, EKF, wheel health, GNSS policies | Reference builder, split, первичная калибровка | Полный bag проходит через estimator |
| 3 | Интеграция geometry/GNSS, launch, performance | Failure Lab, метрики, viewer | Validation на выбранных bag и demo |
| 4 | Offline build, runtime docs, исправления | Validation/demo docs, исправления | Репетиция сдачи на чистом окружении |

Общая синхронизация дважды в день по 15 минут: только изменения контрактов, блокеры и
результаты тестов. Обсуждение внутренней реализации не должно блокировать независимую
работу.

## 9. Тестовые прогоны

| Bag | Назначение |
|---|---|
| `30618_af7496f0` | первый чистый сквозной прогон |
| `30618_2050d396` | сильное расхождение тележек и rejection канала |
| `30618_27e994fc` | 31,6 минуты, drift и memory stability |
| `30639_e4379d7f` | второй трамвай и частичный GNSS |
| любой bag без GNSS | обязательный `disabled` с route start |

Unit-проверки:

- `3.6 км/ч → 1 м/с`, `36 → 10`;
- контрольная MGRS-точка с допуском 0,02 м;
- отсутствие modulo-скачка;
- TF антенн при yaw `0`, `π/2`, `π`;
- dual baseline `12.436 м`;
- `project(pose_at(s))` восстанавливает `s` с ошибкой не более 0,05 м;
- соседние пути различаются по heading;
- scalar EKF updates для `s` и `v`;
- freeze/dropout/spike/slip/reacquisition;
- covariance конечна и положительна.

Интеграционные проверки:

- запуск без GNSS;
- GNSS только в начале;
- GNSS впервые в середине;
- corrections через 30/60/120 секунд;
- GNSS outlier и соседний путь;
- отказ одной/двух антенн и одного/двух колёс;
- выход за Pathgraph;
- API и web выключены, ROS продолжает работать.

## 10. Критерии готовности

Функциональность:

1. `/result/velocity` и `/result/position` имеют ожидаемые типы и timestamps.
2. Position относится к `base_link`, frame — `map`, координаты — continuous MGRS.
3. Отсутствие GNSS не останавливает estimator.
4. Подозрительный wheel channel исключается независимо.
5. GNSS outlier не вызывает смену route или неконтролируемый скачок.
6. `OUT_OF_GRAPH` не маскируется крайней точкой.
7. Robust EKF не хуже wheel median на чистых bag и лучше на `30618_2050d396`.

Производительность обязательного ROS-контура:

```text
output rate       >= 10 Гц, цель 50 Гц
latency p99       <= 100 мс
single peak       <= 250 мс
CPU               <= 2 cores
RSS               <= 0.5 ГБ
queues            bounded
colcon build      без сети
```

Сдаваемые документы:

```text
docs/delivery/MODEL.md
docs/delivery/ASSUMPTIONS.md
docs/delivery/BUILD.md
docs/delivery/RUN.md
docs/delivery/VALIDATION.md
docs/delivery/DEMO.md
```

Демонстрация:

```text
normal movement
→ wheel disagreement/slip
→ rejection подозрительного колеса
→ model-only
→ исчезновение GNSS
→ продолжение оценки
→ optional GNSS recovery
→ метрики и графики
```

## 11. Зафиксированные допущения

- входные колёса — км/ч, выходная скорость — м/с;
- `base_link` находится на оси передней тележки;
- положение — continuous MGRS относительно `37UCB`;
- `map`: x восток, y север, z вверх;
- геоидная поправка до подтверждения datum не применяется;
- закрытый тест без GNSS получает направленный `route_id`, старт — первая точка JSON;
- default GNSS policy — `disabled`;
- `/localization/kinematic_state` не является обязательным выходом;
- web не участвует в realtime-контуре;
- онлайн-ML не входит в обязательную реализацию;
- ошибка без reference имеет статус `unavailable`, а не нулевое значение.

## 12. Definition of Done для каждого PR

В PR обязательно указать:

1. реализованные пункты этого плана;
2. изменённые публичные контракты или `none`;
3. команды и фактический результат тестов;
4. использованные bag и режим GNSS;
5. известные ограничения;
6. действия, необходимые интегратору.

PR не готов к merge, если он меняет чужую зону владения, содержит большие данные,
скрывает недоступные метрики нулями или требует сети для обязательной ROS-сборки.
