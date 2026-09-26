# План параллельной реализации и передачи задач

Дата фиксации: 26 сентября 2026.

Статус: **утверждён для реализации**.

Этот документ — рабочая точка синхронизации команды и достаточное техническое задание
для второго участника. Другие документы читать до начала работы не требуется. Навигация
и статус всей документации зафиксированы в [`README.md`](README.md).

Подробное обоснование отдельных решений, если оно понадобится, находится в:

- [`20-implementation-plan-intermittent-gnss.md`](20-implementation-plan-intermittent-gnss.md);
- [`21-coordinate-frame-and-gnss-extrinsics.md`](21-coordinate-frame-and-gnss-extrinsics.md);
- [`22-organizer-answers-and-decisions.md`](22-organizer-answers-and-decisions.md).

Если документы расходятся, приоритет имеют зафиксированные контракты и распределение
файлов из этого handoff. Старые документы не добавляют задачи и не изменяют область
ответственности участника.

## 1. Цель ближайшей итерации

За 3–4 дня получить конкурсный вертикальный срез:

```text
/vehicle/front_bogie_velocity ─┐
/vehicle/rear_bogie_velocity  ─┼─► SI normalization ─► robust EKF ─► Pathgraph
/vehicle/driver_position_cmd  ─┘                              │
                                                             ├─► /result/velocity
optional GNSS ─► base_link/map matching ─► gated correction ─┘   /result/position
```

Обязательный runtime:

- ROS 2 Humble;
- работа без GNSS;
- скорость колёс переводится из км/ч в м/с;
- непрерывная MGRS-позиция относительно `37UCB`;
- `base_link` находится на оси передней тележки;
- `/result/velocity` и `/result/position` публикуются не реже 10 Гц, цель 50 Гц;
- p99 input → publication не более 100 мс, единичный пик не более 250 мс;
- обязательный контур укладывается в 2 CPU и 0,5 ГБ RAM;
- `colcon build` не требует сети.

Web/API являются отдельным инструментом просмотра прогонов и не входят в обязательный
runtime или его ресурсные ограничения.

## 2. Замороженные решения

### 2.1. GNSS-политики

```text
disabled
initialization_only
intermittent
```

- `disabled` — конкурсный default, GNSS-подписки не создаются.
- `initialization_only` — GNSS задаёт `route_id/s0`, затем исключается из фильтра.
- `intermittent` — явно включаемая периодическая gated-коррекция.

В `disabled` обязателен направленный `route_id`; старт соответствует первой точке JSON,
`s0=0`.

### 2.2. Координаты и геометрия

```text
WGS84 → UTM 37N
x = easting  - 300000
y = northing - 6100000
```

- modulo `100000` запрещён;
- `x` может быть больше `100000`;
- `map`: x восток, y север, z вверх;
- `base_link`: x вперёд, y влево, z вверх.

```text
front bogie = [ 0.000, 0, 0]
rear bogie  = [-7.550, 0, 0]
antenna 1   = [-9.873, 0, 3]
antenna 2   = [ 2.563, 0, 3]
baseline    = 12.436 м
```

До уточнения vertical datum геоидная поправка не применяется.

### 2.3. Математическая модель

```text
x = [s, v, a_act, d]

ds/dt     = v
tau·da/dt = a_cmd(u, v, vehicle_id) - a_act
dv/dt     = a_act - c1·v - c2·v·|v| - g·grade(s) + d
dd/dt     = process_noise
```

Колёсные измерения поступают в core только в SI:

```text
wheel_mps = wheel_raw_kmh / 3.6
```

Свободное состояние масштаба `3.6` в EKF не добавляется.

### 2.4. Обязательные ROS-выходы

```text
/result/velocity  tram_vehicle_msgs/msg/VelocitySensor
/result/position  nav_msgs/msg/Odometry
```

`Odometry`:

```text
header.frame_id = map
child_frame_id  = base_link
pose.position   = continuous MGRS base_link
pose.orientation = body yaw quaternion
twist.linear.x  = v, м/с
```

Ускорение публикуется в диагностическом `/odometry/estimate` через `a_mps2`.

## 3. Потоки разработки и владение файлами

### Поток A — core и обязательный ROS runtime

Рекомендуемый владелец: основной разработчик/архитектор.

Зона владения:

```text
python/odometry_core/**
python/odometry_io/**
ros2_ws/**
contracts/profiles/**
tests/test_core.py
tests/test_adaptive.py
tests/integration/test_ros.py
infra/Dockerfile.ros
```

Задачи:

- реальные подписки `/vehicle/*`;
- km/h → m/s на входной границе;
- три GNSS-политики;
- расширение EKF correction API;
- robust wheel health/slip/reacquisition;
- обязательные `/result/*`;
- launch/config и offline build;
- latency/resource instrumentation.

Поток A не редактирует внутреннюю реализацию `odometry_geometry`, а использует её
публичный API из раздела 4.

### Поток B — geometry и GNSS reference

Рекомендуемый владелец: **второй участник команды**.

Ветка:

```text
feature/geometry-gnss
```

Зона владения:

```text
python/odometry_geometry/**
dataset/analysis/gnss_reference.py
tests/test_geometry.py
tests/test_gnss_reference.py
```

Допустимые общие изменения в этом потоке:

```text
pyproject.toml
uv.lock
python/odometry_geometry/pyproject.toml
```

Другие потоки не меняют эти файлы до первого merge потока B.

Задачи потока B:

#### B1. Новый пакет `odometry_geometry`

- только Python 3.10 standard library в runtime;
- без ROS, pandas, numpy и pyproj;
- загрузка двух JSON Pathgraph;
- численная валидация и стабильные ASCII route IDs;
- накопленная трёхмерная координата `s`;
- интерполяция `x/y/z/yaw/curvature/grade`;
- проекция XY на ближайший сегмент;
- статусы `MATCHED/AMBIGUOUS/OUT_OF_GRAPH`;
- bounded spatial index для быстрого поиска.

#### B2. Проекция WGS84 → continuous `37UCB`

- pure-Python forward UTM zone 37N;
- именованные offsets `300000/6100000`;
- отсутствие modulo при переходе easting `400000`;
- контрольная точка организаторов.

#### B3. GNSS extrinsics

- dual-antenna yaw и `base_link`;
- проверка baseline `12.436 м`;
- single-antenna fallback при известном yaw;
- компенсация `z - 3.0 м` без геоидной поправки;
- chord yaw по передней/задней тележке при отсутствии GNSS.

#### B4. Offline `SpeedReferenceBuilder`

- синхронизация двух GNSS velocity;
- проекция на продольную ось/касательную;
- устойчивый consensus без использования колёс как target;
- проверка производной сглаженной позиции `base_link`;
- `available`, uncertainty и reason codes;
- экспорт reference для evaluator.

#### Готовность потока B

- публичный API соответствует разделу 4;
- unit-тесты проходят без ROS;
- контрольная MGRS-точка совпадает с допуском 0,02 м;
- `project(pose_at(s))` восстанавливает `s` с ошибкой не более 0,05 м на внутренних
  точках маршрута;
- dual-antenna synthetic tests восстанавливают base pose;
- `ruff check` и `pytest` зелёные;
- PR не содержит изменений EKF или ROS-ноды.

### Поток C — Failure Lab и проверка

Владелец: Python-автоматизатор.

Зона владения:

```text
python/odometry_lab/**
tests/test_lab.py
tests/fixtures/**
artifacts/ (только генерируемые результаты, не коммитить большие файлы)
```

Задачи:

- фиксированный train/validation/test split целыми bag;
- маскирование GNSS: never/initial/30/60/120 секунд;
- сценарии freeze/dropout/spike/slip/GNSS jump;
- batch-run и ablation;
- latency/RSS/CPU отчёт;
- инциденты человеческим языком.

### Поток D — API и web-viewer

Владельцы: full-stack участники.

Зона владения:

```text
apps/api/**
apps/web/**
contracts/estimate.schema.json
contracts/report.schema.json
```

Задачи:

- schema version `0.3`;
- отображение Pathgraph, GNSS reference и estimate XY;
- скорость, ускорение, путь, колёса, controller и uncertainty;
- wheel health, GNSS corrections и incidents;
- сохранённые и live-графики;
- UI не запускает и не останавливает процессы.

Поток D не добавляет зависимости API/web в ROS launch.

## 4. Контракт между потоками A и B

Пакет `odometry_geometry` экспортирует:

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

Публичные функции:

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

Правила ошибок:

- неверный JSON/config → `ValueError` при запуске;
- неизвестный route → `KeyError`;
- `pose_at` вне диапазона → отдельный `OutOfGraphError`, без clamp;
- невалидные GNSS числа → `ValueError`, без NaN в результате;
- `project` вне gate возвращает `OUT_OF_GRAPH`, а не exception.

Core не импортируется из geometry. ROS-сообщения не передаются в geometry. Поток A
сам преобразует `TrackMatch` в correction-события core.

## 5. Контракт core correction API

Поток A реализует:

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
```

Estimator API:

```python
ingest_position_correction(sample)
ingest_velocity_correction(sample)
```

Порядок событий с одинаковым timestamp:

```text
control → wheels by wheel_id → GNSS velocity → GNSS position → publication
```

Estimate/telemetry дополнить:

```text
a_mps2
route_id
map_x_m/map_y_m/map_z_m/yaw_rad
wheel_health by channel
gnss_policy
gnss_age_s
accepted/rejected GNSS correction counts
reason_codes
```

Поток B не реализует эти типы и не меняет core.

## 6. Порядок merge и интеграционные точки

### Merge 1 — geometry

Поток B предоставляет полностью зелёный независимый пакет и тесты. После merge поток A
добавляет dependency `odometry-geometry` в ROS/package wiring.

### Merge 2 — organizer IO и `/result/*`

Поток A подключает реальные топики, единицы и выходной контракт без GNSS corrections.

### Merge 3 — model/EKF

Поток A добавляет correction API, robust wheel fusion и откалиброванный model config.

### Merge 4 — GNSS integration

Потоки A и B совместно подключают ROS GNSS → geometry → core corrections. Default
`gnss_policy=disabled` не создаёт GNSS subscriptions.

### Merge 5 — Failure Lab и viewer

Потоки C/D подключают новые поля только после фиксации telemetry schema `0.3`.

Перед каждым merge:

```powershell
uv run --group dev --group analysis pytest -q
uv run --group dev ruff check python tests dataset/analysis
docker compose --profile test build ros-tests
docker compose --profile test run --rm ros-tests
```

Форматтеры не запускать по всему репозиторию в feature-ветках: менять только файлы
своего потока.

## 7. Общие тестовые прогоны

| Bag | Назначение |
|---|---|
| `30618_af7496f0` | чистый первый сквозной прогон |
| `30618_2050d396` | сильное расхождение тележек, slip/disagreement |
| `30618_27e994fc` | 31,6 минуты, drift и memory stability |
| `30639_e4379d7f` | второй трамвай и частичный GNSS |
| любой bag без GNSS | обязательный `disabled/route_start` |

Общие acceptance checks:

1. Raw колёса после IO имеют отношение к GNSS reference около `1`, не `3.6`.
2. `/result/velocity` и `/result/position` имеют правильные типы и timestamps.
3. Частота обоих выходов ≥10 Гц.
4. Отсутствие GNSS не останавливает estimator.
5. Подозрительный wheel channel отклоняется отдельно.
6. Выброс GNSS не переключает route и не создаёт неконтролируемый скачок.
7. `OUT_OF_GRAPH` не маскируется крайней точкой.
8. Web/API остановлены — ROS продолжает работать.
9. Длинный bag не показывает монотонного роста памяти.
10. Ошибка/метрика без reference обозначается `unavailable`, а не нулём.

## 8. Status board

Обновлять чекбоксы только в интеграционной ветке после merge PR.

### Поток A

- [ ] A1. Реальные organizer inputs и `/3.6`
- [ ] A2. `/result/velocity`
- [ ] A3. `/result/position`
- [ ] A4. Три GNSS-политики
- [ ] A5. EKF correction API
- [ ] A6. Robust wheel fusion
- [ ] A7. ROS integration/performance tests

### Поток B — второй участник

- [ ] B1. Пакет `odometry_geometry`
- [ ] B2. Pathgraph loader и `pose_at`
- [ ] B3. `project` и spatial index
- [ ] B4. WGS84 → continuous `37UCB`
- [ ] B5. Dual/single antenna transforms
- [ ] B6. Body chord yaw
- [ ] B7. `SpeedReferenceBuilder`
- [ ] B8. Unit tests и handoff notes

### Поток C

- [ ] C1. Dataset split
- [ ] C2. GNSS masking
- [ ] C3. Failure scenarios
- [ ] C4. Batch metrics и performance report

### Поток D

- [ ] D1. Telemetry/report schema `0.3`
- [ ] D2. API storage новых полей
- [ ] D3. XY Pathgraph/reference/estimate viewer
- [ ] D4. Графики acceleration/health/corrections

### Сдача

- [ ] Математическая модель описана
- [ ] Допущения и ограничения описаны
- [ ] Offline build проверен
- [ ] Инструкция запуска проверена на чистом окружении
- [ ] Все параметры задокументированы
- [ ] Demo на rosbag воспроизводим
- [ ] Accuracy/reference report сохранён
- [ ] Performance/resource report сохранён

## 9. Правила передачи результата второго участника

В PR потока B приложить:

1. краткое описание реализованных B-задач;
2. перечень публичных символов пакета;
3. команды запуска тестов;
4. фактические результаты контрольной MGRS-точки и `project(pose_at(s))`;
5. известные ограничения;
6. пример интеграции без ROS:

```python
graph = PathGraph.from_directory(pathgraph_dir)
point = project_wgs84(latitude, longitude, altitude, ProjectionConfig())
match = graph.project(point.x_m, point.y_m, route_id=route_id)
pose = graph.body_pose_at(route_id, match.s_m)
```

PR не должен включать generated datasets, notebook outputs, `.db3`, большие PNG или
изменения чужих потоков. После открытия PR второй участник остаётся владельцем исправлений
своего пакета до прохождения интеграционных тестов Merge 4.
