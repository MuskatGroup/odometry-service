# Ядро EKF, API по контракту и ROS-обвязка

Дополняет [docs/11](11-adapters-and-integration.md). Метрики, сценарии отказов, бенчмарк и `probe` описаны в [docs/13](13-metrics-scenarios-and-probe.md). Реализация — в `src/odometry_io` и `ros2_ws/`. Проектные документы 01–10 не менялись; здесь описано только то, что реально есть в коде.

## Что реализовано

- `model.py` — `ModelEstimator`: EKF с состоянием `[s, v, a_act, d]` по [docs/01](01-solution-hypothesis.md). Прогноз с подшагами, форма Джозефа, отказоустойчивое слияние колес: гейт по нормированной невязке (`gate_normal` — ослабление доверия, `gate_reject` — отказ), проверка скачка, гистерезис возврата в `FUSED` (`recover_updates`). `a_act` обновляется только в `FUSED`. Адаптация возмущения `d` по умолчанию выключена (`adapt_disturbance=false`, см. docs/13); добавлены детектор зависания и голосование по каналам. Режимы `INITIALIZING / FUSED / DEGRADED / MODEL_ONLY / INVALID`, причины в `reason_codes`.
- `api.py` — фасад контракта [docs/03](03-contracts.md): `OdometryEstimator.initialize / ingest_control / ingest_wheel / advance_to / reset`, `Estimate` с ковариацией 4×4, `sigma`, возрастами входов и счетчиками. Поверх `LiveSession` (ограниченная очередь, поздние и повторные события).
- `ros_bridge.py` — код без ROS: словарь сообщения → `Event` → estimator. Поля, единицы, радиусы и кодировка ручки берутся из того же профиля `adapter-1`, что и для файлов; у каждого канала указывается `topic`. Независимый `tick()` работает и при молчании источника.
- `ros2_ws/src/odometry_node` — тонкая rclpy-оболочка (`estimator_node`, launch, пример конфигурации); `ros2_ws/src/odometry_msgs` — сообщения `ControlSample`, `WheelSample`, `LongitudinalEstimate` по контракту.
- CLI: `--estimator model [--model-config params.json]`; по умолчанию остается baseline `hold`, поэтому один и тот же запуск можно сравнить с B0.

## Запуск сравнения на синтетике

```powershell
$env:PYTHONPATH = "$PWD\src"
python -m odometry_io.emulator --config examples/emulator.json --output work/syn
python -m odometry_io --input work/syn/input.jsonl --format jsonl --profile examples/profile-emulator.json --truth work/syn/truth.jsonl --estimator hold  --output work/run-hold
python -m odometry_io --input work/syn/input.jsonl --format jsonl --profile examples/profile-emulator.json --truth work/syn/truth.jsonl --estimator model --model-config work/model.json --output work/run-model
```

На эмуляторе с проскальзыванием и пропуском (seed 17) RMSE скорости 0.35 → 0.10 м/с, RMSE пути 0.68 → 0.13 м. **Эмулятор использует те же уравнения, что и модель**, поэтому это проверка реализации, а не оценка качества на реальном трамвае.

## Что нужно знать перед реальными данными

- Все параметры `ModelConfig` (`kt`, `kb`, `tau_s`, шумы, пороги) — стартовые, не откалиброванные. Автокалибровки нет.
- `sigma_*` берется из ковариации, но покрытие не проверялось; в `reason_codes` всегда есть `UNCERTAINTY_UNCALIBRATED`.
- `control_timeout_s=None`: последнее положение ручки удерживается бессрочно, потому что частота публикации ручки неизвестна. Как только она известна, задайте таймаут.
- Ошибка общего проскальзывания всех колес незаметна для фильтра (предел наблюдаемости из docs/01). Гейт защищает от отдельных выбросов и резких скачков.
- При разрыве больше `max_gap_s` фильтр сбрасывается (`GAP_RESET`); позиция сохраняется, ее `sigma` начинается заново.
- Сообщение по одному колесу с полем `wheel_id` (как в `WheelSample` контракта) профилем пока не разделяется на каналы по значению поля: нужен один топик на канал либо массив в одном сообщении.
- Отсутствуют адаптация коэффициентов `kt/kb` и нелинейная поправка. Зависание датчика детектируется (`WHEEL_FROZEN`), общее буксование нет.

## ROS: статус

Проверено 24 сентября 2026 в контейнере `ros:humble-ros-base` (ROS 2 Humble, Ubuntu 22.04, Python 3.10), репозиторий смонтирован только для чтения:

- `colcon build` собирает `odometry_msgs` (генерация сообщений) и `odometry_node`;
- нода стартует из launch, принимает `ControlSample` и `WheelSample` (`ros2 topic pub`, `header: auto`) и публикует `/odometry/estimate` и `/odometry/filtered`: при 8 м/с скорость 7.9998 м/с, путь растет, `mode=FUSED`, `valid=true`;
- при остановке потока колес таймер продолжает публикацию: через 2 с `MODEL_ONLY` (`WHEEL_STALE`, `sigma_s` растет), через ~11 с `INVALID` (`MODEL_ONLY_HORIZON`), `/odometry/filtered` в `INVALID` не публикуется.

**Bag-replay с `use_sim_time`** ([ros2_ws/tools/bag_replay_check](../ros2_ws/tools/bag_replay_check/README.md)): rosbag из эмулятора (время от 100 с, проскальзывание и пропуск колес) проигран через `ros2 bag play --clock`, нода работала на симуляционном времени. Оценки идут со штампами bag, режимы переключаются в нужные моменты: пропуск колес 6.0 с → `MODEL_ONLY` в 6.47 с (таймаут 0.5 с + 20 мс окно), возврат колес 7.5 с → `DEGRADED` → `FUSED` в 7.72 с (после серии согласованных отсчетов); проскальзывание 11.0 с → `DEGRADED` (12 → 12.27 с `FUSED`). RMSE скорости в окне проскальзывания 0.155 м/с при ошибке сырого колеса 3.0 м/с; в конце пути 150.7 м против эталона 151.5 м. Эталон построен на тех же уравнениях, что и модель: проверена интеграция, а не точность. `/clock` от `ros2 bag play` идет с частотой 40 Гц, поэтому таймер на симуляционном времени дает около 40 кадров/с вместо 50; частота `/clock` задается аргументом `--clock <Гц>` (запуск с большей частотой не проверялся).

Не проверялось: реальные типы организаторов, нагрузка и задержка, `colcon test`, C++.

Порядок сборки в Ubuntu 22.04 + Humble. `PYTHONPATH` с исходниками ядра задавайте **после** `colcon build` и `source install/setup.bash`: до сборки он ломает CMake. Либо установите ядро через `pip install <repo>`.

```bash
cd ros2_ws && colcon build && source install/setup.bash
export PYTHONPATH=/path/to/odometry-service/src:$PYTHONPATH
ros2 launch odometry_node estimator.launch.py config:=src/odometry_node/config/example.json
```

Типы и имена топиков в `config/example.json` — контракт v0.1 (`odometry_msgs/msg/ControlSample`, `WheelSample`). Для сообщений организаторов достаточно поменять `inputs` и пути полей профиля. Для проигрывания bag с `--clock` включите `use_sim_time:=true`; `stamp_source` выбирает `header.stamp` или время приема.
