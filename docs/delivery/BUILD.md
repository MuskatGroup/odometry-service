# Сборка обязательного ROS-контура

Целевая среда: Ubuntu 22.04, ROS 2 Humble, Python 3.10.

## Docker

Перед сборкой Docker daemon должен быть запущен:

```bash
docker compose --profile test build ros-tests
docker compose --profile test run --rm ros-tests
```

Dockerfile устанавливает системные ROS-зависимости до копирования workspace, затем
собирает `tram_vehicle_msgs`, `odometry_msgs`, `odometry_python` и `odometry_node` через `colcon`.
`odometry_python` устанавливает core/geometry/io из единственных исходников в `python/`,
а также model YAML и Pathgraph в package share. Отдельный `pip install` наших библиотек не нужен.

## Нативная Humble-среда

```bash
# Один раз, на этапе подготовки среды (требует сети):
sudo apt-get install python3-colcon-common-extensions python3-yaml python3-websockets \
  ros-humble-nav-msgs ros-humble-diagnostic-msgs ros-humble-rosbag2 \
  ros-humble-rosbag2-storage-default-plugins
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build
source install/setup.bash
ros2 pkg executables odometry_node
```

В списке должен присутствовать `odometry_node reserve_odometry_node`.

Сборка Docker-образа требует сети для получения базового образа и `apt`. После подготовки
среды `colcon build` и runtime работают без сети. Клонировать нужно весь репозиторий:
`ros2_ws/src/odometry_python` использует соседние `python/`, `configs/` и `dataset/Pathgraph/`.
Launch-файл сам находит установленные профили и карту через ament index.

Чистая сборка без сети (не использует старые build/install):

```bash
docker compose --profile reserve build reserve
docker run --rm --network none --entrypoint /bin/bash odometry-ros:validation -lc \
  'source /opt/ros/humble/setup.bash && cd /workspace/ros2_ws && colcon --log-base /tmp/offline-log build --build-base /tmp/offline-build --install-base /tmp/offline-install --executor sequential'
```

## Локальные проверки Python

```bash
uv sync --locked
uv run pytest -q
uv run ruff check python tests ros2_ws/src/odometry_node ros2_ws/src/odometry_python tools
```

На машине без ROS восемь integration-тестов будут пропущены. Также opt-in API-тесты требуют
`RUN_API_TESTS=1` и .NET SDK; два real-data теста требуют созданного `dataset/derived`.
Код внешнего checker в `tests/check-code` исключён из обычного pytest/lint: он имеет отдельное
ROS-окружение и не является unit-тестами наших библиотек. Финальная ROS-проверка выполняется
только командой Docker из первого раздела или в нативной Humble-среде.
