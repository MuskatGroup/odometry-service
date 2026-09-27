# Сборка обязательного ROS-контура

Целевая среда: Ubuntu 22.04, ROS 2 Humble, Python 3.10.

## Docker

Перед сборкой Docker daemon должен быть запущен:

```bash
docker compose --profile test build ros-tests
docker compose --profile test run --rm ros-tests
```

Dockerfile устанавливает системные ROS-зависимости до копирования workspace, затем
собирает `tram_vehicle_msgs`, `odometry_msgs` и `odometry_node` через `colcon`.
Python-библиотеки `odometry_core`, `odometry_geometry`, `odometry_io` и `odometry_lab`
устанавливаются внутрь образа до сборки workspace.

## Нативная Humble-среда

```bash
python3 -m pip install \
  ./python/odometry_core \
  ./python/odometry_geometry \
  ./python/odometry_io \
  ./python/odometry_lab
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 pkg executables odometry_node
```

В списке должен присутствовать `odometry_node reserve_odometry_node`.

Сборка Docker-образа требует сети для `apt`/`pip`. После создания образа `colcon build` и
runtime не должны скачивать зависимости. Pathgraph и идентифицированные model YAML входят
в репозиторий и передаются ноде как локальные файлы.

## Локальные проверки Python

```powershell
.\.venv\Scripts\python.exe -m pytest -q
.\.venv\Scripts\ruff.exe check python tests ros2_ws/src/odometry_node
```

На машине без ROS два integration-теста будут пропущены. Финальная проверка выполняется
только командой Docker из первого раздела или в нативной Humble-среде.
