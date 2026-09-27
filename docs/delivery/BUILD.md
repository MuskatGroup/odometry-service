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

## Нативная Humble-среда

```bash
python3 -m pip install ./python/odometry_core ./python/odometry_io
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

Для финальной offline-проверки сначала подготовить базовый образ и Python wheels в среде
с сетью, затем повторить `colcon build` при отключённой сети. Pathgraph и
идентифицированные model YAML являются входными файлами запуска, а не скачиваемыми
зависимостями.

## Локальные проверки Python

```powershell
.\.venv\Scripts\python.exe -m pytest -q
.\.venv\Scripts\python.exe -m ruff check python tests
```
