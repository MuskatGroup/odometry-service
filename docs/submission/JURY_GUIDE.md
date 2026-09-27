# Инструкция для жюри

## 1. Состав решения

Собираемый workspace находится в [`ros2_ws/src`](../../ros2_ws/src):

| Пакет | Назначение |
|---|---|
| `tram_vehicle_msgs` | входные и выходное сообщение скорости организаторов |
| `odometry_msgs` | расширенное состояние и диагностика алгоритма |
| `odometry_python` | установка ядра, геометрии, профилей и Pathgraph без сети |
| `odometry_node` | исполняемый файл `reserve_odometry_node` и launch-файл |

API и web для работы алгоритма не нужны.

Выданный организаторами приёмочный стенд и исходный check-bag сохранены без изменений
в [`tests/check-code`](../../tests/check-code). Его `checker_ros` сравнивает
`/result/velocity` и `/result/position` с `/localization/kinematic_state`.

## 2. Сборка ROS 2 Humble

Требования: Ubuntu 22.04, ROS 2 Humble, Python 3.10, `python3-yaml`, пакеты
`nav_msgs`, `sensor_msgs`, `diagnostic_msgs`, `geometry_msgs` и `rosbag2`.

### В Docker

Команды выполняются из корня репозитория в Bash. Контейнер запускается в фоне с
именем, чтобы ноду и проигрыватель bag можно было открыть в двух терминалах:

```bash
mkdir -p artifacts
docker build -f infra/Dockerfile.ros -t reserve-odometry:submission .
docker run --rm -d --name reserve-odometry-check \
  --cpus 2 --memory 512m \
  -e ROS_DOMAIN_ID=76 \
  -v "$(pwd)/tests/check-code/bags:/bags:ro" \
  -v "$(pwd)/artifacts:/artifacts" \
  reserve-odometry:submission sleep infinity
```

Образ собирает workspace командой `colcon build`. Runtime-зависимости входят в
образ; во время последующего `colcon build` интернет не требуется.

### На машине с Humble

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build
source install/setup.bash
```

Используйте обычный `colcon build`: пакет `odometry_python` устанавливает три
runtime-библиотеки из их канонических каталогов и не рассчитан на режим
`--symlink-install`.

## 3. Запуск на rosbag

При Docker-запуске в первом терминале:

```bash
docker exec -it reserve-odometry-check bash -lc '
  source /opt/ros/humble/setup.bash &&
  source /workspace/ros2_ws/install/setup.bash &&
  exec ros2 launch odometry_node reserve.launch.py \
    vehicle_id:=30618 \
    route_id:=shchukinskaya_to_tallinskaya \
    s0:=76.70021572499135 \
    initial_v_mps:=8.264238620430273 \
    gnss_policy:=disabled \
    use_sim_time:=true'
```

Во втором терминале:

```bash
docker exec -it reserve-odometry-check bash -lc '
  source /opt/ros/humble/setup.bash &&
  source /workspace/ros2_ws/install/setup.bash &&
  exec ros2 bag play /bags/30618_88aea4d9 --clock 100 --rate 1 \
    --start-offset 230 --disable-keyboard-controls'
```

Значения `route_id`, `s0` и `initial_v_mps` задают разрешённую начальную
привязку в точке `+230 с`. Начало выданного check-bag лежит вне предоставленного
Pathgraph, поэтому для воспроизводимого картографического сравнения используется
покрытая картой часть записи. После старта политика `disabled` не создаёт GNSS-подписок.

Для другого прогона следует передать его известную стартовую позицию либо включить
`gnss_policy:=initialization_only`: тогда первые доступные GNSS-сообщения определят
маршрут и `s`, после чего подписки будут закрыты.

## 4. Ожидаемые топики

```bash
docker exec -it reserve-odometry-check bash -lc '
  source /opt/ros/humble/setup.bash &&
  source /workspace/ros2_ws/install/setup.bash &&
  ros2 topic hz /result/velocity'

docker exec -it reserve-odometry-check bash -lc '
  source /opt/ros/humble/setup.bash &&
  source /workspace/ros2_ws/install/setup.bash &&
  ros2 topic echo /result/position --once'

docker exec -it reserve-odometry-check bash -lc '
  source /opt/ros/humble/setup.bash &&
  source /workspace/ros2_ws/install/setup.bash &&
  ros2 topic hz /result/position'

docker exec -it reserve-odometry-check bash -lc '
  source /opt/ros/humble/setup.bash &&
  source /workspace/ros2_ws/install/setup.bash &&
  ros2 topic echo /odometry/diagnostics --once'
```

| Топик | Тип | Содержание |
|---|---|---|
| `/result/velocity` | `tram_vehicle_msgs/msg/VelocitySensor` | скорость в м/с |
| `/result/position` | `nav_msgs/msg/Odometry` | `frame_id=map`, `child_frame_id=base_link`, MGRS XYZ, yaw и скорость |
| `/odometry/estimate` | `odometry_msgs/msg/LongitudinalEstimate` | `s`, `v`, ускорение, disturbance, covariance, wheel health |
| `/odometry/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | задержка, CPU, RSS, очередь и ошибки |

`/result/position` не публикуется без абсолютной привязки или после выхода за
допустимый горизонт полностью модельного прогноза. Скорость продолжает публиковаться.

## 5. Автоматическая проверка

В образ входит валидатор, который запускает production-ноду, проигрыватель и
независимый подписчик эталона:

Сначала завершите ручные `ros2 launch` и `ros2 bag play` сочетанием `Ctrl+C` в их
терминалах. Валидатор сам запускает оба процесса и не должен работать параллельно с ними.

```bash
docker exec -it reserve-odometry-check bash -lc '
  source /opt/ros/humble/setup.bash &&
  source /workspace/ros2_ws/install/setup.bash &&
  python3 /workspace/tools/validate_rosbag.py \
    --bag /bags/30618_88aea4d9 \
    --output /artifacts/validation \
    --vehicle-id 30618 \
    --route-id shchukinskaya_to_tallinskaya \
    --s0 76.70021572499135 \
    --initial-v-mps 8.264238620430273 \
    --start-offset 230 \
    --rate 1'
```

Каталог `artifacts/validation` должен отсутствовать или быть пустым. Для повторного
прогона задайте новое имя каталога: существующие результаты намеренно не перезаписываются.

Результаты:

- `report.json` — RMSE, bias, частота, режимы, задержка, CPU/RSS;
- `velocity.csv`, `position.csv`, `estimates.csv` — полученные выходы;
- `reference.csv` — эталон, прочитанный только отдельным оценщиком;
- `diagnostics.jsonl`, `runtime.log`, `player.log` — диагностика и логи.

Эталон `/localization/kinematic_state` не передаётся ноде. В строгом режиме ROS graph
ноды содержит только три обязательные подписки и `/clock`.

## 6. Тесты

```bash
docker exec -it reserve-odometry-check bash -lc '
  source /opt/ros/humble/setup.bash &&
  source /workspace/ros2_ws/install/setup.bash &&
  python3 -m pytest -q /workspace/tests'
```

Проверяются EKF, таблицы тяги/торможения, уклон, проскальзывание, freeze/dropout,
reacquisition, fixed-lag обработка поздних данных, Pathgraph, реальные ROS-типы и
обязательные выходные топики.

После проверки:

```bash
docker stop reserve-odometry-check
```
