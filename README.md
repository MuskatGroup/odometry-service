# Резервная одометрия трамвая

Решение хакатона: ROS 2 Humble-нода оценивает продольные ускорение, скорость и
положение трамвая по команде контроллера и скоростям двух тележек. GNSS не нужен
в основном режиме; он может использоваться только для начальной привязки или
разрешённой периодической коррекции.

## Материалы для жюри

| Материал | Ссылка |
|---|---|
| ROS 2-пакеты | [`ros2_ws/src`](ros2_ws/src) |
| Официальный стенд и check-bag | [`tests/check-code`](tests/check-code) |
| Сборка, запуск rosbag, выходные топики и метрики | [`docs/submission/JURY_GUIDE.md`](docs/submission/JURY_GUIDE.md) |
| Математическая модель | [`docs/submission/MODEL.md`](docs/submission/MODEL.md) |
| Допущения, ограничения и параметры | [`docs/submission/CONFIGURATION.md`](docs/submission/CONFIGURATION.md) |
| Точность и быстродействие | [`docs/submission/VALIDATION.md`](docs/submission/VALIDATION.md) |
| Ограничения и план развития | [`docs/submission/ROADMAP.md`](docs/submission/ROADMAP.md) |

## Контракт

Входы:

- `/vehicle/front_bogie_velocity` — `tram_vehicle_msgs/msg/VelocitySensor`, км/ч;
- `/vehicle/rear_bogie_velocity` — `tram_vehicle_msgs/msg/VelocitySensor`, км/ч;
- `/vehicle/driver_position_cmd` — `tram_vehicle_msgs/msg/DriverControllerCommand`, `[-15; 15]`.

Обязательные выходы:

- `/result/velocity` — `tram_vehicle_msgs/msg/VelocitySensor`, м/с;
- `/result/position` — `nav_msgs/msg/Odometry`, положение `base_link` в `map`.

Расширенные выходы:

- `/odometry/estimate` — состояние фильтра, ускорение, covariance и состояние колёс;
- `/odometry/diagnostics` — задержка, частота, CPU/RSS и счётчики ошибок.

## Быстрый старт

```bash
docker compose build reserve
docker compose run --rm reserve bash

# внутри контейнера
source /opt/ros/humble/setup.bash
source /workspace/ros2_ws/install/setup.bash
ros2 launch odometry_node reserve.launch.py \
  vehicle_id:=30618 \
  route_id:=shchukinskaya_to_tallinskaya \
  s0:=76.70021572499135 \
  initial_v_mps:=8.264238620430273 \
  gnss_policy:=disabled \
  use_sim_time:=true
```

Подробная проверка и воспроизведение результата описаны в
[`JURY_GUIDE.md`](docs/submission/JURY_GUIDE.md).

Лицензия: [MIT](LICENSE).
