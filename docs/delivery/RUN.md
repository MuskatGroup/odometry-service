# Запуск резервной одометрии

Главный executable для сдачи — `odometry_node reserve_odometry_node`. Старые executables
`estimator`, `replay`, `source_adapter` и `telemetry_bridge` оставлены для лабораторного
стенда и не стоят между входами организаторов и production-алгоритмом.

## ROS-контракт

| Направление | Топик | Тип | Единицы |
|---|---|---|---|
| вход | `/vehicle/front_bogie_velocity` | `tram_vehicle_msgs/msg/VelocitySensor` | км/ч |
| вход | `/vehicle/rear_bogie_velocity` | `tram_vehicle_msgs/msg/VelocitySensor` | км/ч |
| вход | `/vehicle/driver_position_cmd` | `tram_vehicle_msgs/msg/DriverControllerCommand` | `[-15,15]` |
| опциональный вход | `/sensing/gnss/{master,rover}/fix` | `sensor_msgs/msg/NavSatFix` | WGS84 |
| опциональный вход | `/sensing/gnss/{master,rover}/vel` | `geometry_msgs/msg/TwistStamped` | м/с |
| выход | `/result/velocity` | `tram_vehicle_msgs/msg/VelocitySensor` | м/с |
| выход | `/result/position` | `nav_msgs/msg/Odometry` | continuous MGRS, `map→base_link` |
| расширенный выход | `/odometry/estimate` | `odometry_msgs/msg/LongitudinalEstimate` | SI |
| диагностика | `/odometry/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | — |

Ускорение находится в `LongitudinalEstimate.a_mps2`; отдельный обязательный топик
ускорения организаторами не задан.

## Обязательные параметры

- `vehicle_id` — профиль трамвая, с fallback на `default.yaml`;
- `route_id` и `s0` — обязательны для `disabled`;
- `pathgraph_directory` — каталог двух направленных JSON;
- `model_config` — файл или каталог идентифицированных параметров;
- `gnss_policy` — `disabled`, `initialization_only` или `intermittent`.

Production-запуск без `model_config` завершается ошибкой. Hardcoded параметры доступны
только при явном `allow_unidentified_model:=true` и предназначены для тестов.
Структура model YAML описана в [`CONFIGURATION.md`](CONFIGURATION.md).

## Строгий режим без GNSS

```bash
ros2 launch odometry_node reserve.launch.py \
  vehicle_id:=30618 \
  route_id:=tallinskaya_to_shchukinskaya \
  s0:=0.0 \
  gnss_policy:=disabled \
  pathgraph_directory:=/workspace/dataset/Pathgraph \
  model_config:=/workspace/configs/models
```

В `disabled` GNSS subscriptions не создаются.

## Начальная или периодическая GNSS-коррекция

Заменить `gnss_policy` на `initialization_only` либо `intermittent`. В первом режиме после
первой принятой привязки GNSS subscriptions уничтожаются. Во втором position/velocity
updates проходят route, baseline, cross-track и EKF gates.

## Параметры runtime

| Параметр | Default | Назначение |
|---|---:|---|
| `vehicle_id` | `default` | выбор `<vehicle_id>.yaml`, затем `default.yaml` |
| `route_id` | пусто | фиксированное направление Pathgraph |
| `s0` | `0.0` | начальная along-track координата, м |
| `initial_v_mps` | `0.0` | начальная скорость |
| `gnss_policy` | `disabled` | политика использования GNSS |
| `pathgraph_directory` | пусто | каталог JSON Pathgraph |
| `model_config` | пусто | YAML либо каталог профилей |
| `publish_rate_hz` | `50.0` | частота публикации, минимум 10 Гц |
| `processing_delay_ms` | `120.0` | fixed-lag окно приёма запоздавших timestamp; публикация прогнозируется на текущее время |
| `max_model_only_s` | `10.0` | валидный горизонт без колёс |
| `gnss_sync_tolerance_ms` | `200.0` | допуск синхронизации антенн |
| `gnss_cross_track_gate_m` | `8.0` | максимальное отклонение от пути |
| `gnss_baseline_error_gate_m` | `1.0` | допуск ошибки базы антенн |
| `gnss_position_variance_m2` | `4.0` | fallback variance позиции |
| `gnss_velocity_variance_m2ps2` | `0.25` | variance GNSS-скорости |

## Воспроизведение rosbag

```bash
ros2 run odometry_node reserve_odometry_node --ros-args \
  --params-file /workspace/configs/ros/disabled.yaml \
  -p use_sim_time:=true \
  -p vehicle_id:=30618 \
  -p route_id:=tallinskaya_to_shchukinskaya \
  -p s0:=0.0 \
  -p pathgraph_directory:=/workspace/dataset/Pathgraph \
  -p model_config:=/workspace/configs/models
```

Во втором терминале:

```bash
ros2 bag play /workspace/dataset/data/<bag-id> --clock
```

Нода напрямую слушает выданные `/vehicle/*` и при разрешённой политике `/sensing/gnss/*`.
Для bag time запускать ноду с `use_sim_time:=true`.

`Header.stamp` колёс в предоставленных bag отстаёт от времени записи примерно на
35–100 мс. Поэтому основной EKF фиксирует историю только до
`now-processing_delay_ms`, а disposable-копия прогнозируется до `now` и публикуется с
текущим stamp. Окно не добавляется целиком к задержке `/result/*`. Сообщения старше
watermark всё равно отвергаются как `OUT_OF_ORDER` и учитываются в diagnostics.

## Проверка результата

```bash
ros2 topic hz /result/velocity
ros2 topic hz /result/position
ros2 topic echo /odometry/estimate --once
ros2 topic echo /odometry/diagnostics
```

В diagnostics проверить `input_lateness_p95_ms`, `wheel_lateness_p99_ms`,
`late_input_count`, `too_late_input_count`, `reorder_window_ms` и
`filter_commit_lag_ms`. Для штатного bag `too_late_input_count` должен оставаться около
нуля; рост означает, что окно меньше фактической транспортной задержки.

`/result/velocity` публикуется при наличии численной оценки скорости. Валидный
`/result/position` появляется только при известном route, доступном Pathgraph и пригодной
covariance. API и браузер для работы ноды не требуются.

Остановить проигрывание и ноду следует через `Ctrl+C`, после чего сохранить идентификатор
bag, профиль модели, параметры запуска и измеренные diagnostics в протокол демонстрации.
