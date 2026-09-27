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
| выход | `/result/position` | `nav_msgs/msg/Odometry` | `map` при привязке; иначе продольная координата `odom`/`track/<route>` |
| расширенный выход | `/odometry/estimate` | `odometry_msgs/msg/LongitudinalEstimate` | SI |
| диагностика | `/odometry/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | — |

Ускорение находится в `LongitudinalEstimate.a_mps2`; отдельный обязательный топик
ускорения организаторами не задан.

## Обязательные параметры

- `vehicle_id` — профиль трамвая, с fallback на `default.yaml`;
- `route_id` и `s0` — нужны для абсолютной позиции без GNSS; без route публикуется относительная одометрия;
- `pathgraph_directory` — каталог двух направленных JSON;
- `model_config` — файл или каталог идентифицированных параметров;
- `gnss_policy` — `disabled`, `initialization_only` или `intermittent`.

Production-запуск без `model_config` завершается ошибкой. Hardcoded параметры доступны
только при явном `allow_unidentified_model:=true` и предназначены для тестов.
Структура model YAML описана в [`CONFIGURATION.md`](CONFIGURATION.md).
При `ros2 launch ... reserve.launch.py` пути к модели и карте по умолчанию берутся из
установленного `odometry_python`. При прямом `ros2 run` путь к модели задаётся явно.

## Быстрый запуск и автоматическая проверка

```bash
docker compose --profile reserve build reserve
docker compose --profile reserve run --rm reserve /usr/bin/python3 /tools/validate_rosbag.py \
  --bag /check-bags/30618_88aea4d9 --output /artifacts/my-ros-check
```

Это полный rosbag при скорости 1×, production executable, два выходных топика,
диагностика и независимый подписчик эталона. Лимиты контейнера: 2 CPU и 512 МБ.
Результаты — `artifacts/my-ros-check/{report.json,velocity.csv,position.csv,estimates.csv,
reference.csv,diagnostics.jsonl,runtime.log,player.log}`. Каталог должен быть новым.
Для короткой проверки добавить `--duration 60`; для обычного датасета использовать
`--bag /bags/<bag-id>`. Эталон `/localization/kinematic_state` есть в check-bag, но не во всех
записях: при его отсутствии ошибки будут `null`, а не ноль. GNSS-эталон обычных bag
проверяется отдельно через Failure Lab.

Без маршрута `/result/position.pose.pose.position.x` — пройденная вдоль пути дистанция,
не реконструкция XY-траектории. Для абсолютной карты нужно добавить известные
`--route-id <route> --s0 <metres>` либо `--gnss-policy initialization_only`.
`--start-offset N` выбирает начало фрагмента в секундах от начала bag.

## Строгий режим без GNSS

```bash
ros2 launch odometry_node reserve.launch.py \
  vehicle_id:=30618 \
  route_id:=tallinskaya-shchukinskaya \
  s0:=0.0 \
  use_sim_time:=true \
  gnss_policy:=disabled \
  pathgraph_directory:=/workspace/dataset/Pathgraph \
  model_config:=/workspace/configs/models
```

В `disabled` GNSS subscriptions не создаются.

## Начальная или периодическая GNSS-коррекция

Заменить `gnss_policy` на `initialization_only` либо `intermittent`. В первом режиме после
первой принятой привязки GNSS-вход закрывается программным флагом; подписки безопасно
уничтожаются только при завершении ноды (иначе возможна гонка executor Humble).
Если привязка не получена,
окно всё равно закрывается через 5 секунд (`gnss_initialization_window_s`). Поздний GNSS
не используется скрыто. Первая привязка не сбрасывает скорость, управление и очередь.
Во втором position/velocity
updates проходят route, baseline, cross-track и EKF gates.

## Параметры runtime

| Параметр | Default | Назначение |
|---|---:|---|
| `vehicle_id` | `default` | выбор `<vehicle_id>.yaml`, затем `default.yaml` |
| `route_id` | пусто | фиксированное направление Pathgraph |
| `s0` | `0.0` | начальная along-track координата, м |
| `initial_v_mps` | `-1.0` | -1: первое пригодное колесное измерение; >=0: явно заданная скорость |
| `gnss_policy` | `disabled` | политика использования GNSS |
| `pathgraph_directory` | пусто | каталог JSON Pathgraph |
| `model_config` | пусто | YAML либо каталог профилей |
| `publish_rate_hz` | `50.0` | частота публикации, минимум 10 Гц |
| `processing_delay_ms` | `300.0` | fixed-lag окно, включающее ожидание пары GNSS; публикация прогнозируется на текущее время |
| `gnss_initialization_window_s` | `5.0` | максимальное время разрешённой начальной GNSS-привязки |
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
  -p route_id:=tallinskaya-shchukinskaya \
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

`input_to_publication_ms` измеряется монотонными часами от входа wheel/control callback
(включая ожидание lock) до публикации результатов. Это **не** время от физического датчика:
ожидание до вызова callback сюда не входит. При отсутствии новых входов значение `None`.
`compute_ms` — отдельное время шага фильтра. Percentiles latency имеют окно до 4096 входов.

В diagnostics также проверить `input_lateness_p95_ms`, `wheel_lateness_p99_ms`,
`late_input_count`, `too_late_input_count`, `reorder_window_ms` и
`filter_commit_lag_ms`. Для штатного bag `too_late_input_count` должен оставаться около
нуля; рост означает, что окно меньше фактической транспортной задержки.

`/result/velocity` и `/result/position` публикуются и при деградации; качество читается из
`/odometry/estimate` и diagnostics. Без карты frame=`odom`, x — расстояние от старта.
За пределами известного маршрута frame=`track/<route_id>`, x — абсолютная along-track
координата; это не XY карты. Поперечные/угловые ковариации в этих режимах большие.
Frame нельзя игнорировать при сравнении координат. `OUT_OF_GRAPH` не делает автоматически
невалидной скорость и не подставляет край карты вместо настоящего положения.
API и браузер для работы ноды не требуются.

Остановить проигрывание и ноду следует через `Ctrl+C`, после чего сохранить идентификатор
bag, профиль модели, параметры запуска и измеренные diagnostics в протокол демонстрации.
