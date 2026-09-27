# Запуск резервной одометрии

## Обязательные параметры

- `vehicle_id` — профиль трамвая, с fallback на `default.yaml`;
- `route_id` и `s0` — обязательны для `disabled`;
- `pathgraph_directory` — каталог двух направленных JSON;
- `model_config` — файл или каталог идентифицированных параметров;
- `gnss_policy` — `disabled`, `initialization_only` или `intermittent`.

## Строгий режим без GNSS

```bash
ros2 launch odometry_node reserve.launch.py \
  vehicle_id:=30618 \
  route_id:=tallinskaya-shchukinskaya \
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

## Воспроизведение rosbag

```bash
ros2 bag play /workspace/dataset/data/<bag-id> --clock
```

Нода напрямую слушает выданные `/vehicle/*` и при разрешённой политике `/sensing/gnss/*`.
Для bag time запускать ноду с `use_sim_time:=true`.

## Проверка результата

```bash
ros2 topic hz /result/velocity
ros2 topic hz /result/position
ros2 topic echo /odometry/diagnostics
```

`/result/velocity` публикуется при наличии численной оценки скорости. Валидный
`/result/position` появляется только при известном route, доступном Pathgraph и пригодной
covariance. API и браузер для работы ноды не требуются.
