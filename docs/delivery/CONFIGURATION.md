# Конфигурация решения

Есть два независимых уровня конфигурации: параметры ROS-ноды и versioned YAML-профиль
математической модели. Все физические величины внутри ядра задаются в SI.

Масса, радиус колеса, передаточное отношение и КПД не задаются отдельно: их
неразделимое влияние включено в идентифицированную карту ускорения. Физическая связь
с моментом и границы такой параметризации описаны в [MODEL.md](MODEL.md).

## Выбор профиля модели

Параметр ROS `model_config` принимает путь к одному YAML-файлу либо каталогу. Для каталога
сначала ищется `<vehicle_id>.yaml`, затем `default.yaml`. Если ни один файл не найден,
production-запуск завершается ошибкой.

Доступные профили:

- `configs/models/30618.yaml`;
- `configs/models/30639.yaml`;
- `configs/models/default.yaml`.

Загрузчик строгий: неизвестные поля, нечисловые значения, неверные знаки и несовпадающие
размеры таблиц считаются ошибкой конфигурации. Параметры называются
«идентифицированными», а не «обученной ML-моделью».

## Схема model YAML

| Раздел | Поля | Смысл |
|---|---|---|
| metadata | `schema_version`, `model_version`, `vehicle_id` | версия контракта и применимость |
| provenance | `identified_at_utc`, `identification_method`, `identification_bags`, `validation_bags` | происхождение чисел и защита от утечки между выборками |
| `longitudinal` | `tau_s`, `c1_inv_s`, `c2_inv_m`, `disturbance_limit_mps2` | лаг привода, сопротивление и предел возмущения |
| `traction` | `controller_u`, `speed_mps`, `acceleration_mps2` | таблица ускорения в тяге |
| `braking` | `controller_u`, `speed_mps`, `acceleration_mps2` | таблица ускорения при торможении |
| `noise` | `q_v`, `q_a`, `q_d`, `wheel_variance_floor` | process noise EKF и минимальный шум колёс |
| `wheel_health` | `gate_normal`, `gate_reject`, `max_wheel_accel_mps2`, `freeze_s`, `reacquire_s`, `recover_updates` | обнаружение отказов и возврат канала |
| `metrics` | произвольные измеренные метрики | справочная часть, не влияет на runtime |

`acceleration_mps2` — матрица размера
`len(controller_u) × len(speed_mps)`. Тяговая таблица неотрицательна, тормозная —
неположительна. За границами осей используется ближайшее граничное значение.

## Профили ROS

Каталог `configs/ros` содержит стартовые параметры трёх политик:

| Файл | Политика | Назначение |
|---|---|---|
| `disabled.yaml` | `disabled` | основной режим сдачи: GNSS не подписывается и используется только offline как эталон |
| `initialization_only.yaml` | `initialization_only` | GNSS задаёт начальные route/position; после первой привязки или 5 с вход закрыт |
| `intermittent.yaml` | `intermittent` | разрешены gated GNSS-коррекции в середине маршрута |

Профиль ROS не содержит локальные пути и идентификатор конкретного запуска. Их нужно
передавать явно через `-p`:

```bash
ros2 run odometry_node reserve_odometry_node --ros-args \
  --params-file configs/ros/disabled.yaml \
  -p vehicle_id:=30618 \
  -p route_id:=tallinskaya_to_shchukinskaya \
  -p s0:=0.0 \
  -p pathgraph_directory:=dataset/Pathgraph \
  -p model_config:=configs/models
```

Полный список runtime-параметров находится в [`RUN.md`](RUN.md). Любое изменение порогов
для итогового прогона следует сохранять вместе с отчётом, иначе результат невоспроизводим.
