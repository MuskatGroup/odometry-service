# Аудит реализации разработчика 1

Дата проверки: 26 сентября 2026.

## Реализовано

- organizer `tram_vehicle_msgs` встроен в ROS workspace;
- прямые подписки `/vehicle/front_bogie_velocity`, `/vehicle/rear_bogie_velocity` и
  `/vehicle/driver_position_cmd`;
- нормализация км/ч → м/с и controller notch → `[-1,1]`;
- `/result/velocity`, `/result/position`, расширенная оценка и diagnostics;
- EKF `[s,v,a_act,d]`, общий scalar update для position/velocity и Joseph covariance;
- traction/braking maps, лаг привода, сопротивление, grade-provider и bounded disturbance;
- строгий versioned YAML-профиль с fallback `vehicle_id → default`;
- wheel health, freeze/dropout/slip/slide/inconsistent, hysteresis и `MODEL_ONLY`;
- `disabled`, `initialization_only`, `intermittent`, GNSS pre-gates и correction counters;
- geometry dependency изолирована адаптером, собственной UTM/Pathgraph реализации нет;
- bounded latency window, p95/p99/max, CPU, RSS и queue-depth diagnostics;
- три ROS-конфигурации, production launch и четыре обязательных runtime-документа.

## Автоматически проверено

```text
pytest:      39 passed, 4 skipped
ruff:        passed
compileall:  passed
XML/YAML:    parsed successfully
diff check:  passed
```

Skipped-тесты требуют ROS 2 Humble. На Windows-хосте `ros2` отсутствует.

## Зависимости и ещё не подтверждённые критерии

- `odometry_geometry` разрабатывается независимо; до его merge невозможно выполнить
  реальный Pathgraph/MGRS/GNSS прогон.
- Идентифицированные `configs/models/*.yaml` должны поступить от потока geometry/reference;
  production runtime намеренно не использует скрытый hardcoded fallback.
- Docker/colcon тест подготовлен, но не выполнен: Docker Desktop daemon не запущен.
- p99/CPU/RSS инструментированы, но соответствие лимитам подтверждается только прогоном в
  целевом Humble-контейнере на длинном bag.
- Точность модели не заявляется до появления reference, параметров и validation report.

После получения geometry/model artifacts обязательная проверка:

```bash
docker compose --profile test build ros-tests
docker compose --profile test run --rm ros-tests
ros2 bag play dataset/data/30618_af7496f0 --clock
```

Таким образом, кодовая зона ответственности разработчика 1 закрыта. Интеграционная и
численная приёмка остаётся открытой по явно перечисленным внешним зависимостям.
