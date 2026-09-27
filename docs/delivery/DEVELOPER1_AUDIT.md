# Исторический аудит ветки runtime-estimator

> Архивный отчёт до интеграции geometry/reference. Актуальный статус сдачи находится в
> [`README.md`](README.md). Перечисленные ниже внешние зависимости уже влиты, однако
> целевой ROS/rosbag-прогон по-прежнему должен быть выполнен.

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

## Состояние после интеграции

- `odometry_geometry`, Pathgraph и `configs/models/*.yaml` интегрированы в `main`.
- Production runtime строго загружает профиль и не использует скрытый hardcoded fallback.
- Docker/colcon тест подготовлен, но в последнем локальном аудите не выполнен: Docker
  daemon не был запущен.
- p99/CPU/RSS инструментированы, но соответствие лимитам подтверждается только прогоном в
  целевом Humble-контейнере на длинном bag.
- Offline validation report существует, но сквозные метрики production ROS-ноды ещё нужно
  зафиксировать на предоставленном rosbag.

Перед сдачей обязательна проверка:

```bash
docker compose --profile test build ros-tests
docker compose --profile test run --rm ros-tests
ros2 bag play dataset/data/30618_af7496f0 --clock
```

Код ветки интегрирован. Интеграционная и эксплуатационная приёмка остаётся открытой по
списку в актуальном [`README.md`](README.md).
