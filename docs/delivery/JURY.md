# Проверка решения жюри

Версия для сдачи: <https://github.com/MuskatGroup/odometry-service/tree/submission/final-2026-09-27>.
Основной executable: `odometry_node reserve_odometry_node`.
Целевая среда: Ubuntu 22.04, ROS 2 Humble, Python 3.10. Веб-интерфейс для проверки не требуется.

## 1. Получить и собрать

```bash
git clone --branch submission/final-2026-09-27 https://github.com/MuskatGroup/odometry-service.git
cd odometry-service
docker compose --profile reserve build reserve
```

Клонировать нужно весь репозиторий: ROS-пакеты используют соседние `python/`,
`configs/` и `dataset/Pathgraph/`. `colcon build` собирает четыре пакета:
`tram_vehicle_msgs`, `odometry_msgs`, `odometry_python`, `odometry_node`.
ROS-пакеты: <https://github.com/MuskatGroup/odometry-service/tree/submission/final-2026-09-27/ros2_ws/src>.

Docker первоначально скачивает базовую среду и системные зависимости. Сам `colcon build`
и работа готового образа не требуют сети. Проверка чистой сборки без сети:

```bash
docker run --rm --network none --entrypoint /bin/bash odometry-ros:validation -lc \
  'source /opt/ros/humble/setup.bash && cd /workspace/ros2_ws && colcon --log-base /tmp/offline-log build --build-base /tmp/offline-build --install-base /tmp/offline-install --executor sequential'
```

Нативная сборка без Docker и список зависимостей: [BUILD.md](BUILD.md).

## 2. Автоматическая проверка без GNSS

В репозитории находится проверочный bag организаторов `tests/check-code/bags/30618_88aea4d9`.
Команда запускает production-ноду, rosbag player и независимый сборщик результатов:

```bash
docker compose --profile reserve run --rm reserve /usr/bin/python3 /tools/validate_rosbag.py \
  --bag /check-bags/30618_88aea4d9 \
  --output /artifacts/judge-relative \
  --duration 120 --gnss-policy disabled
```

Для полного bag убрать `--duration 120` (около 22 минут при 1×).
Каждый повтор должен использовать новый каталог `--output`, чтобы не затереть результаты.
Контейнер ограничен 2 CPU и 512 МиБ. В политике `disabled` у оценивателя нет подписок
GNSS или эталонной локализации. Сборщик читает эталон отдельно и не подаёт его в модель.

Без абсолютной начальной позиции `/result/position` содержит относительную along-track
одометрию: `frame_id=odom`, x — путь от старта. Это допустимый по заданию режим.
Сравнивать такие x/y/z с глобальным MGRS-эталоном нельзя: `map_position_rmse_m` будет
`null`, а не фиктивным нулём. Скорость и частота обоих выходов проверяются.

## 3. Проверка абсолютной позиции на карте

Начало проверочного bag находится вне выданной карты. Для воспроизводимого участка внутри
Pathgraph используйте старт с 230-й секунды и одну явно заданную начальную точку:

```bash
docker compose --profile reserve run --rm reserve /usr/bin/python3 /tools/validate_rosbag.py \
  --bag /check-bags/30618_88aea4d9 \
  --output /artifacts/judge-map \
  --start-offset 230 --duration 120 --gnss-policy disabled \
  --route-id shchukinskaya_to_tallinskaya \
  --s0 76.70021572499135 --initial-v-mps 8.264238620430273
```

Эти начальные координата и скорость взяты из одной стартовой точки эталона, что раскрыто
явно; последующих GNSS-коррекций нет. Для другого bag нужны его начальные условия.
Скорость можно также инициализировать первым пригодным колесным измерением, не передавая
`--initial-v-mps`. Для продолжения от 230-й секунды до конца убрать `--duration`.

Дополнительный режим `--gnss-policy initialization_only` принимает первую пригодную
привязку в первые 5 секунд входного потока, затем закрывает GNSS-вход. Если старт вне
карты, привязка не гарантируется. Режим с периодическими коррекциями не является основным
режимом сдачи. Полный перечень параметров: [RUN.md](RUN.md).

## 4. Входы, выходы и диагностика

| Топик | Тип | Единицы / содержание |
|---|---|---|
| `/vehicle/front_bogie_velocity` | `tram_vehicle_msgs/msg/VelocitySensor` | вход, км/ч |
| `/vehicle/rear_bogie_velocity` | `tram_vehicle_msgs/msg/VelocitySensor` | вход, км/ч |
| `/vehicle/driver_position_cmd` | `tram_vehicle_msgs/msg/DriverControllerCommand` | вход, −15…15 |
| `/result/velocity` | `tram_vehicle_msgs/msg/VelocitySensor` | выход, м/с |
| `/result/position` | `nav_msgs/msg/Odometry` | позиция, ориентация, скорость, covariance |
| `/odometry/estimate` | `odometry_msgs/msg/LongitudinalEstimate` | ускорение, s/v, valid, режим, здоровье колёс |
| `/odometry/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | задержка, память, CPU, очереди, ошибки |

Оба обязательных выхода публикуются и при отказах. `frame_id=map` означает continuous
MGRS 37UCB; `odom` — относительную координату; `track/<route>` — along-track координату
вне границ карты. После горизонта model-only оценка помечается `valid=false`; интервалы
не исключаются из offline RMSE. Frame и диагностику необходимо учитывать потребителю.

Во время работающего прогона в отдельных терминалах:

```bash
docker compose --profile reserve run --rm reserve ros2 topic hz /result/velocity
docker compose --profile reserve run --rm reserve ros2 topic hz /result/position
docker compose --profile reserve run --rm reserve ros2 topic echo /odometry/diagnostics --once
```

## 5. Где результаты

В `artifacts/judge-map/` или `artifacts/judge-relative/` сохраняются:

- `report.json` — завершение playback, частоты, ошибки и максимумы диагностики;
- `velocity.csv`, `position.csv`, `estimates.csv` — выходы с временными метками;
- `reference.csv` — независимый эталон `/localization/kinematic_state` проверочного bag;
- `diagnostics.jsonl` — режимы, latency, CPU, RSS, исключения;
- `runtime.log`, `player.log` — логи процессов.

```bash
cat artifacts/judge-map/report.json
```

Смотреть `rates.*.bag_time_hz` (требование ≥10 Гц), `diagnostic_maxima.latency_p99_ms`,
`latency_max_ms`, `rss_mb`, `cpu_cores`, `crash_count`, `too_late_input_count`.
Задержка измеряется от входа callback до публикации; транспорт до callback не включён.
Это измерение в обычной ОС, не гарантия hard real-time. `null` означает отсутствие
достаточного эталона. `full_bag_completed` отличается от завершения выбранного фрагмента.

Эталон check-bag — локализация организаторов; его нельзя автоматически объявлять чистым
GNSS. Отдельные GNSS-метрики по обычным bag, тесты отказов и границы результатов приведены
в [FINAL_VALIDATION.md](FINAL_VALIDATION.md). Параметры не подбираются по test-части.

## 6. Автоматические тесты

```bash
uv sync --locked --python 3.10
uv run pytest -q
uv run ruff check python tests ros2_ws/src/odometry_node ros2_ws/src/odometry_python tools
docker compose --profile test build ros-tests
docker compose --profile test run --rm --no-deps ros-tests
```

Без ROS и .NET opt-in тесты пропускаются на хосте; ROS проверяется в контейнере,
API и сборка Vue — отдельным job GitHub Actions. Актуальные проверки:
<https://github.com/MuskatGroup/odometry-service/actions/workflows/ci.yml>.
