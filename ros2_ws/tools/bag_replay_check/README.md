# Проверка bag-replay в ROS 2 Humble (Docker)

Генерирует rosbag из синтетического эмулятора (время от 100 с, проскальзывание и пропуск колес), проигрывает его с `--clock`, запускает ноду с `use_sim_time:=true` и сравнивает `/odometry/estimate` с эталоном. Скрипты предназначены для контейнера; репозиторий монтируется только для чтения.

```bash
docker run --rm -v /path/to/odometry-service:/repo:ro -v /path/to/odometry-service/ros2_ws/tools/bag_replay_check:/smoke:ro ros:humble-ros-base bash /smoke/replay.sh
```

Скрипты должны иметь окончания строк LF. Эталон здесь построен на тех же уравнениях, что и модель, поэтому проверяется интеграция (время, режимы, публикация), а не точность.
