# Dataset analysis

Аналитические инструменты используют общее виртуальное окружение проекта:

```text
<repository>/.venv
```

Зависимости объявлены в группе `analysis` корневого `pyproject.toml` и
зафиксированы в `uv.lock`. Локальные файлы `requirements.txt` в этой директории
не используются: единственным источником зависимостей является корень проекта.

## Подготовка окружения

Из корня репозитория выполните:

```powershell
uv sync --group dev --group analysis
```

В VS Code выберите интерпретатор:

```text
<repository>/.venv/Scripts/python.exe
```

После смены интерпретатора перезапустите kernel открытого notebook.

## Запуск JupyterLab

```powershell
uv run --group analysis jupyter lab dataset/analysis
```

## Ноутбуки

- `analysis.ipynb` читает только `metadata.yaml` всех bag и предназначен для
  быстрой инвентаризации, выбора прогонов и поиска кандидатов в дубликаты.
- `explore_bag.ipynb` декодирует один выбранный bag и строит подробные графики
  управления, колёс, GNSS, времени и диагностических расхождений.

Общие преобразования находятся в `bag_analysis.py`, а низкоуровневое чтение
ROS 2 bag — в `read_bag.py`. Не копируйте десериализацию заново в notebook.

## Просмотр Pathgraph

Простой интерактивный просмотрщик показывает оба направленных пути, начало и конец,
стрелки направления, профиль высоты и кривизну:

```powershell
uv run --group analysis python dataset/analysis/pathgraph_viewer.py
```

По умолчанию Matplotlib использует `WebAgg` и открывает просмотрщик в браузере; процесс
работает до `Ctrl+C`. Доступны масштабирование и перемещение. Щелчок рядом с линией
показывает ближайшую точку, её `route_id`, `s`, `x/y/z`, `yaw` и кривизну. В notebook
используется выбранный Jupyter backend, поэтому отдельный сервер не запускается.

Показать только одно направление:

```powershell
uv run --group analysis python dataset/analysis/pathgraph_viewer.py `
  --route tallinskaya_to_shchukinskaya
```

Сохранить изображение без открытия окна:

```powershell
uv run --group analysis python dataset/analysis/pathgraph_viewer.py `
  --save dataset/analysis/pathgraph.png `
  --no-show
```

Те же данные можно использовать в notebook:

```python
from pathgraph_analysis import load_all_pathgraphs, pathgraph_summary
from pathgraph_viewer import create_pathgraph_figure

routes = load_all_pathgraphs()
display(pathgraph_summary(routes))
create_pathgraph_figure(list(routes.values()))
```

`pathgraph_analysis.py` валидирует JSON, упорядочивает точки по `point_indices`,
нормализует `tang`, вычисляет накопленную координату `s` и сглаженный уклон.

## Проверка окружения в notebook

```python
import sys
import rosbags

print(sys.executable)
```

Путь должен оканчиваться на `.venv\\Scripts\\python.exe`.

## Чтение bag

```python
from read_bag import DATA_ROOT, iter_records

records = list(iter_records(DATA_ROOT / "30618_0d865417"))
len(records)
```

Полную схему ROS-полей и фактическое количество сообщений по каждому топику
можно посмотреть без ручного чтения CDR BLOB:

```python
from bag_analysis import bag_topic_schema

bag_topic_schema("30618_0d865417")
```

Топик с `message_count == 0` объявлен в bag, но не содержит данных.

Все уникальные топики сразу по всему датасету:

```python
from bag_analysis import dataset_topic_inventory

dataset_topic_inventory()
```

GNSS-записи используются для аналитики и оценки точности. Они не должны
передаваться в резервный оцениватель как входные измерения.
