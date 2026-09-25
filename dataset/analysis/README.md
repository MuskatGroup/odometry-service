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

GNSS-записи используются для аналитики и оценки точности. Они не должны
передаваться в резервный оцениватель как входные измерения.
