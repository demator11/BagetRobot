# BagetRobot — Webots Robot Mapper

Робот в симуляторе Webots исследует помещение и строит карту (occupancy grid),
покрывая датчиками максимальную площадь за фиксированное время.

## Архитектура

```
BagetRobot/
├── controllers/
│   ├── robot_mapper/                     # Библиотека алгоритмов
│   │   ├── models/
│   │   │   ├── command.py
│   │   │   ├── coverage_map.py
│   │   │   ├── obstacle_grid.py
│   │   │   ├── pose.py
│   │   │   └── sensor_data.py
│   │   ├── strategies/
│   │   │   ├── exploration/
│   │   │   │   ├── base.py               # Интерфейс стратегии
│   │   │   │   ├── frontier.py           # Frontier-based exploration
│   │   │   │   ├── greedy.py             # Жадная стратегия
│   │   │   │   └── random.py             # Случайная стратегия
│   │   │   └── navigation/
│   │   │       ├── base.py               # Интерфейс навигации
│   │   │       └── differential.py       # Bang-bang управление (DRIVE/TURN)
│   │   └── visualizer.py
│   └── robot_mapper_controller/          # Точка входа (Webots controller)
│       ├── robot_mapper_controller.py    # Main loop
│       └── webots_robot.py               # HAL для Webots API
├── scripts/
│   └── demo_with_coverage.py             # Демо без Webots
└── worlds/
    └── robot_mapper.wbt                  # Webots world
```

## Навигация

Управление дифференциальное: **либо едет прямо, либо поворачивает на месте**.

Состояния: `DRIVE` > `TURN` > `DRIVE` > ...

- `DRIVE`: движение прямо на текущую цель на максимальной скорости.
- `TURN`: чистое вращение на месте до целевого угла (дефолтный допуск 0.08 rad).

### Объезд препятствий

1. BFS pathfinding через OccupancyGrid: от текущей позиции до целевого frontier.
2. Путь строится без просветов - клетки, соседние со стеной, не включаются.
3. Робот следует по waypoints пути: поворот на 90° > движение > поворот.
4. Если сонары видят стену ближе 13 см (на пути) или 30 см (без пути):
   - blocked_steps счётчик, первые 4 шага — реверс (-0.1 м/с),
   - на 5-м шаге - `_choose_turn_angle` (выбирает направление с максимумом UNKNOWN клеток),
   - поворот на 90° в выбранную сторону.
5. Если вплотную (< 15 см) — реверс (-0.2 м/с) во время поворота.

### Параметры (Config в robot_mapper_controller.py)

| Параметр | Значение | Описание |
|----------|----------|----------|
| `max_linear_speed` | 0.5 m/s | Скорость прямо |
| `max_angular_speed` | 1.5 rad/s | Скорость поворота |
| `angle_tolerance` | 0.08 rad | Допуск доворота |
| `position_tolerance` | 0.15 m | Допуск достижения waypoint |
| `safe_distance` | 0.3 m | Дистанция до стены (без пути) |
| `map_size` | 200 | Размер occupancy grid (клеток) |
| `map_resolution` | 0.1 m | Разрешение карты |

## Модели

**OccupancyGrid** (obstacle_grid.py):
- `FREE` (0.0), `WALL` (1.0), `UNKNOWN` (0.5)
- `update_from_sonar(pose, distances, angles)` — inverse sensor model
- `find_path(x1,y1, x2,y2)` — BFS с clearance
- `path_is_free(x1,y1, x2,y2)` — проверка прямой видимости
- `find_detour(x1,y1, x2,y2)` — поиск обхода

**CoverageMap** (coverage_map.py):
- Отмечает посещённые клетки, считает процент покрытия.

## Frontier Exploration

FrontierExploration находит границы между FREE и UNKNOWN клетками,
кластеризует их и выбирает цель по скорингу:
`score = cluster_size / distance * cos(angle_diff)`.
Цели в черном списке (после застревания) исключаются.

## Запуск

1. Открыть `worlds/robot_mapper.wbt` в Webots.
2. Выбрать контроллер `robot_mapper_controller`.
3. Запустить симуляцию.
