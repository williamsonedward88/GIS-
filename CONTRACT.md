# 接口契约（改动需双方确认）

## 冻结文件
- `core/types.py`          —— 数据结构
- `core/cost/segment_cost.py` —— 唯一成本口径
- `core/solver/base.py`    —— Solver 抽象
- `core/cost/base.py`      —— DirectionModel

## 铁律
1. 所有 `(x, y)` 参数 = 世界坐标；`(r, c)` = 数组索引。禁止混用。
2. `core/` 下禁止 `import qgis` / `import processing`。
3. `np.inf` 表示不可通行。
4. 对角移动禁止切角。

## 责任人
- A：`plugin.py` `dialog.py` `io/*` `scripts/*`
- B：`core/solver/*` `core/geometry/*` `core/analysis/*` `tests/*`
- 共用：以上冻结文件
