# 北京香山真实道路选线成本数据

## 应用场景

本数据用于“基于土地覆盖与坡度综合成本的山区道路最小成本选线”。成本栅格值表示每米相对道路建设成本；实际出现的水体、无有效分类区域及坡度大于35度的区域作为NoData障碍。

## 研究区与输出参数

- 中心位置：39.9909972 N, 116.1796917 E（北京香山）
- 目标坐标系：EPSG:32650（WGS 84 / UTM zone 50N）
- 像元大小：30 m x 30 m
- 栅格大小：334列 x 334行
- 实际覆盖范围：10020 m x 10020 m
- UTM范围：左424950.000、右434970.000、下4422090.000、上4432110.000
- 有效像元：106529
- NoData像元：5027
- 有效成本范围：1.000 - 96.000
- 有效区域坡度范围：0.000 - 35.000度
- NoData值：-9999

## 原始数据

### ESA WorldCover 2021 v200

- 原始文件：`ESA_WorldCover_10m_2021_v200_N39E114_Map.tif`
- 原始分辨率：约10米
- 原始坐标系：EPSG:4326
- 来源：https://esa-worldcover.org/en/data-access
- 许可：Creative Commons Attribution 4.0 International
- 署名：© ESA WorldCover project 2021 / Contains modified Copernicus Sentinel data (2021) processed by ESA WorldCover consortium.

### Copernicus DEM GLO-30

- 原始文件：`Copernicus_DSM_COG_10_N39_00_E116_00_DEM.tif`、`Copernicus_DSM_COG_10_N40_00_E116_00_DEM.tif`
- 原始分辨率：约30米
- 原始坐标系：EPSG:4326
- 来源：https://registry.opendata.aws/copernicus-dem/
- 数据说明：https://dataspace.copernicus.eu/explore-data/data-collections/copernicus-contributing-missions/collections-description/COP-DEM
- 数据类型：数字表面模型（DSM）

三个完整原始瓦片（土地覆盖1幅、高程2幅）保存在项目仓库外的`GIS算法/原始数据/北京香山/`目录，不提交到GitHub。

## DEM接缝与缺失检查

两幅DEM在北纬40度附近相接。原始瓦片边界坐标完全一致，两个瓦片均无NoData像元；合并并投影到10公里研究区后，DEM的NoData像元数仍为0。

- 跨瓦片边界相邻像元绝对高程差：均值10.20米、中位数10.10米。
- N39瓦片内部边缘相邻像元差：均值9.91米、中位数9.30米。
- N40瓦片内部边缘相邻像元差：均值9.79米、中位数9.68米。
- 跨边界有符号平均差：-0.70米。

边界变化与瓦片内部自然地形起伏处于同一量级，没有数据空洞或显著系统高程跳变。地图上的黑色区域来自水体、坡度大于35度或未映射类别，不是DEM接缝缺失。

## 土地覆盖成本

研究区实际只出现7类土地覆盖，因此成本表仅保留实际用到的类别：

| WorldCover编码 | 类型 | 单位成本 | 像元数 | 占比 |
|---:|---|---:|---:|---:|
| 10 | 林地 | 6.0 | 81795 | 73.32% |
| 30 | 草地 | 1.0 | 2552 | 2.29% |
| 40 | 农田 | 3.0 | 4770 | 4.28% |
| 50 | 建成区 | 12.0 | 21720 | 19.47% |
| 60 | 裸地/稀疏植被 | 1.5 | 610 | 0.55% |
| 80 | 永久水体 | NoData | 95 | 0.09% |
| 90 | 湿地 | 15.0 | 14 | 0.01% |

## 坡度成本系数

| 坡度 | 系数 |
|---|---:|
| 0-5度 | 1.0 |
| 5-10度 | 1.5 |
| 10-15度 | 2.0 |
| 15-25度 | 4.0 |
| 25-35度 | 8.0 |
| 大于35度 | NoData |

最终成本计算公式：`cost = landcover_cost * slope_factor`。

土地覆盖采用最近邻法投影到30米网格，DEM采用双线性法投影；坡度在UTM米制坐标下计算。两种数据使用相同的范围、像元大小、行列数和网格起点。

## 起终点

提供5组位于最大连通区域内的起终点。每个GeoJSON只包含一个点。按照课程要求，所有输入点都精确位于有效像元中心；`cases.json`同时记录其row、col、中心坐标及零偏移量，便于自动校验。

| 编号 | 起点row,col | 终点row,col | 直线距离/m | 文件 |
|---|---|---|---:|---|
| 1 | 250, 283 | 83, 50 | 8600.0 | `start_01.geojson` / `end_01.geojson` |
| 2 | 66, 267 | 267, 66 | 8527.7 | `start_02.geojson` / `end_02.geojson` |
| 3 | 167, 29 | 167, 307 | 8340.0 | `start_03.geojson` / `end_03.geojson` |
| 4 | 33, 167 | 300, 167 | 8010.0 | `start_04.geojson` / `end_04.geojson` |
| 5 | 217, 66 | 116, 250 | 6296.9 | `start_05.geojson` / `end_05.geojson` |

`start.geojson`和`end.geojson`复制第1组，作为默认插件功能测试输入；`cases.json`记录全部5组输入坐标、中心偏移和期望映射节点。

## 文件

- `cost.tif`：最终单波段Float32成本栅格。
- `start.geojson`、`end.geojson`：默认第1组起终点。
- `start_01.geojson`至`start_05.geojson`：5个起点文件。
- `end_01.geojson`至`end_05.geojson`：5个终点文件。
- `cases.json`：所有起终点组合及坐标信息。
- `README.md`：数据来源、成本规则与处理说明。

本成本体系为课程实验中的相对成本模型，不代表实际工程造价；未考虑地质、已有道路、权属、法规和详细生态红线。
