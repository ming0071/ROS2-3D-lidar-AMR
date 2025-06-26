import pyDOE2
import pandas as pd
print(dir(pyDOE2))
# # 產生 L27 orthogonal array (13 columns, 27 rows, levels 3)
# design = oa("L27")

# # 取前 8 欄表示 8 個 critic 權重參數
# critics = [
#     "goal",
#     "goalAngle",
#     "preferFoeward",
#     "Osstacle",
#     "Cost",
#     "pathAlign",
#     "PathFollow",
#     "pathAngleAlign",
# ]
# design = design[:, : len(critics)]  # shape: (27, 8)

# # 等級 1/2/3 對應到實際值
# level_map = {
#     "goal": [3.0, 5.0, 7.0],
#     "goalAngle": [3.0, 10.0, 20.0],
#     "preferFoeward": [3.0, 5.0, 7.0],
#     "Osstacle": [5.0, 10.0, 20.0],
#     "Cost": [3.6, 3.8, 4.0],
#     "pathAlign": [10.0, 14.0, 20.0],
#     "PathFollow": [3.0, 5.0, 7.0],
#     "pathAngleAlign": [1.0, 2.0, 3.0],
# }

# # 映射成實際值
# mapped = []
# for row in design:
#     new_row = [level_map[c][level - 1] for c, level in zip(critics, row)]
#     mapped.append(new_row)

# # 轉為 dataframe 並存檔
# df = pd.DataFrame(mapped, columns=critics)
# df.to_csv("./data/Path/MPPI_experiment.csv", index=False)
