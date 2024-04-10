import csv
import os

def save_to_csv(data_list, file_name='output.csv'):
    print(f"Saving data to {file_name} in directory {os.getcwd()}")
    
    with open(file_name, mode='w', newline='') as file:
        writer = csv.writer(file)
        
        # 写入头部
        writer.writerow(["object_name", "x", "y", "z"])
        
        # 写入数据
        for data in data_list:
            object_name, coordinates = data
            x, y, z = coordinates
            writer.writerow([object_name, x, y, z])
        
    print(f"Data saved to {file_name}")
        
# 手动输入的数据列表
data_list = [
    ("Cell Phone", (6.0, -1.0, 0.0)),
    ("Remote", (-6.0, 0.0, 0.0)),
    ("TV Monitor", (0.0, 0.0, 0.0))
]

# 调用函数将数据保存到CSV文件
save_to_csv(data_list)
