import pandas as pd

def process_sensor_data(file_path):
    # 读取txt文件，跳过第一列的'IMU'字符串
    data = pd.read_csv(file_path, sep=' ', header=None, usecols=[1,2,3,4,5,6,7])

    # 打印前几行数据检查列数
    print("Data preview:\n", data.head())

    # 检查列数
    num_columns = data.shape[1]
    if num_columns < 7:
        raise ValueError("数据列数不足，至少需要7列")

    # 提取时间戳和传感器数据
    timestamps = data.iloc[:, 0]  # 第二列时间戳，索引0
    accel_x = data.iloc[:, 1]     # 第三列加速度X，索引1
    accel_y = data.iloc[:, 2]     # 第四列加速度Y，索引2
    accel_z = data.iloc[:, 3]     # 第五列加速度Z，索引3
    gyro_x = data.iloc[:, 4]      # 第六列角速度X，索引4
    gyro_y = data.iloc[:, 5]      # 第七列角速度Y，索引5
    gyro_z = data.iloc[:, 6]      # 第八列角速度Z，索引6

    # 筛选出前3秒的数据
    initial_time = timestamps.iloc[0]
    three_seconds_data = data[timestamps - initial_time <= 3]

    # 计算加速度和角速度的算术平均值
    accel_mean = three_seconds_data.iloc[:, 1:4].mean()
    gyro_mean = three_seconds_data.iloc[:, 4:7].mean()

    print("加速度XYZ的算术平均值:")
    print(accel_mean)
    print("角速度XYZ的算术平均值:")
    print(gyro_mean)


file_path = '/home/jason/DPGO/FixedLagSmoother/python/imu_data.txt'  # 替换为你的txt文件路径
process_sensor_data(file_path)
