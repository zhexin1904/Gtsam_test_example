import pandas as pd

def process_files(txt1_path, txt2_path, output_path):
    # 读取 txt1 文件
    txt1_df = pd.read_csv(txt1_path, delim_whitespace=True, header=None)
    print(f"txt1 columns: {txt1_df.shape[1]}")  # 打印列数
    print(txt1_df.head())  # 打印前几行以检查数据格式

    # 检查 txt1 文件的列数是否正确
    if txt1_df.shape[1] < 3:
        raise ValueError("txt1 文件的列数不足，预期至少 3 列。")

    txt1_df.columns = ['Col1', 'ID', 'Timestamp1'] + [f"ExtraCol{i}" for i in range(4, txt1_df.shape[1] + 1)]

    # 读取 txt2 文件
    txt2_df = pd.read_csv(txt2_path, delim_whitespace=True, header=None)
    print(f"txt2 columns: {txt2_df.shape[1]}")  # 打印列数
    print(txt2_df.head())  # 打印前几行以检查数据格式

    # 检查 txt2 文件的列数是否正确
    if txt2_df.shape[1] < 8:
        raise ValueError("txt2 文件的列数不足，预期至少 8 列。")

    txt2_df.columns = ['Col1', 'Timestamp2', 'AccX', 'AccY', 'AccZ', 'GyroX', 'GyroY', 'GyroZ']

    # 初始化新的 DataFrame 用于存储结果
    result_df = pd.DataFrame(columns=['ID', 'AccX', 'AccY', 'AccZ', 'GyroX', 'GyroY', 'GyroZ'])

    # 处理 txt1 和 txt2 文件
    previous_id = None
    previous_timestamp = None
    for index, row in txt1_df.iterrows():
        current_id = row['ID']
        current_timestamp = row['Timestamp1']
        
        if previous_id is not None:
            # 找到在两个时间戳之间的行
            mask = (txt2_df['Timestamp2'] >= previous_timestamp) & (txt2_df['Timestamp2'] < current_timestamp)
            subset_df = txt2_df[mask]
            
            # 为这些行添加对应的 ID
            subset_df['ID'] = previous_id

            # 追加到结果 DataFrame
            result_df = pd.concat([result_df, subset_df[['ID', 'AccX', 'AccY', 'AccZ', 'GyroX', 'GyroY', 'GyroZ']]])

        previous_id = current_id
        previous_timestamp = current_timestamp

    # 保存结果到新的 txt 文件
    result_df.to_csv(output_path, sep=' ', index=False, header=False)

if __name__ == "__main__":
    # 替换成您的 txt 文件路径
    txt1_path = '/home/jason/DPGO/FixedLagSmoother/python/combined_kri_initials.txt'
    txt2_path = '/home/jason/DPGO/FixedLagSmoother/python/imu_data.txt'
    output_path = '/home/jason/DPGO/FixedLagSmoother/python/output.txt'
    
    process_files(txt1_path, txt2_path, output_path)
    print(f"Processed data has been saved to {output_path}")
