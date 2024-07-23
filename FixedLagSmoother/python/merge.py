def process_files(data1_file, data2_file, output_file):
    # 读取数据1和数据2
    with open(data1_file, 'r', encoding='utf-8') as file1:
        data1_lines = file1.readlines()
    
    with open(data2_file, 'r', encoding='utf-8') as file2:
        data2_lines = file2.readlines()

    # 创建一个字典来存储 data2 中每行第二个数据与对应行的映射
    data2_dict = {}
    for line in data2_lines:
        parts = line.split()
        key = parts[1]
        if key not in data2_dict:
            data2_dict[key] = []
        data2_dict[key].append(line)
    
    # 创建一个列表来存储处理后的行
    output_lines = []
    i = 0
    while i < len(data1_lines):
        line = data1_lines[i]
        parts = line.split()
        key = parts[1]
        # 如果 data2 中有对应的行，先添加这些行
        if key in data2_dict:
            output_lines.extend(data2_dict[key])
            del data2_dict[key]  # 确保每个键只处理一次
        # 添加 data1 的行
        output_lines.append(line)
        i += 1

    # 将结果写入输出文件
    with open(output_file, 'w', encoding='utf-8') as outfile:
        outfile.writelines(output_lines)


# 使用示例
data1_file = 'IMU_smartFactor.txt'  # 输入文件路径
data2_file = 'imu_keyID.txt'  # 输入文件路径
output_file = 'data.txt'  # 输出文件路径

process_files(data1_file, data2_file, output_file)
