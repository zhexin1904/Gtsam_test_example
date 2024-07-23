def process_file(input_file, output_file):
    with open(input_file, 'r', encoding='utf-8') as infile, open(output_file, 'w', encoding='utf-8') as outfile:
        for line in infile:
            if line.startswith('x'):
                parts = line.split(' ', 1)
                if len(parts) == 2:
                    number = parts[0][1:]  # 提取数字部分
                    rest_of_line = parts[1]
                    outfile.write(f"x {number} {rest_of_line}")
                else:
                    outfile.write(line)  # 如果行不符合预期格式，保持原样
            else:
                outfile.write(line)  # 如果行不以x开头，保持原样

# 使用示例
input_file = 'imu_keyID.txt'  # 输入文件路径
output_file = 'imu_keyID_processed.txt'  # 输出文件路径

process_file(input_file, output_file)