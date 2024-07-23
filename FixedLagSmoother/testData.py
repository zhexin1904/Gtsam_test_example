def process_txt(input_file, max_lines):
    try:
        # 打开输入文件读取
        with open(input_file, 'r', encoding='utf-8') as infile:
            # 用于存储第四个数据
            fourth_data_set = set()
            
            for i, line in enumerate(infile):
                if i >= max_lines:
                    break
                
                # 以空格划分每行数据
                data = line.strip().split()
                
                # 确保每行至少有四个数据
                if len(data) >= 4:
                    # 获取第四个数据并加入集合
                    fourth_data_set.add(data[3])
        
        # 输出结果
        print(f"第四个数据出现的种类数: {len(fourth_data_set)}")
        print("第四个数据的种类:", fourth_data_set)

    except Exception as e:
        print(f"处理文件时发生错误: {e}")

# 使用示例
input_file = 'reduced_frontendData_VO.txt'
max_lines = 656  # 人为指定的行数
process_txt(input_file, max_lines)
