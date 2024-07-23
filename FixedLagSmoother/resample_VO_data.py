def process_txt(input_file, output_file):
    try:
        # 读取输入文件
        with open(input_file, 'r', encoding='utf-8') as infile:
            lines = infile.readlines()
        
        # 删除所有奇数行
        even_lines = [line for index, line in enumerate(lines) if index % 2 == 0]

        # 写入输出文件
        with open(output_file, 'w', encoding='utf-8') as outfile:
            outfile.writelines(even_lines)
        
        print(f"处理完成，结果已保存到 {output_file}")

    except Exception as e:
        print(f"处理文件时发生错误: {e}")

# 使用示例
input_file = 'reduced_frontendData_VO.txt'
output_file = 'reduced_frontendData_VO_.txt'
process_txt(input_file, output_file)
