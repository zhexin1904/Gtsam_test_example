def read_lines(filename):
    with open(filename, 'r') as file:
        lines = file.readlines()
    return lines

def write_lines(lines, filename):
    with open(filename, 'w') as file:
        for line in lines:
            file.write(line)

def rearrange_lines(lines):
    reordered_lines = []
    for i in range(0, len(lines), 6):
        group = [
            lines[i + 4],  # x5
            lines[i + 2],  # v5
            lines[i],      # b5
            lines[i + 5],  # x6
            lines[i + 3],  # v6
            lines[i + 1]   # b6
        ]
        reordered_lines.extend(group)
    return reordered_lines

def process_file(input_filename, output_filename):
    lines = read_lines(input_filename)
    reordered_lines = rearrange_lines(lines)
    write_lines(reordered_lines, output_filename)
    print(f"File processing complete. Check the output file: {output_filename}")

if __name__ == "__main__":
    input_filename = "initiasl_values.txt"  # 替换为你的输入文件名
    output_filename = "initiasl_values_processed.txt"  # 替换为你的输出文件名
    process_file(input_filename, output_filename)
