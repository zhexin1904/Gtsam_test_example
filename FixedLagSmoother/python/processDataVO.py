# def remove_lines_starting_with_l(input_file, output_file):
#     with open(input_file, 'r') as infile, open(output_file, 'w') as outfile:
#         for line in infile:
#             if not line.lstrip().lower().startswith('x'):
#                 outfile.write(line)

# # 示例使用
# input_file = 'frontendData_VO.txt'
# output_file = 'frontendData_VO_processed.txt'
# remove_lines_starting_with_l(input_file, output_file)
def remove_lines_starting_with_b_or_v(input_file, output_file):
    with open(input_file, 'r') as infile, open(output_file, 'w') as outfile:
        for line in infile:
            if not line.lstrip().lower().startswith(('b', 'v')):
                outfile.write(line)

# 示例使用
input_file = 'initiasl_values_processed_VO.txt'
output_file = 'initiasl_values_processed_VO_2.txt'
remove_lines_starting_with_b_or_v(input_file, output_file)
