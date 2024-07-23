# from collections import Counter

# def count_fourth_column(input_file, output_file):
#     with open(input_file, 'r', encoding='utf-8') as file:
#         lines = file.readlines()

#     fourth_column_data = [line.split()[3] for line in lines if len(line.split()) > 3]
#     counts = Counter(fourth_column_data)

#     with open(output_file, 'w', encoding='utf-8') as file:
#         for data, count in counts.items():
#             file.write(f"{data} {count}\n")

# input_file = '/home/jason/DPGO/FixedLagSmoother/frontendData_VO.txt'
# output_file = '/home/jason/DPGO/FixedLagSmoother/frontendData_VO_count.txt'
# count_fourth_column(input_file, output_file)

from collections import Counter

def count_data_types(input_file, output_file):
    with open(input_file, 'r', encoding='utf-8') as file:
        lines = file.readlines()

    data_types = [line.split()[3] for line in lines if len(line.split()) > 3]
    counts = Counter(data_types)

    with open(output_file, 'w', encoding='utf-8') as file:
        for data_type, count in counts.items():
            file.write(f"{data_type} {count}\n")

input_file = '/home/jason/DPGO/FixedLagSmoother/frontendData_VO.txt'
output_file = '/home/jason/DPGO/FixedLagSmoother/frontendData_VO_count.txt'
count_data_types(input_file, output_file)
