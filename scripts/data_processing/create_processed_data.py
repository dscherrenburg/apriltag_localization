import csv
import os
import numpy as np

def create_data_table_per_test(data_location, table_location, data_name, table_name="data_table", data_format=".csv", table_format=".csv"):
    try:
        with open(table_location + "/" + table_name + table_format, 'r') as f:
            reader = csv.reader(f)
            line_count = len(list(reader))
    except IOError:
        line_count = 0
            
    with open(table_location + "/" + table_name + table_format, 'a+') as table:
        writer = csv.writer(table)
        if line_count == 0:
            header = ["Buffer size", "Maximum error", "Maximum time difference", "Average distance error"]
            writer.writerow(header)

        buffer_size = int(data_name[8:10])
        max_error = float(data_name[13:16])/100
        time_difference = float(data_name[22:24])/10

        tag_error, tag_error_x, tag_error_y, time = [], [], [], []

        with open(data_location + "/" + data_name + data_format, "r") as data:
            data_reader = csv.reader(data)
            header = next(data_reader)
            for row in data_reader:
                tag_x_error = abs(float(row[1])-float(row[3]))
                tag_y_error = abs(float(row[2])-float(row[4]))
                tag_dist_error = np.sqrt(tag_x_error**2 + tag_y_error**2)
                tag_error_x.append(tag_x_error)
                tag_error_y.append(tag_y_error)
                tag_error.append(tag_dist_error)
                time.append(row[0])
            avg_error = sum(tag_error) / len(tag_error)
    
        data = [buffer_size, max_error, time_difference, avg_error]
        writer.writerow(data)


def create_data_table_per_test_full(data_location, table_location, table_name="data_table"):
    data_files = os.listdir(data_location)
    for file in data_files:
        file_name = os.path.splitext(file)[0]
        file_format = os.path.splitext(file)[1]
        create_data_table_per_test(data_location, table_location, file_name, table_name=table_name,  data_format=".csv")


def calculate_mean_std(data):
    """
    Calculate mean and standard deviation of the data.
    :param data: data to calculate mean and standard deviation
    :return: mean and standard deviation of the data
    """
    mean = np.mean(data)
    std = np.std(data)
    return mean, std

def create_data_table_total_avg_error(processed_data_location, processed_data_name, save_location, save_name):
    data_dict = {}
    with open(processed_data_location + "/" + processed_data_name + ".csv", 'r') as table:
        table_reader = csv.reader(table)
        header = next(table_reader)

        for row in table_reader:
            buffer_size, max_error, time_difference, avg_error = [float(row[i]) for i in range(len(row))]
            if buffer_size not in data_dict:
                data_dict[buffer_size] = {max_error: [avg_error]}
            else:
                if max_error not in data_dict[buffer_size]:
                    data_dict[buffer_size][max_error] = [avg_error]
                else:
                    data_dict[buffer_size][max_error].append(avg_error)
    
    with open(save_location + "/" + save_name + ".csv", 'w') as new_table:
        new_table_w = csv.writer(new_table)
        header = ["Buffer size", "Maximum error", "Average error"]
        new_table_w.writerow(header)

        for buffer_size in data_dict:
            for max_error in data_dict[buffer_size]:
                mean_error, std_error = calculate_mean_std(data_dict[buffer_size][max_error])
                
                data = [buffer_size, max_error, mean_error]
                new_table_w.writerow(data)
        


if __name__ == "__main__":
    test_location = "./straight_tests/mean"
    # test_location = "/home/levijn/BEP/simulation_ws/src/apriltag_localization/straight_tests"
    data_location = test_location + "/data"
    table_location = test_location
    
    create_data_table_per_test_full(data_location, table_location, table_name="test_table")
    create_data_table_total_avg_error(table_location, "test_table", table_location, "total_avg_error_table")
    
    
    