import csv
import os
import numpy as np
import matplotlib.pyplot as plt

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
            header = ["Buffer size", "Maximum error", "Maximum time difference", "Average distance error", "Average enpoint error", "Average distance error amcl"]
            writer.writerow(header)

        buffer_size = int(data_name[8:10])
        max_error = float(data_name[13:16])/100
        time_difference = float(data_name[22:24])/10

        tag_error, tag_error_x, tag_error_y, time, amcl_error_x, amcl_error_y, amcl_error = [], [], [], [], [], [], []

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
                
                amcl_x_error = abs(float(row[1])-float(row[5]))
                amcl_y_error = abs(float(row[2])-float(row[6]))
                
                amcl_dist_error = np.sqrt(amcl_x_error**2 + amcl_y_error**2)
                amcl_error_x.append(amcl_x_error)
                amcl_error_y.append(amcl_y_error)
                amcl_error.append(amcl_dist_error)
                time.append(row[0])
            avg_error = sum(tag_error) / len(tag_error)
            avg_amcl_error = sum(amcl_error) / len(amcl_error)
        
        endpoint_error = sum(tag_error[-3:]) / len(tag_error[-3:])
    
        data = [buffer_size, max_error, time_difference, avg_error, endpoint_error, avg_amcl_error]
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

def create_data_table_total_avg_error(processed_data_location, processed_data_name, save_location, save_name, plot=True):
    data_dict = {}
    avg_amcl_error = []
    with open(processed_data_location + "/" + processed_data_name + ".csv", 'r') as table:
        table_reader = csv.reader(table)
        header = next(table_reader)

        for row in table_reader:
            buffer_size, max_error, time_difference, avg_error, endpoint_error, amcl_error = [float(row[i]) for i in range(len(row))]
            avg_amcl_error.append(amcl_error)
            if buffer_size not in data_dict:
                data_dict[buffer_size] = {max_error: ([avg_error],[endpoint_error])}
            else:
                if max_error not in data_dict[buffer_size]:
                    data_dict[buffer_size][max_error] = ([avg_error], [endpoint_error])
                else:
                    data_dict[buffer_size][max_error][0].append(avg_error)
                    data_dict[buffer_size][max_error][1].append(endpoint_error)
                    
    plot_dict = {}
    for buffer_size in data_dict:
        plot_dict[buffer_size] = {"x": [], "y": [], "y_endpoint": []}

    
    with open(save_location + "/" + save_name + ".csv", 'w') as new_table:
        new_table_w = csv.writer(new_table)
        header = ["Buffer size", "Maximum error", "Average error", "Average end error"]
        new_table_w.writerow(header)

        for buffer_size in data_dict:
            for max_error in data_dict[buffer_size]:
                mean_error, std_error = calculate_mean_std(data_dict[buffer_size][max_error][0])
                
                end_data = data_dict[buffer_size][max_error][1]
                mean_end_error = sum(end_data) / len(end_data)
                
                data = [buffer_size, max_error, mean_error, mean_end_error]
                new_table_w.writerow(data)
                
                plot_dict[buffer_size]["x"].append(max_error)
                plot_dict[buffer_size]["y"].append(mean_error)
                plot_dict[buffer_size]["y_endpoint"].append(mean_end_error)
        
    if plot:
        plt.figure(1)
        colors = ["r","b","g","y","c","m","k"]
        for i, buffersize in enumerate(plot_dict):
            zipped_list = zip(plot_dict[buffersize]["x"], plot_dict[buffersize]["y"], plot_dict[buffersize]["y_endpoint"])
            sorted_list = sorted(zipped_list, key=lambda x: x[0])
            tuples = zip(*sorted_list)
            x, y, y_end = [list(tuple) for tuple in tuples]
            
            
            plt.plot(x, y, c=colors[i], label="Buffer size: " + str(int(buffersize)))
        
        plt.xlabel("Maximum error")
        plt.ylabel("Average error")
        plt.title("Average error per buffer size vs maximum error")
        plt.legend()
        plt.savefig(save_location + "/" + save_name + ".png")
        
        plt.figure(2)
        for i, buffersize in enumerate(plot_dict):
            zipped_list = zip(plot_dict[buffersize]["x"], plot_dict[buffersize]["y"], plot_dict[buffersize]["y_endpoint"])
            sorted_list = sorted(zipped_list, key=lambda x: x[0])
            tuples = zip(*sorted_list)
            x, y, y_end = [list(tuple) for tuple in tuples]
            
            
            plt.plot(x, y_end, c=colors[i], label="Buffer size: " + str(int(buffersize)))
        
        plt.xlabel("Maximum error")
        plt.ylabel("Average end point error")
        plt.title("Average end point error per buffer size vs maximum error")
        plt.legend()
        plt.savefig(save_location + "/" + save_name + "_endpoint_error" + ".png")


def calculate_amcl_error(processed_data_location, processed_data_name):
    with open(processed_data_location + "/" + processed_data_name + ".csv", 'r') as table:
        table_reader = csv.reader(table)
        header = next(table_reader)

        amcl_errors = []
        for row in table_reader:
            buffer_size, max_error, time_difference, avg_error, avg_endpoint_error, amcl_error = [float(row[i]) for i in range(len(row))]
            amcl_errors.append(amcl_error)
        avg_amcl_error = sum(amcl_errors) / len(amcl_errors)
    return avg_amcl_error


if __name__ == "__main__":
    # test_location = "./straight_tests/mean"
    test_location = "/home/levijn/BEP/simulation_ws/src/apriltag_localization/straight_tests/median"
    data_location = test_location + "/data"
    table_location = test_location
    
    create_data_table_per_test_full(data_location, table_location, table_name="median_processed_test_data_table")
    create_data_table_total_avg_error(table_location, "median_processed_test_data_table", table_location, "median_total_avg_error_table")
    print(calculate_amcl_error(table_location, "median_processed_test_data_table"))
    
    
    