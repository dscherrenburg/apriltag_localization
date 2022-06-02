import matplotlib.path as mpath
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import csv
import numpy as np
import os
import rospy

def create_plots(data_location, plots_location, test_name, test_format):
    w = 4
    h = 3
    d = 70
    plt.figure(0)
    fig, ax = plt.subplots()
    ax.axis([-1.5, -0.5, 1, 2.5])
    # [time, truex, truey, tagx, tagy, amclx, amcly]
    real_path_data, tag_path_data, amcl_path_data, tag_error, tag_error_x, tag_error_y, time = [], [], [], [], [], [], []
    
    with open(data_location + "/" + test_name + test_format, "r") as f:
        rospy.loginfo(data_location + "/" + test_name + test_format)

        reader = csv.reader(f)
        header = next(reader)
        first_values = next(reader)
        print(header)
        print(first_values)
        real_path_data.append((mpath.Path.MOVETO, (first_values[1], first_values[2])))
        tag_path_data.append((mpath.Path.MOVETO, (first_values[3], first_values[4])))
        amcl_path_data.append((mpath.Path.MOVETO, (first_values[5], first_values[6])))
        for row in reader:
            tag_x_error = abs(float(row[1])-float(row[3]))
            tag_y_error = abs(float(row[2])-float(row[4]))
            tag_dist_error = np.sqrt(tag_x_error**2 + tag_y_error**2)
            # tag_error_x.append(tag_x_error)
            # tag_error_y.append(tag_y_error)
            tag_error.append(tag_dist_error)
            time.append(row[0])
            real_path_data.append((mpath.Path.CURVE4, (row[1], row[2])))
            tag_path_data.append((mpath.Path.CURVE4, (row[3], row[4])))
            amcl_path_data.append((mpath.Path.CURVE4, (row[5], row[6])))
        avg_error = sum(tag_error) / len(tag_error)

    real_codes, real_verts = zip(*real_path_data)
    real_path = mpath.Path(real_verts, real_codes)
    real_patch = mpatches.PathPatch(real_path, edgecolor="red", facecolor="none", lw=2, label="real path")

    tag_codes, tag_verts = zip(*tag_path_data)
    tag_path = mpath.Path(tag_verts, tag_codes)
    tag_patch = mpatches.PathPatch(tag_path, edgecolor="green", facecolor="none", lw=2, label="tag estimation path")

    amcl_codes, amcl_verts = zip(*amcl_path_data)
    amcl_path = mpath.Path(amcl_verts, amcl_codes)
    amcl_patch = mpatches.PathPatch(amcl_path, edgecolor="blue", facecolor="none", lw=2, label="amcl estimation path")

    # Paths
    dir_name = "paths"
    save_location = os.path.join(plots_location, dir_name)
    try: 
        os.makedirs(save_location)
    except OSError: 
        pass
    ax.add_patch(real_patch)
    ax.add_patch(tag_patch)
    ax.add_patch(amcl_patch)
    ax.plot(-0.92, 1.38, 'go', label='Starting point')
    plt.legend()
    plt.title("Path estimation")
    plt.xlabel("x")
    plt.ylabel("y")
    plt.savefig(save_location  + "/" + test_name + ".png")

    # # Error in x
    # dir_name = "tag_error_in_x"
    # save_location = os.path.join(plots_location, dir_name)
    # try: 
    #     os.makedirs(save_location)
    # except OSError: 
    #     pass
    # plt.figure()
    # plt.plot(time, tag_error_x)
    # plt.xlabel('Time')
    # plt.ylabel('Error in x')
    # plt.title("Error in x of tag estimation")
    # plt.savefig(save_location  + "/" + test_name + ".png")

    # # Error in y
    # dir_name = "tag_error_in_y"
    # save_location = os.path.join(plots_location, dir_name)
    # try: 
    #     os.makedirs(save_location)
    # except OSError: 
    #     pass
    # plt.figure()
    # plt.plot(time, tag_error_y)
    # plt.xlabel('Time')
    # plt.ylabel('Error in y')
    # plt.title("Error in y of tag estimation")
    # plt.savefig(save_location  + "/" + test_name + ".png")

    # Error in distance
    dir_name = "tag_error_in_distance"
    save_location = os.path.join(plots_location, dir_name)
    try: 
        os.makedirs(save_location)
    except OSError: 
        pass
    plt.figure()
    plt.axhline(y = avg_error, color = 'r', linestyle = '-')
    plt.plot(time, tag_error)
    plt.xlabel('Time')
    plt.ylabel('Error in distance')
    plt.title("Error in global distance of tag estimation")
    plt.savefig(save_location  + "/" + test_name + ".png")

def create_data_table(data_location, table_location, data_name, table_name="data_table", data_format=".csv", table_format=".csv"):

    table = open(table_location + "/" + table_name + table_format, 'a')
    writer = csv.writer(table)

    f = open(table_location + "/" + table_name + table_format, 'r')
    table_reader = csv.reader(f)
    line_count = len(list(table_reader))
    if line_count == 0:
        header = ["Buffer size", "Maximum error", "Maximum time difference", "Average distance error"]
        writer.writerow(header)
    
    "test3_buf10_er020_tdiff02"
    buffer_size = int(data_name[9:11])
    max_error = float(data_name[14:17])/100
    time_difference = float(data_name[-2:])/10

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

def plot_table(table_location, table_name="data_table", table_format=".csv"):
    table = open(table_location + "/" + table_name + table_format, 'r')
    table_reader = csv.reader(table)
    header = next(table_reader)
    buffer_size, max_error, time_difference, avg_error = [], [], [], []
    for row in table_reader:
        buffer_size.append(row[0])
        max_error.append(row[1])
        time_difference.append(row[2])
        avg_error.append(row[3])

    plt.figure()
    plt.plot(buffer_size, max_error, label="Maximum error")
    # plt.plot(buffer_size, time_difference, label="Maximum time difference")
    # plt.plot(buffer_size, avg_error, label="Average distance error")
    plt.xlabel("Buffer size")
    plt.ylabel("Error")
    plt.legend()
    plt.title("Error vs buffer size")
    plt.savefig(table_location + "/" + table_name + ".png")

def all_plots(data_location, plots_location, data_name=None, data_format=".csv", plotformat=".png"):
    if data_name is None:
        data_files = os.listdir(data_location)
        for file in data_files:
            file_name = os.path.splitext(file)[0]
            file_format = os.path.splitext(file)[1]

            # create_data_table(data_location, plots_location, file_name, data_format=file_format)
            create_plots(data_location, plots_location, file_name, file_format)
    else:
        create_plots(data_location, plots_location, data_name, data_format)



if __name__ == '__main__':
    # save_name = rospy.get_param("~test_file_name")
    # save_location = rospy.get_param("~test_file_location")
    # save_format = rospy.get_param("~test_file_format")
    
    # test_location = "/home/daan/localization_ws/src/apriltag_localization/tests"
    test_location = "/home/levijn/BEP/simulation_ws/src/apriltag_localization/straight_tests"
    data_location = test_location + "/data"
    plots_location = test_location + "/plots"
    # save_name = "test3_buf10_er02_tdiff02"
    # data_format = ".csv"

    try: 
        os.makedirs(plots_location)
    except OSError as fail: 
        pass

    # create_plots(data_location, plots_location, save_name, data_format)
    all_plots(data_location, plots_location)

