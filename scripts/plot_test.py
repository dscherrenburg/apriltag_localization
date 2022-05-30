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
    plt.figure(figsize=(w, h), dpi=d)
    fig, ax = plt.subplots()
    ax.axis([-2, 0, -1.5, 1.5])
    real_path_data, estimated_path_data, error, error_x, error_y, time = [], [], [], [], [], []
    
    with open(data_location + "/" + test_name + test_format, "r") as f:
        rospy.loginfo(data_location + "/" + test_name + test_format)

        reader = csv.reader(f)
        header = next(reader)
        first_values = next(reader)
        print(header)
        print(first_values)
        real_path_data.append((mpath.Path.MOVETO, (first_values[1], first_values[2])))
        estimated_path_data.append((mpath.Path.MOVETO, (first_values[3], first_values[4])))
        for row in reader:
            x_error = abs(float(row[1])-float(row[3]))
            y_error = abs(float(row[2])-float(row[4]))
            dist_error = np.sqrt(x_error**2 + y_error**2)
            error_x.append(x_error)
            error_y.append(y_error)
            error.append(dist_error)
            time.append(row[0])
            real_path_data.append((mpath.Path.CURVE4, (row[1], row[2])))
            estimated_path_data.append((mpath.Path.CURVE4, (row[3], row[4])))
        avg_error = sum(error) / len(error)

    real_codes, real_verts = zip(*real_path_data)
    estimated_codes, estimated_verts = zip(*estimated_path_data)
    real_path = mpath.Path(real_verts, real_codes)
    estimated_path = mpath.Path(estimated_verts, estimated_codes)

    real_patch = mpatches.PathPatch(real_path, edgecolor="red", facecolor="none", lw=2)
    estimated_patch = mpatches.PathPatch(estimated_path, edgecolor="green", facecolor="none", lw=2)

    # Paths
    dir_name = "Paths"
    save_location = os.path.join(plots_location, dir_name)
    try: 
        os.makedirs(save_location)
    except OSError: 
        pass
    ax.add_patch(real_patch)
    ax.add_patch(estimated_patch)
    plt.savefig(save_location  + "/" + test_name + ".png")

    # Error in x
    dir_name = "Error_in_x"
    save_location = os.path.join(plots_location, dir_name)
    try: 
        os.makedirs(save_location)
    except OSError: 
        pass
    plt.figure()
    plt.plot(time, error_x)
    plt.xlabel('Time')
    plt.ylabel('Error in x')
    plt.savefig(save_location  + "/" + test_name + ".png")

    # Error in y
    dir_name = "Error_in_y"
    save_location = os.path.join(plots_location, dir_name)
    try: 
        os.makedirs(save_location)
    except OSError: 
        pass
    plt.figure()
    plt.plot(time, error_y)
    plt.xlabel('Time')
    plt.ylabel('Error in y')
    plt.savefig(save_location  + "/" + test_name + ".png")

    # Error in distance
    dir_name = "Error_in_distance"
    save_location = os.path.join(plots_location, dir_name)
    try: 
        os.makedirs(save_location)
    except OSError: 
        pass
    plt.figure()
    plt.axhline(y = avg_error, color = 'r', linestyle = '-')
    plt.plot(time, error)
    plt.xlabel('Time')
    plt.ylabel('Error in distance')
    plt.savefig(save_location  + "/" + test_name + ".png")

def all_plots(data_location, plots_location, plotformat=".png"):
    data_files = os.listdir(data_location)
    for file in data_files:
        file_name = os.path.splitext(file)[0]
        file_format = os.path.splitext(file)[1]

        create_plots(data_location, plots_location, file_name, file_format)



if __name__ == '__main__':
    # save_name = rospy.get_param("~test_file_name")
    # save_location = rospy.get_param("~test_file_location")
    # save_format = rospy.get_param("~test_file_format")
    
    test_location = "/home/daan/localization_ws/src/apriltag_localization/tests"
    data_location = test_location + "/data"
    plots_location = test_location + "/plots"
    # save_name = "test_buffer30_error01"
    # data_format = ".csv"

    try: 
        os.makedirs(plots_location)
    except OSError as fail: 
        pass

    # create_plots(data_location, plots_location, save_name, data_format)
    all_plots(data_location, plots_location)