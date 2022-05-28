import matplotlib.path as mpath
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import csv
import numpy as np
import os
import rospy

def create_plots(test_location, test_name, test_format):
    w = 4
    h = 3
    d = 70
    plt.figure(figsize=(w, h), dpi=d)
    fig, ax = plt.subplots()
    ax.axis([-2, 0, -1.5, 1.5])
    real_path_data, estimated_path_data, error, error_x, error_y, time = [], [], [], [], [], []

    # directory = "plots__" + test_name
    # test_path = os.path.join(test_location, "tests")
    # directory_path = os.path.join(test_path, directory)
    # try: 
    #     os.mkdir(test_path)
    #     os.mkdir(directory_path)
    # except OSError as fail: 
    #     try:
    #         os.mkdir(directory_path)
    #     except OSError as fail2:
    #         pass
    
    with open(test_location + "/" + test_name + test_format, "r") as f:
        rospy.loginfo(test_location + "/" + test_name + test_format)
        reader = csv.reader(f)
        header = next(reader)
        first_values = next(reader)
        print(header)
        print(first_values)
        real_path_data.append((mpath.Path.MOVETO, (first_values[1], first_values[2])))
        estimated_path_data.append((mpath.Path.MOVETO, (first_values[3], first_values[4])))
        for row in reader:
            # print(row)
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
    # print(real_path_data)

    real_codes, real_verts = zip(*real_path_data)
    estimated_codes, estimated_verts = zip(*estimated_path_data)
    real_path = mpath.Path(real_verts, real_codes)
    estimated_path = mpath.Path(estimated_verts, estimated_codes)

    real_patch = mpatches.PathPatch(real_path, edgecolor="red", facecolor="none", lw=2)
    estimated_patch = mpatches.PathPatch(estimated_path, edgecolor="green", facecolor="none", lw=2)

    ax.add_patch(real_patch)
    ax.add_patch(estimated_patch)
    plt.savefig(test_location  + "/paths.png")

    plt.figure()
    plt.plot(time, error_x)
    plt.xlabel('Time')
    plt.ylabel('Error in x')
    plt.savefig(test_location + "/error_x.png")

    plt.figure()
    plt.plot(time, error_y)
    plt.xlabel('Time')
    plt.ylabel('Error in y')
    plt.savefig(test_location + "/error_y.png")

    plt.figure()
    plt.axhline(y = avg_error, color = 'r', linestyle = '-')
    plt.plot(time, error)
    plt.xlabel('Time')
    plt.ylabel('Error in distance')
    plt.savefig(test_location + "/error_distance.png")

if __name__ == '__main__':
    save_name = rospy.get_param("~test_file_name")
    save_location = rospy.get_param("~test_file_location")
    save_format = rospy.get_param("~test_file_format")
    # save_location = "/home/levijn/BEP/simulation_ws/"
    # save_name = "simple_test"
    # save_format = ".csv"
    create_plots(save_location, save_name, save_format)