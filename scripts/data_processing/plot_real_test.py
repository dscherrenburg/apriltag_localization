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
    ax.axis([-1.5, 1.5, -1.5, 0.4])

    tag_path_data, amcl_path_data, time = [], [], []
    
    with open(data_location + "/" + test_name + test_format, "r") as f:
        rospy.loginfo(data_location + "/" + test_name + test_format)

        reader = csv.reader(f)
        header = next(reader)
        first_values = next(reader)
        tag_path_data.append((mpath.Path.MOVETO, (first_values[1], first_values[2])))
        amcl_path_data.append((mpath.Path.MOVETO, (first_values[3], first_values[4])))
        for row in reader:
            time.append(row[0])
            tag_path_data.append((mpath.Path.CURVE4, (row[1], row[2])))
            amcl_path_data.append((mpath.Path.CURVE4, (row[3], row[4])))

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

    ax.add_patch(tag_patch)
    ax.add_patch(amcl_patch)
    # ax.plot(-1, 2, 'go', label='Starting point')
    # ax.plot(0, 0, 'go', c="r", label='Origin')
    plt.legend()
    plt.title("Path estimation")
    plt.xlabel("x")
    plt.ylabel("y")
    plt.savefig(save_location  + "/" + test_name + ".png")
    
    plt.close()


def all_plots(data_location, plots_location):
    data_files = os.listdir(data_location)
    for file in data_files:
        file_name = os.path.splitext(file)[0]
        file_format = os.path.splitext(file)[1]

        print(file_name)
        create_plots(data_location, plots_location, file_name, file_format)




if __name__ == '__main__':
    
    # test_location = "./turn_tests/mean"
    # test_location = "/home/beau/localization_ws/src/apriltag_localization/turn_tests/median"
    test_location = "/home/levijn/BEP/simulation_ws/src/apriltag_localization/real_tests_straight/median"

    data_location = test_location + "/data"
    plots_location = test_location + "/plots"

    try: 
        os.makedirs(plots_location)
    except OSError as fail: 
        pass

    all_plots(data_location, plots_location)
