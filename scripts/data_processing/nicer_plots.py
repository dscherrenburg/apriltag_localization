import csv
from ctypes.wintypes import tagRECT
import matplotlib.pyplot as plt
import matplotlib.path as mpath
import matplotlib.patches as mpatches

def plot_2_paths():
    real_path_file_1 = "C:\\Users\\levij\\TU Delft\\BEP\\apriltag_localization\\real_tests_straight\\median\\data\\test_buf20_er025_tdiff02_5.csv"
    real_path_file_2 = "C:\\Users\\levij\\TU Delft\\BEP\\apriltag_localization\\real_tests_straight\\median\\data\\test_buf20_er025_tdiff02_3.csv"
    
    fig, (ax1, ax2) = plt.subplots(1, 2, sharex='col', sharey='row')
    axsize = [-0.4, 0.4, -1.1, 0.3]
    ax1.axis(axsize)
    ax2.axis(axsize)
    tag_path_data_1, amcl_path_data_1, time_1 = [], [], []
    tag_path_data_2, amcl_path_data_2, time_2 = [], [], []
    
    with open(real_path_file_1, "r") as f:
        reader = csv.reader(f)
        header = next(reader)
        first_values = next(reader)
        tag_path_data_1.append((mpath.Path.MOVETO, (first_values[1], first_values[2])))
        amcl_path_data_1.append((mpath.Path.MOVETO, (first_values[3], first_values[4])))
        for row in reader:
            time_1.append(row[0])
            tag_path_data_1.append((mpath.Path.CURVE4, (row[1], row[2])))
            amcl_path_data_1.append((mpath.Path.CURVE4, (row[3], row[4])))
    
    tag_codes, tag_verts = zip(*tag_path_data_1)
    tag_path = mpath.Path(tag_verts, tag_codes)
    tag_patch = mpatches.PathPatch(tag_path, edgecolor="green", facecolor="none", lw=2, label="tag estimation path")

    amcl_codes, amcl_verts = zip(*amcl_path_data_1)
    amcl_path = mpath.Path(amcl_verts, amcl_codes)
    amcl_patch = mpatches.PathPatch(amcl_path, edgecolor="blue", facecolor="none", lw=2, label="amcl estimation path")

    ax1.add_patch(tag_patch)
    ax1.add_patch(amcl_patch)
    
    with open(real_path_file_2, "r") as f:
        reader = csv.reader(f)
        header = next(reader)
        first_values = next(reader)
        tag_path_data_2.append((mpath.Path.MOVETO, (first_values[1], first_values[2])))
        amcl_path_data_2.append((mpath.Path.MOVETO, (first_values[3], first_values[4])))
        for row in reader:
            time_2.append(row[0])
            tag_path_data_2.append((mpath.Path.CURVE4, (row[1], row[2])))
            amcl_path_data_2.append((mpath.Path.CURVE4, (row[3], row[4])))

    tag_codes, tag_verts = zip(*tag_path_data_2)
    tag_path = mpath.Path(tag_verts, tag_codes)
    tag_patch = mpatches.PathPatch(tag_path, edgecolor="green", facecolor="none", lw=2, label="Proposed method path")

    amcl_codes, amcl_verts = zip(*amcl_path_data_2)
    amcl_path = mpath.Path(amcl_verts, amcl_codes)
    amcl_patch = mpatches.PathPatch(amcl_path, edgecolor="blue", facecolor="none", lw=2, label="AMCL path")

    ax2.add_patch(tag_patch)
    ax2.add_patch(amcl_patch)
    
    ax1.set_title("Test 1", fontsize=8)
    ax1.set_xlabel("x (m)")
    ax1.set_ylabel("y (m)")
    
    ax2.set_title("Test 2", fontsize=8)
    ax2.set_xlabel("x (m)")
    ax2.legend()
    
    fig.suptitle("Path estimation of AMCL and Proposed method\n(Straight-line test)", fontsize=11)
    plt.savefig("./real_straight_line_tests_side_by_side.png", dpi=300)
    plt.show()
    


def realpath_vs_simpath():
    real_path_file = "C:\\Users\\levij\\TU Delft\\BEP\\apriltag_localization\\real_tests_straight\\median\\data\\test_buf20_er025_tdiff02_5.csv"
    sim_path_file = "C:\\Users\\levij\\TU Delft\\BEP\\apriltag_localization\\straight_tests\\median\\data\\test_buf20_er025_tdiff02_3.csv"
    
    fig, (ax1, ax2) = plt.subplots(1, 2, sharex='col')
    axsize = [-0.4, 0.4, -1.1, 0.3]
    ax1.axis([-1.4, -0.6, 0.9, 2.3])
    ax2.axis(axsize)
    tag_path_data_1, amcl_path_data_1, time_1 = [], [], []
    real_path_data, tag_path_data_2, amcl_path_data_2, time_2 = [], [], [], []
    
    with open(real_path_file, "r") as f:
        reader = csv.reader(f)
        header = next(reader)
        first_values = next(reader)
        tag_path_data_1.append((mpath.Path.MOVETO, (first_values[1], first_values[2])))
        amcl_path_data_1.append((mpath.Path.MOVETO, (first_values[3], first_values[4])))
        for row in reader:
            time_1.append(row[0])
            tag_path_data_1.append((mpath.Path.CURVE4, (row[1], row[2])))
            amcl_path_data_1.append((mpath.Path.CURVE4, (row[3], row[4])))
    
    tag_codes, tag_verts = zip(*tag_path_data_1)
    tag_path = mpath.Path(tag_verts, tag_codes)
    tag_patch = mpatches.PathPatch(tag_path, edgecolor="green", facecolor="none", lw=2, label="Proposed method path")

    amcl_codes, amcl_verts = zip(*amcl_path_data_1)
    amcl_path = mpath.Path(amcl_verts, amcl_codes)
    amcl_patch = mpatches.PathPatch(amcl_path, edgecolor="blue", facecolor="none", lw=2, label="amcl estimation path")

    ax2.add_patch(tag_patch)
    ax2.add_patch(amcl_patch)
    
    with open(sim_path_file, "r") as f:
        reader = csv.reader(f)
        header = next(reader)
        first_values = next(reader)
        real_path_data.append((mpath.Path.MOVETO, (first_values[1], first_values[2])))
        tag_path_data_2.append((mpath.Path.MOVETO, (first_values[3], first_values[4])))
        amcl_path_data_2.append((mpath.Path.MOVETO, (first_values[5], first_values[6])))
        for row in reader:
            time_2.append(row[0])
            real_path_data.append((mpath.Path.CURVE4, (row[1], row[2])))
            tag_path_data_2.append((mpath.Path.CURVE4, (row[3], row[4])))
            amcl_path_data_2.append((mpath.Path.CURVE4, (row[5], row[6])))
    
    real_codes, real_verts = zip(*real_path_data)
    real_path = mpath.Path(real_verts, real_codes)
    real_patch = mpatches.PathPatch(real_path, edgecolor="red", facecolor="none", lw=3, label="Ground truth path", ls=":")

    tag_codes, tag_verts = zip(*tag_path_data_2)
    tag_path = mpath.Path(tag_verts, tag_codes)
    tag_patch = mpatches.PathPatch(tag_path, edgecolor="green", facecolor="none", lw=2, label="Proposed method path")

    amcl_codes, amcl_verts = zip(*amcl_path_data_2)
    amcl_path = mpath.Path(amcl_verts, amcl_codes)
    amcl_patch = mpatches.PathPatch(amcl_path, edgecolor="blue", facecolor="none", lw=2, label="AMCL path")
    

    ax1.add_patch(tag_patch)
    ax1.add_patch(amcl_patch)
    ax1.add_patch(real_patch)
    
    ax2.set_title("Real-world path estimates", fontsize=8)
    ax2.set_xlabel("x (m)")
    ax2.axes.yaxis.set_ticklabels([])

    
    
    ax1.set_title("Simulation path estimates and ground truth", fontsize=8)
    ax1.set_xlabel("x (m)")
    ax1.set_ylabel("y (m)")
    ax1.legend(prop={'size': 9})
    
    fig.suptitle("Simulation path versus real-world path", fontsize=11)
    plt.savefig("./straight_line_tests_real_vs_simulation.png", dpi=300)
    plt.show()
    

def create_plot_avg_error_median_mean_straight_test():
    median_test_location = "C:\\Users\\levij\\TU Delft\\BEP\\apriltag_localization\\straight_tests\\median\\straight_median_processed_buffersize_data_table.csv"
    mean_test_location = "C:\\Users\\levij\\TU Delft\\BEP\\apriltag_localization\\straight_tests\\mean\\straight_mean_processed_buffersize_data_table.csv"
    
    plot_dict_median = {buffer_size: [[], []] for buffer_size in [10, 20, 30]}
    with open(median_test_location, "r") as f:
        reader = csv.reader(f)
        header = next(reader)
        for row in reader:
            buffer_size, max_error, error, end_error = [float(row[i]) for i in range(len(row))]
            plot_dict_median[buffer_size][0].append(max_error)
            plot_dict_median[buffer_size][1].append(error)
    
    plot_dict_mean = {buffer_size: [[], []] for buffer_size in [10, 20, 30]}
    with open(mean_test_location, "r") as f:
        reader = csv.reader(f)
        header = next(reader)
        for row in reader:
            buffer_size, max_error, error, end_error = [float(row[i]) for i in range(len(row))]
            plot_dict_mean[buffer_size][0].append(max_error)
            plot_dict_mean[buffer_size][1].append(error)

    fig, (ax1, ax2) = plt.subplots(1, 2, sharex=True, sharey=True)
    
    colors = ["r","b","g","y","c","m","k"]
    for i, buffersize in enumerate(plot_dict_median):
        zipped_list = zip(plot_dict_median[buffersize][0], plot_dict_median[buffersize][1])
        sorted_list = sorted(zipped_list)
        print(sorted_list)
        tuples = zip(*sorted_list)
        x, y = [list(tuple) for tuple in tuples]
        ax1.plot(x, y, color=colors[i], label="Buffer size: {}".format(buffersize))
    
    for i, buffersize in enumerate(plot_dict_mean):
        zipped_list = zip(plot_dict_mean[buffersize][0], plot_dict_mean[buffersize][1])
        sorted_list = sorted(zipped_list, key=lambda x: x[0])
        tuples = zip(*sorted_list)
        x, y = [list(tuple) for tuple in tuples]
        ax2.plot(x, y, color=colors[i], label="Buffer size: {}".format(int(buffersize)))

    ax1.set_title("Median error", fontsize=10)
    ax1.set_xlabel("Maximum allowed error (m)")
    ax1.set_ylabel("Average path error (m)")
    
    ax2.set_title("Mean error", fontsize=10)
    ax2.set_xlabel("Maximum allowed error (m)")
    ax2.legend()
    
    fig.suptitle("Average path error vs maximum allowed error per buffer size\n(Straight-line test)", fontsize=12)
    plt.show()




if __name__ == '__main__':
    realpath_vs_simpath()