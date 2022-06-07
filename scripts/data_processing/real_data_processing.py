import csv
import os
import math as m

def return_csv_lists(csv_file):
    """returns a list of lists from a csv file in the format of: [[time], [tag_x], [tag_y], [amcl_x], [amcl_y]]"""
    data = [[], [], [], [], []]
    with open(csv_file, 'r') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        header = next(reader)
        for row in reader:
            for i in range(len(row)):
                data[i].append(float(row[i]))
    
    # format [[time], [tag_x], [tag_y], [amcl_x], [amcl_y]]
    return data


if __name__ == "__main__":
    tests = {"straight": "/home/levijn/BEP/simulation_ws/src/apriltag_localization/real_tests_straight/median",
             "turn": "/home/levijn/BEP/simulation_ws/src/apriltag_localization/real_tests_turn/median",
             "slip": "/home/levijn/BEP/simulation_ws/src/apriltag_localization/real_tests_turn_slip/median"}
    processed_data_location = "/home/levijn/BEP/simulation_ws/src/apriltag_localization"
    processed_data_filename = "real_tests_data_processed.csv"
    
    with open(os.path.join(processed_data_location, processed_data_filename), 'w') as csvfile:
        writer = csv.writer(csvfile, delimiter=',')
        header = ["test_name", "tag_end_error_mean", "amcl_end_error_mean"]
        writer.writerow(header)
        
        for test in tests:
            test_location = tests[test]
            data_location = test_location + "/data"
            measurements_location = test_location + "/measurements.csv"
            
            # read measurements csv
            measurements_dict = {}
            with open(measurements_location, 'r') as csvfile:
                reader = csv.reader(csvfile, delimiter=',')
                header = next(reader)
                for row in reader:
                    measurements_dict[row[0]] = (float(row[1])/100, float(row[2])/100)
            
            # read test data
            test_dict = {}
            data_files = os.listdir(data_location)
            for file in data_files:
                filename = file.split('.')[0]
                time, tag_x, tag_y, amcl_x, amcl_y = return_csv_lists(os.path.join(data_location, file))
                test_dict[filename] = {"tag": (tag_x, tag_y),
                                    "amcl": (amcl_x, amcl_y)}
            
            # calculate endpoint errors
            end_errors_tag = []
            end_errors_amcl = []
            
            for test_name in measurements_dict:
                measurements = measurements_dict[test_name]
                test_data_tag = test_dict[test_name]["tag"]
                test_data_amcl = test_dict[test_name]["amcl"]
                
                last_position_tag = (test_data_tag[0][-1], test_data_tag[1][-1])
                last_position_amcl = (test_data_amcl[0][-1], test_data_amcl[1][-1])
                
                end_error_tag = m.sqrt((measurements[0] - last_position_tag[0])**2 + (measurements[1] - last_position_tag[1])**2)
                end_error_amcl = m.sqrt((measurements[0] - last_position_amcl[0])**2 + (measurements[1] - last_position_amcl[1])**2)
                
                end_errors_tag.append(end_error_tag)
                end_errors_amcl.append(end_error_amcl)
                
            end_error_tag_mean = sum(end_errors_tag)/len(end_errors_tag)
            end_error_amcl_mean = sum(end_errors_amcl)/len(end_errors_amcl)
            
            writer.writerow([test, end_error_tag_mean, end_error_amcl_mean])
            
            