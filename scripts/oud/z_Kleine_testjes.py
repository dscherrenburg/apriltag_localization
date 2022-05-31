import os
import shutil

datafolders = os.listdir("/home/daan/localization_ws/src/apriltag_localization/tests/")
os.makedirs("/home/daan/localization_ws/src/apriltag_localization/tests/data/")
for folder in datafolders:
    shutil.copy("/home/daan/localization_ws/src/apriltag_localization/tests/" + folder + "/" + folder + ".csv", "/home/daan/localization_ws/src/apriltag_localization/tests/data/")


