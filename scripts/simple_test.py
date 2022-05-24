from combine_map_and_detections import VisualLocalization
import rospy


class SimpleTest:
    def __init__(self):
        pass


if __name__ == '__main__':
    rospy.init_node('simple_test')
    localizer = VisualLocalization()
    test = SimpleTest()
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        localizer.run_localization()
        test.run_test()
        rate.sleep()