import os
import rospy
from real_world_test import RealMoveTest
from simulation_turn_test import MoveAndTurn
from simulation_straight_test import Move


if __name__ == '__main__':
    rospy.init_node('real_world_test')
    
    save_location = rospy.get_param("~save_location")
    save_name = rospy.get_param("~save_name")
    save_format = rospy.get_param("~save_format")
    test_type = rospy.get_param("~test_type")
    
    rospy.loginfo(save_location + "/" + save_name + save_format)
    
    rospy.sleep(3)
    
    try: 
        os.makedirs(save_location)
    except OSError as fail: 
        pass
    
    if test_type == "straight":
        moves = [Move(4, 0.25)]
        test = RealMoveTest(save_location, save_name, save_format, moves=moves)
        test.run()
    elif test_type == "turn":
        moves = [MoveAndTurn(4, 0.25, 0.5)]
        test = RealMoveTest(save_location, save_name, save_format, moves=moves)
    else:
        rospy.loginfo("Test type not recognized")
        