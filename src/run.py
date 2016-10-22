import rospy, pickle, time
from geometry_msgs.msg import Pose
import numpy as np
import tfx

from robot import *
from primitives.motion_primitives import *
from planner.cut_planner import CutPlanner



if __name__ == '__main__':

    psm1 = robot("PSM1")
    psm2 = robot("PSM2")

    c = CutPlanner()
    c.loadWaypoints()
    plan = c.generatePlan(psm1, airCut=False)
    exec_primitive_list(plan[0], psm1, plan[1])

    #print psm1.get_current_cartesian_position().position
    #grab_gauze(psm2)
    #movedc(psm1, new_position=[0.0502955383167, 0.0129230873448, -0.0612554903039])
    #exec_primitive_list([home_robot, movedc], psm1, [{}, {'new_position': [0.0502955383167, 0.0129230873448, -0.0612554903039]}])

    #exec_primitive_list([home_robot, cut], psm1, [{},{}])

    
