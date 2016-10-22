"""
This file describes motion primitives for the DVRK.
These are canned parameterized routines that you execute
during your control loop.
"""
import rospy, pickle, time, datetime
from robot import *
from geometry_msgs.msg import Pose
import numpy as np
import tfx
import sys


#helper methods first
def get_frame(pos, rot):
    """
    Gets a TFX pose from an input position/rotation for PSM1.
    """
    return tfx.pose(pos, rot)


def get_frame_ang(pos, nextpos, zoffset=0.003, angle=None):
    """
    Given two x,y,z coordinates, output a TFX pose that points the grippers to roughly the next position, at pos.
    """
    pos[2] -= zoffset
    rotation = [94.299363207+angle, -4.72728031036, 86.1958002688]
    rot = tfx.tb_angles(rotation[0], rotation[1], rotation[2])
    frame = tfx.pose(pos, rot)
    return frame

def get_frame_next(pos, nextpos, zoffset=0.003, angle=None):
    """
    Given two x,y,z coordinates, output a TFX pose that points the grippers to roughly the next position, at pos.
    """
    pos[2] -= zoffset
    rotation = [94.299363207+angle, -4.72728031036, 86.1958002688]
    rot = tfx.tb_angles(rotation[0], rotation[1], rotation[2])
    frame = tfx.pose(nextpos, rot)
    return frame


def get_angle(pos, nextpos):
    """
    Returns angle to nextpos in degrees
    """
    delta = nextpos - pos
    theta = np.arctan(delta[1]/delta[0]) * 180 / np.pi
    if delta[0] < 0:
        return theta + 180
    return theta






#primitives are below
"""
* Each of the primitives takes an arm as a first parameter
* Then takes in a set of optional parameters
* Each is also responsible for its own error checking
"""


def home_robot(arm,
               pos=[0.023580864372, 0.00699340564912, -0.0485527311586],
               rot=[0.617571885272, 0.59489495214, 0.472153066551, 0.204392867261]):
    """
    Homes the robot arm to a given position and rotation
    """

    if len(pos) != 3:
    	raise ValueError("Invalid Position to home_robot" + str(pos))

    if len(rot) != 4:
    	raise ValueError("Invalid Rotation to home_robot" + str(rot))

    arm.move_cartesian_frame(get_frame(pos, rot))





def cut(arm, closed_angle=1.0, open_angle=80.0, close_time=2.5, open_time=2.35):
    """
    Cutting motion
    """

    if closed_angle < 0:
    	raise ValueError("Invalid Closed Angle to cut" + str(closed_angle))

    if closed_angle > 85:
    	raise ValueError("Invalid Open Angle to cut" + str(open_angle))

    if close_time < 0.5 or open_time < 0.5:
    	raise ValueError("Not Enough time to close the grippers" + str(close_time) + "," + str(open_time))

    arm.open_gripper(closed_angle)
    time.sleep(close_time)
    arm.open_gripper(open_angle)
    time.sleep(open_time)


def movedc(arm, new_position=None, zoffset=0, speed=0.01):
	"""
	Moves the arm along a directionality contrained path such that the gripper is
	in the direction of movement
	"""
	if new_position == None:
		raise ValueError("New position cannot be null")

	if speed >= 0.06:
		raise ValueError("Speed is above what is safe")

	pos = arm.get_current_cartesian_position().position
	angle = get_angle(np.ravel(pos), np.ravel(new_position))

	frame = get_frame_ang(np.ravel(pos), np.ravel(new_position), zoffset=zoffset, angle = angle)
	arm.move_cartesian_frame_linear_interpolation(frame, speed)

	frame = get_frame_next(np.ravel(pos), np.ravel(new_position), zoffset=zoffset, angle = angle)
	arm.move_cartesian_frame_linear_interpolation(frame, speed)


def movep(arm, new_position=None, new_orientation=None, zoffset=0, speed=0.01):
	"""
	Moves the arm along along a linear path between the two points
	"""
	if new_position == None or new_orientation == None:
		raise ValueError("New position cannot be null")

	if speed >= 0.06:
		raise ValueError("Speed is above what is safe")

	frame = tfx.pose(new_position, new_orientation)
	arm.move_cartesian_frame_linear_interpolation(frame, speed)


def exec_primitive(primitive, arm, args_dict):
	"""
	Executes one of the primitives in this file on the described arm with the parameters
	"""
	start = datetime.datetime.now()
	print "[Primitive Execution] Started", primitive, "with args", args_dict
	primitive(arm, **args_dict)
	print "[Primitive Execution] Ended", primitive, "with args", args_dict, "after", datetime.datetime.now()-start
	return primitive, args_dict, datetime.datetime.now()-start, arm.get_current_cartesian_position()




def exec_primitive_list(primtive_list, arm, list_of_args, strict=True):
	"""
	Given a sequence of primitives and a list of args and executes them sequentially.

	Strict execution means that it will fail if any errors are encountered
	"""
	execution_time = 0
	execution_sucess = True

	for i,primitive in enumerate(primtive_list):

		try:
			p, a, t, s = exec_primitive(primitive, arm, list_of_args[i])
			execution_time = execution_time + t.seconds
		except:
			execution_sucess = False

			print "Unexpected error:", sys.exc_info()[0]

			if strict:
				raise ValueError("Failure At Primitive " + str(primitive))


	print "[Primitive Execution] Total Execution Time of the Primitives ", execution_time

	return execution_sucess, arm.get_current_cartesian_position()



