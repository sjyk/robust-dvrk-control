"""
This class generates a cutting plan
Given a set of way points it generates a smoothed plan.
The plan is a list of primitives.
"""
from scipy.interpolate import interp1d
from scipy.signal import savgol_filter
from primitives.motion_primitives import *

class CutPlanner(object):

	def __init__(self, interpolation_factor=4):
		self.interpolation_factor = interpolation_factor

	def interpolation(self, arr, factor):
	    """
	    Given a matrix of x,y,z coordinates, output a linearly interpolated matrix of coordinates with factor * arr.shape[1] points.
	    """
	    x = arr[:, 0]
	    y = arr[:, 1]
	    z = arr[:, 2]
	    t = np.linspace(0,x.shape[0],num=x.shape[0])
	    to_expand = [x, y, z]
	    for i in range(len(to_expand)):
	        spl = interp1d(t, np.ravel(to_expand[i]))
	        to_expand[i] = spl(np.linspace(0,len(t), len(t)*factor))
	    new_matrix = np.matrix(np.r_[0:len(t):1.0/factor])
	    for i in to_expand:
	        new_matrix = np.concatenate((new_matrix, np.matrix(i)), axis = 0)
	    return new_matrix.T[:,1:]


	def loadWaypoints(self, fname="gauze_pts.p"):
		"""
		Loads from a stored pickle file of way points,
		fits a curvature constrained path
		"""

		lst = []
		f3 = open(fname, "rb")
		while True:
			try:
				pos2 = pickle.load(f3)
				lst.append(pos2)
			except EOFError:
				f3.close()
				break

		pts = np.matrix(lst)
		pts = self.interpolation(pts, self.interpolation_factor)
		self.waypoints = pts


	def generatePlan(self, arm, airCut=True):
		"""
		Generates a set of primitives that can be executed
		"""
		plan = []
		args = []

		init_position = arm.get_current_cartesian_position().position
		init_rot = arm.get_current_cartesian_position().rotation

		zoffset = 0

		if not airCut:
			zoffset = -0.01

		for i in range(self.waypoints.shape[0]-1):

			if i == 0:
				plan.append(movep)
				args.append({'new_position': self.waypoints[i,:], 'new_orientation': init_rot})
			else:
				plan.append(movedc)
				args.append({'new_position': self.waypoints[i,:], 'zoffset': zoffset})
			
			plan.append(cut)
			args.append({})

		return plan, args

