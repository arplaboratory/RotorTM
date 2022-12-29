#! /usr/bin/env python3
import numpy as np
from numpy import sin
from numpy import cos
from Optimization.entire_path.generate_poly import generate_poly
from Optimization.entire_path.generate_poly_coeff import generate_poly_coeff
from Optimization.optimize_traj import optimize_traj
from Optimization.entire_path.generate_polynomial_matrix import generate_polynomial_matrix
from Optimization.allocate_time import allocate_time

class traj:
	def __init__(self):
		# for circles
		self.state_struct = {}
		self.Radius = None
		self.ramp_theta_coeff = None
		self.zcoeff = None
		self.tf = None
		self.last_pos = None
		self.offset_pos = None
		self.T = None 
		self.omega_des = None
		self.ramp_t = None
		self.ramp_dist = None
		self.circle_dist = None
		self.start = None
		self.duration = None

		# for line_quintic_traj_generator
		self.mapquad = None
		self.pathall = None
		self.coefficient = None
		self.finalpath = None
		self.timepoint = None
		self.timesegment = None

		# for min_snap_traj_generator
		self.snap_coeff = None
		self.snap_finalpath = None
		self.timelist = None
		self.polynomial_coeff = None
		self.traj_constant = None

		# check if the current traj has finished
		self.finished = False
		self.traj_type = 0
		# 0 is flag for traj is not initialized
		# 1 is circle
		# 2 is line
		# 3 is min_snap

	def circle(self, t, init_pos = None, r = None, period = None, circle_duration = None):
		# CIRCLE trajectory generator for a circle
		
		if (np.all(init_pos!=None)) and (r != None) and (period != None) and (circle_duration != None):
			print('Generating Circular Trajectory ...')

			self.finished = False
			self.Radius = r
			self.offset_pos = np.array([init_pos[0]-self.Radius, init_pos[1], init_pos[2]])
			self.T = period
			self.omega_des = 2*np.pi/self.T
			self.alpha_des = np.pi/40
			self.start = init_pos
			self.ramp_t = self.omega_des/self.alpha_des
			self.duration = circle_duration

			thetainitial = np.array([[0],[0],[0],[self.ramp_t*self.omega_des],[0],[0]])
			A = np.append(generate_poly(5,2,0), generate_poly(5,2,1), axis = 0)

			self.ramp_theta_coeff = np.matmul(np.linalg.inv(A), thetainitial)
			self.ramp_dist = sum(np.multiply(self.ramp_theta_coeff, np.array([[1],[1/2],[1/3],[1/4],[1/5],[1/6]])))
			self.circle_dist = self.omega_des*self.duration
			self.tf = self.ramp_t * 2 + self.duration
			self.traj_type = 1
		else:
			if t < self.tf:
				if t<=self.ramp_t:  # ramping up the circle
					dt = t/self.ramp_t
					integral_poly = generate_poly(6,0,dt)
					integral_poly = np.multiply(integral_poly[:,1:7],[1,1/2,1/3,1/4,1/5,1/6])
					polynominalmat = np.append(integral_poly, generate_poly(5,2,dt), axis=0)
					theta_d = np.matmul(polynominalmat, self.ramp_theta_coeff)
					theta_d = np.multiply(theta_d, np.array([[1],[1/self.ramp_t],[1/self.ramp_t**2],[1/self.ramp_t**3]]))
				else:
					if t<=(self.ramp_t + self.duration): # constant velocity cruising
						dt = t - self.ramp_t
						theta_d = np.zeros((4,1),dtype=float)
						theta_d[0] = self.omega_des * dt + self.ramp_dist
						theta_d[1] = self.omega_des

					else:  # ramping down the circle
						dt = 1 - (t - self.duration - self.ramp_t)/self.ramp_t
						integral_poly = generate_poly(6,0,dt)
						integral_poly = np.multiply(integral_poly[:,1:7],[1,1/2,1/3,1/4,1/5,1/6])
						polynominalmat = np.append(integral_poly, generate_poly(5,2,dt), axis=0)

						theta_d = np.matmul(polynominalmat, self.ramp_theta_coeff)
						theta_d = np.multiply(theta_d, np.array([[1],[1/self.ramp_t],[1/self.ramp_t**2],[1/self.ramp_t**3]]))
						theta_d[0] = self.circle_dist + 2*self.ramp_dist - theta_d[0]

				x_pos = self.Radius * cos(theta_d[0])
				y_pos = self.Radius * sin(theta_d[0])
				x_vel = -self.Radius * sin(theta_d[0]) * theta_d[1]
				y_vel =  self.Radius * cos(theta_d[0]) * theta_d[1]
				x_acc = -self.Radius * cos(theta_d[0]) * theta_d[1]**2 - self.Radius * sin(theta_d[0]) * theta_d[2]
				y_acc = -self.Radius * sin(theta_d[0]) * theta_d[1]**2 + self.Radius * cos(theta_d[0]) * theta_d[2]
				x_jrk = self.Radius * sin(theta_d[0]) * theta_d[1]**3 - 3 * self.Radius * cos(theta_d[0]) * theta_d[1] * theta_d[2] - self.Radius * sin(theta_d[0]) * theta_d[3]
				y_jrk = -self.Radius * cos(theta_d[0]) * theta_d[1]**3 - 3 * self.Radius * sin(theta_d[0]) * theta_d[1] * theta_d[2] + self.Radius * cos(theta_d[0]) * theta_d[3]

				pos = self.offset_pos + np.array([x_pos[0], y_pos[0], 0]) 
				self.last_pos = pos 
				vel = np.array([x_vel[0], y_vel[0], 0.0])
				acc = np.array([x_acc[0], y_acc[0], 0.0])
				jrk = np.array([x_jrk[0], y_jrk[0], 0.0])
			else:
				pos = self.last_pos
				vel = np.array([[0],[0],[0]])
				acc = np.array([[0],[0],[0]])
				jrk = np.array([[0],[0],[0]])
				self.finished = True

			self.state_struct["pos_des"] = pos
			self.state_struct["vel_des"] = vel
			self.state_struct["acc_des"] = acc
			self.state_struct["jrk_des"] = jrk
			self.state_struct["quat_des"] = np.array([1,0,0,0])
			self.state_struct["omega_des"] = np.array([0,0,0])

	def line_quintic_traj(self, t, map = None, path = None):

		# map is a class 
		# path is a 2D array
		if (np.any(map!= None) ) and (np.any(path != None)):
			print("Generating quintic trajectory")

			self.finished = False
			self.mapquad = map
			self.pathall = path
			pathqn = self.pathall
			ttotal = 10

			xy_res = map.resolution[0]
			basicdata = map.basicdata
			rowbasicdata = basicdata.shape[0]
			if rowbasicdata >= 2: 
				block = basicdata[1:rowbasicdata,:]
			else:
				block = np.array([])

			# use pathqn as the final path
			self.finalpath = pathqn

			pathlength = self.finalpath.shape[0]
			m = pathlength - 1

			distance = np.zeros((self.finalpath.shape[0],1))
			self.timesegment = np.zeros((self.finalpath.shape[0], 2))

			for i in range(1, m+1):
				previous = self.finalpath[i-1, :]
				afterward = self.finalpath[i, :]

				distance[i-1, :] = np.linalg.norm(afterward-previous)
				if distance[i-1,:]<=1:
					self.timesegment[i-1, 0] = distance[i-1,:]*5
					self.timesegment[i-1, 1] = 0
				else:
					self.timesegment[i-1, :] = np.sqrt(distance[i-1,:])*10
					self.timesegment[i-1, 1] = 0
			
			time_temp = 0
			self.timepoint = np.zeros((m, 1))
			for i in range(1, m+1):
				time_temp = time_temp + self.timesegment[i-1, 0]
				self.timepoint[i-1, 0] = time_temp
			self.timepoint = np.append(np.array([[0]]), self.timepoint, axis = 0)

			constraints = np.zeros((6*m, 6), dtype=float)
			condition = np.zeros((6*m, 3), dtype=float)
			self.coefficient = np.zeros((6*m, 3), dtype=float)
			for j in range(1, m+1):
				tstart = 0
				tend = self.timesegment[j-1, 0]

				constraints[6*j-6,:] = np.array([1, tstart, tstart**2, tstart**3  ,   tstart**4   ,  tstart**5])
				constraints[6*j-5,:] = np.array([0, 1     , 2*tstart, 3*tstart**2,   4*tstart**3 ,  5*tstart**4])
				constraints[6*j-4,:] = np.array([0, 0     , 2       , 6*tstart  ,   12*tstart**2,  20*tstart**3])
				constraints[6*j-3,:] = np.array([1, tend  , tend**2  , tend**3    ,   tend**4     ,  tend**5     ])
				constraints[6*j-2,:] = np.array([0, 1     , 2*tend  , 3*tend**2  ,   4*tend**3   ,  5*tend**4   ])
				constraints[6*j-1,:] = np.array([0, 0     , 2       , 6*tend    ,   12*tend**2  ,  20*tend**3  ])
				condition  [6*j-6,:] = self.finalpath[j-1,:]
				condition  [6*j-3,:] = self.finalpath[j,:]
				inverse = np.linalg.inv(constraints[6*j-6:6*j,0:6])
				coefficient_temp = np.matmul(inverse,condition[6*j-6:6*j,0:3])
				self.coefficient[6*j-6:6*j,0:3] = coefficient_temp
			self.traj_type = 2
		else:
				lengthtime = self.timepoint.shape[0]
				length = lengthtime -1 
				state = np.zeros((3, 3), dtype=float)
				for i in range(1, length+1):
					if (t >= self.timepoint[i-1][0]) and (t < self.timepoint[i][0]) and (self.timesegment[i-1, 1] == 0):
						currenttstart = self.timepoint[i-1][0]
						state = np.array([[1, (t-currenttstart), (t-currenttstart)**2, (t-currenttstart)**3, (t-currenttstart)**4, (t-currenttstart)**5], [0, 1, 2*(t-currenttstart), 3*(t-currenttstart)**2, 4*(t-currenttstart)**3, 5*(t-currenttstart)**4], [0, 0, 2, 6*(t-currenttstart), 12*(t-currenttstart)**2, 20*(t-currenttstart)**3]]) 
						state = np.matmul(state, self.coefficient[6*i-6:6*i,0:3])
					elif (t >= self.timepoint[i-1]) and (t < self.timepoint[i]) and (self.timesegment[i-1, 1] == 1):
						state[0, :] = self.finalpath[i,:]
						state[1, :] = np.array([0,0,0])
						state[2, :] = np.array([0,0,0])
					elif (t >= self.timepoint[lengthtime-1]):
						state[0, :] = self.finalpath[lengthtime - 1, :]
						state[1, :] = np.array([0,0,0])
						state[2, :] = np.array([0,0,0])
						self.finished = True
				self.state_struct["pos_des"] = np.transpose(state[0,:])
				self.state_struct["vel_des"] = np.transpose(state[1,:])
				self.state_struct["acc_des"] = np.transpose(state[2,:])
				self.state_struct["jrk_des"] = np.array([[0],[0],[0]])
				self.state_struct["quat_des"] = np.array([1,0,0,0])
				self.state_struct["omega_des"] = np.array([0,0,0])

	def min_snap_traj_generator(self, t_current, path = None, options = None):
		
		if (np.any(path!= None) ) and (np.any(options != None)):
			print("The path is ", path)

			self.finished = False
			self.pathall = path
			self.finalpath = path
			self.traj_constant = options

			if (self.traj_constant.pt_num-1 > self.traj_constant.total_traj_num):
				self.traj_constant.pt_num = self.traj_constant.total_traj_num + 1

			self.traj_constant.traj_num = self.traj_constant.pt_num -1
			self.polynomial_coeff = generate_poly_coeff(self.traj_constant)
			
			# optimization
			T_seg_c = allocate_time(path,self.traj_constant.max_vel,self.traj_constant.max_acc)
			self.coefficient, self.timelist = optimize_traj(path, self.traj_constant, T_seg_c, self.traj_constant.cor_constraint)
			print(self.timelist)
			print("The total traj num is ", self.traj_constant.total_traj_num)
			self.traj_type = 3

		else:
			for i in range(self.traj_constant.total_traj_num):
				if (self.traj_constant.pt_num == 2) or ((i+1)%(self.traj_constant.pt_num-1) == 1):
					t_start = self.timelist[i,0]
				
				if (t_current >= self.timelist[i,0]) and (t_current < self.timelist[i+1,0]):
					
					time_term = t_current - t_start
					time_matrix = generate_polynomial_matrix(self.traj_constant.max_exponent,3,time_term)
					state = np.matmul(np.multiply(self.polynomial_coeff[0:4,:], time_matrix), self.coefficient[:,i*self.traj_constant.dim:(i+1)*self.traj_constant.dim])
					
				elif t_current >= self.timelist[self.traj_constant.total_traj_num]:
					
					state = np.zeros((4, 3), dtype=float)
					state[0,:] = self.finalpath[self.traj_constant.total_traj_num:self.traj_constant.total_traj_num+1,:]
					state[1,:] = np.array([[0,0,0]])
					state[2,:] = np.array([[0,0,0]])
					state[3,:] = np.array([[0,0,0]])
					self.finished = True

			self.state_struct["pos_des"] = np.transpose(state[0,:])
			self.state_struct["vel_des"] = np.transpose(state[1,:])
			self.state_struct["acc_des"] = np.transpose(state[2,:])
			self.state_struct["jrk_des"] = np.transpose(state[3,:])
			self.state_struct["quat_des"] = np.array([1,0,0,0])
			self.state_struct["omega_des"] = np.array([0,0,0])


