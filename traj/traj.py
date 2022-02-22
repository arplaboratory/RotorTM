#! /usr/bin/env python


from RotorTM.utils.Optimization.entire_path.generate_poly import generate_poly
from RotorTM.utils.Optimization.entire_path.generate_poly_coeff import generate_poly_coeff
from RotorTM.utils.Optimization.optimize_traj import optimize_traj
from RotorTM.utils.Optimization.entire_path.generate_polynomial_matrix import generate_polynomial_matrix
from numpy import sin
from numpy import cos
import numpy as np
from RotorTM.utils.Optimization.allocate_time import allocate_time


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

	def circle(self, t, init_pos = None, r = None, period = None, circle_duration = None):
		# CIRCLE trajectory generator for a circle

		if (np.all(init_pos!=None)) and (r != None) and (period != None) and (circle_duration != None):
			print('Generating Circular Trajectory ...')
			self.Radius = r
			offset_pos_temp = np.append(init_pos[0]-self.Radius, init_pos[1], axis=0)
			self.offset_pos = np.append(offset_pos_temp, np.array([0]))
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
			final_theta = 2 * self.ramp_dist + self.circle_dist
			x_pos = self.Radius * np.cos(final_theta)
			y_pos = self.Radius * np.sin(final_theta)
			stop_pos_temp = np.append(x_pos, y_pos, axis=0)
			stop_pos_temp = np.append(stop_pos_temp, self.start[2], axis=0)
			stop = np.add(self.offset_pos, stop_pos_temp)
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

				temp_pos = np.append(x_pos, y_pos, axis=0)
				temp_pos = np.append(temp_pos, self.start[2], axis=0)
				pos = np.add(self.offset_pos, temp_pos)
				self.last_pos = pos
				vel = np.append(x_vel, y_vel, axis=0)
				vel = np.append(vel, np.array([0.0]), axis=0)
				acc = np.append(x_acc, y_acc, axis=0)
				acc = np.append(acc, np.array([0.0]), axis=0)
				jrk = np.append(x_jrk, y_jrk, axis=0)
				jrk = np.append(jrk, np.array([0.0]), axis=0)
				yaw = 0
				yawdot = 0
			else:
				pos = self.last_pos
				vel = np.array([[0],[0],[0]])
				acc = np.array([[0],[0],[0]])
				jrk = np.array([[0],[0],[0]])
				yaw = 0
				yawdot = 0

			self.state_struct["pos_des"] = pos
			self.state_struct["vel_des"] = vel
			self.state_struct["acc_des"] = acc
			self.state_struct["jrk_des"] = jrk
			self.state_struct["qd_yaw_des"] = yaw
			self.state_struct["qd_yawdot_des"] = yawdot
			self.state_struct["quat_des"] = np.array([1,0,0,0])
			self.state_struct["omega_des"] = np.array([0,0,0])

	def line_quintic_traj(self, t, map = None, path = None):

		# map is a class 
		# path is a 2D array
		if (np.any(map!= None) ) and (np.any(path != None)):
			self.mapquad = map
			self.pathall = path
			pathqn = self.pathall # ####may need modification
			ttotal = 10

			xy_res = map.resolution[0]
			basicdata = map.basicdata
			rowbasicdata = basicdata.shape[0]
			if rowbasicdata >= 2: # might need to be changed to 1
				block = basicdata[1:rowbasicdata,:]
			else:
				block = np.array([])

			# finalpath = simplify_path(pathqn,block,mapquad)
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
					self.timesegment[i-1, 0] = distance[i-1,:]
					self.timesegment[i-1, 1] = 0 # change back to 1 when done
				else:
					self.timesegment[i-1, :] = np.sqrt(distance[i-1,:])*2
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

		else:
				lengthtime = self.timepoint.shape[0]
				length = lengthtime -1 
				self.state_struct["qd_yaw_des"] = 0
				self.state_struct["qd_yawdot_des"] = 0
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
				self.state_struct["pos_des"] = np.transpose(state[0,:])
				self.state_struct["vel_des"] = np.transpose(state[1,:])
				self.state_struct["acc_des"] = np.transpose(state[2,:])
				self.state_struct["jrk_des"] = np.array([[0],[0],[0]])

	# init optimization still needs work
	def min_snap_traj_generator(self, t_current, path = None, options = None):
		
		if (np.any(path!= None) ) and (np.any(options != None)):
			self.pathall = path
			self.finalpath = path
			self.traj_constant = options

			if (self.traj_constant.pt_num-1 > self.traj_constant.total_traj_num):
				self.traj_constant.pt_num = self.traj_constant.total_traj_num + 1

			self.traj_constant.traj_num = self.traj_constant.pt_num -1
			self.polynomial_coeff = generate_poly_coeff(self.traj_constant)
			
			# optimization
			# skipping options

			T_seg_c = allocate_time(path,self.traj_constant.max_vel,self.traj_constant.max_acc)
			self.coefficient, self.timelist = optimize_traj(path, self.traj_constant, T_seg_c, self.traj_constant.cor_constraint)

			# for testing only 
			self.coefficient = np.array([[0,	0,	0,	-0.0416965619698132,	-0.218855981262230,	-0.0208482939048973,	0.00424521399370728,	-0.0251901718012580,	0.00212268167645621,	0.00459027025679635,	0.0222167480760644,	0.00229515499570285],
										[0,	0,	0,	-0.0814617284400699,	-0.469566063947324,	-0.0407308912865352,	0.0275479920453524,	-0.0179498997335257,	0.0137742475703451,	0.0118057902230596,	0.0623377330609565,	0.00590295642097165],
										[0,	0,	0,	-0.0803408604327523,	-0.551727479722655,	-0.0401704607750187,	0.0949170196048935,	0.112624383038113,	0.0474589936469037,	0.00411933277946876,	0.0575517458974790,	0.00205975572807737],
										[-0.0249656167735726,	-0.426569039465686,	-0.0124828290725290,	0.106121451973093,	0.404135871202583,	0.0530607521375807,	0.0859282067852385,	0.177728245067686,	0.0429636891834999,	-0.0417837413463769,	-0.112778704823717,	-0.0208919058332887],
										[0.0242038574976291,	0.214990722292932,	0.0121019398308200,	-0.0222871727141851,	-0.0749990722400463,	-0.0111435916898159,	-0.0529474360850257,	-0.103304068719143,	-0.0264735800578703,	0.0157155526648616,	0.0335421691862252,	0.00785777974313101],
										[-0.00373462924007937,	-0.0272001398473725,	-0.00186731607510025,	0.00134432536799305,	0.00426206626558220,	0.000672163015238655,	0.0120293360085826,	0.0231969692411790,	0.00601464120309192,	-0.00241789734889025,	-0.00467170989942681,	-0.00120894836962065],
										[-2.48561205136277e-16,	-9.27996732877306e-16,	-1.24282002527195e-16,	4.33821379326722e-07,	1.62825901690269e-06,	2.16909907382701e-07,	-0.00158020559967943,	-0.00303823484295296,	-0.000790099322363965,	0.000213830650773867,	0.000391157592120971,	0.000106915219643745],
										[-1.35632755842742e-15,	-5.07528217681406e-15,	-6.78171395489735e-16,	-6.58187453788788e-08,	-2.47037798415863e-07,	-3.29092510985259e-08,	0.000139951790592016,	0.000269070641338854,	6.99755790528368e-05,	-1.26268454291684e-05,	-2.20318910770929e-05,	-6.31341100126710e-06],
										[-1.26582141131039e-15,	-4.75862346128331e-15	,-6.32917809185207e-16,	6.89704835974672e-09,	2.58867966276567e-08,	3.44851113793549e-09,	-8.63682647884204e-06,	-1.66043686290165e-05,	-4.31839321718896e-06,	5.18822850783462e-07,	8.55770929089490e-07,	2.59410691959851e-07],
										[4.50722490689024e-16,	1.69328680108062e-15,	2.25363774930036e-16,	-4.75568765410979e-10,	-1.78496436433108e-09,	-2.37783463180274e-10,	3.53727601156553e-07,	6.80013783190245e-07,	1.76862960062070e-07,	-1.41222589051678e-08,	-2.17631026419158e-08,	-7.06110173966658e-09],
										[-5.76929731399120e-17,	-2.16702982455500e-16,	-2.88468103656573e-17,	1.94285790156342e-11,	7.29219709326689e-11,	9.71425114127762e-12,	-8.65345808413739e-09,	-1.66348693650874e-08,	-4.32670798611338e-09,	2.29094164166653e-10,	3.24641345793079e-10,	1.14546489579265e-10],
										[2.73089088765438e-18,	1.02567335313399e-17,	1.36546077070260e-18,	-3.56454792477659e-13,	-1.33789766014606e-12,	-1.78226678213842e-13,	9.58016947914100e-11,	1.84155298279425e-10,	4.79006089179353e-11,	-1.67709361404298e-12,	-2.13689385905371e-12,	-8.38541270003930e-13]])

		else:
			for i in range(1, self.traj_constant.total_traj_num+1):

				if (self.traj_constant.pt_num == 2) or (i%(self.traj_constant.pt_num-1) == 1):
					t_start = self.timelist[i-1,0]
				
				if (t_current >= self.timelist[i-1,0]) and (t_current < self.timelist[i,0]):
					
					time_term = t_current - t_start
					time_matrix = generate_polynomial_matrix(self.traj_constant.max_exponent,3,time_term)
					state = np.matmul(np.multiply(self.polynomial_coeff[0:4,:], time_matrix), self.coefficient[:,(i-1)*self.traj_constant.dim:i*self.traj_constant.dim])
					
				elif t_current >= self.timelist[self.traj_constant.total_traj_num-1]:
					
					state = np.zeros((4, 3), dtype=float)
					state[0,:] = self.finalpath[self.traj_constant.total_traj_num:self.traj_constant.total_traj_num+1,:]
					state[1,:] = np.array([[0,0,0]])
					state[2,:] = np.array([[0,0,0]])
					state[3,:] = np.array([[0,0,0]])

			self.state_struct["pos_des"] = np.transpose(state[0,:])
			self.state_struct["vel_des"] = np.transpose(state[1,:])
			self.state_struct["acc_des"] = np.transpose(state[2,:])
			self.state_struct["jrk_des"] = np.transpose(state[3,:])
			self.state_struct["qd_yaw_des"] = np.array([[0]])
			self.state_struct["qd_yawdot_des"] = np.array([[0]])
			self.state_struct["quat_des"] = np.array([[1,0,0,0]])
			self.state_struct["omega_des"] = np.array([[0,0,0]])


