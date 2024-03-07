#!/usr/bin/python3
#from MpcControl import *
import numpy as np
import numpy.linalg as LA
import scipy.linalg as sLA
from pathlib import Path
import os
from dataclasses import dataclass
import yaml
import inspect
from rotor_tm_utils import utilslib
from rotor_tm_utils.RPYtoRot_ZXY import RPYtoRot_ZXY

# Class to store various parameters of the UAV and the controller
@dataclass
class uav_params_class:
  uav_name: str # The UAV's name
  mass: float # UAV mass 
  inertia: list # UAV inertia
  arm_length: float # motor arm length 
  max_angle: float #degree
  num_props: int # number of propellors
  max_rpm: int # maximum rpm of the rotor 
  min_rpm: int # minimum rpm of the rotor
  motor_coefficients: float # The motor coefficients gramf/rpm^2 
  mesh_path: str # UAV mesh file path for visualization 

@dataclass
class mechanism_params_class:
  mechanism_type: str # attach mechanism type 
  num_of_robots: int # number of robots 
  rho: list # attach place on the payload 
  robot_list: list # This list of robots 
  cable_length: list 
  yaw_list: list

@dataclass
class pl_params_class:
  mass: float # payload mass
  inertia: list # payload inertia
  mesh_path: str # payload mesh file path for visualization 

class read_params:
  def __init__(self,):
    print("Initilizing the read_params")

  def yaml_to_dict(self,path_to_yaml):
    # reads yaml file and creates a dictionary
    with open(path_to_yaml, 'r') as stream:
        try:
          parsed_yaml=yaml.safe_load(stream)
        except yaml.YAMLError as exc:
          print(exc)
    return parsed_yaml

  # easy way to initialize a dataclass through a dictionary. Useful for classes with many instances
  def dict_to_class(self, clss, clss_name, data): # here clss is the name of the Class e.g MpcQuadrotor

    return clss(
        **{
        key: (data[key] if (val.default == val.empty)&(key in data) else data.get(key, val.default))
        for key, val in inspect.signature(clss_name).parameters.items()
        }
    )

  def system_setup(self, payload_params_path = None,quad_params_path = None,mechanism_params_path = None,payload_control_params_path = None,uav_controller_params_path=None): 
    payload_control_gains = self.read_payload_control_gains(payload_control_params_path)
    mechanism_params = self.read_mechanism_params(mechanism_params_path)
    quad_params = []
    uav_type = []
    uav_in_team = []
    for robot_idx, robot_name in enumerate(mechanism_params.robot_list):
      quad_ = self.read_uav_params(quad_params_path+robot_name+".yaml")
      uav_control_gains = self.read_uav_control_gains(uav_controller_params_path+robot_name+"_control_gains.yaml")
      quad_.Kp = uav_control_gains.Kp
      quad_.Kd = uav_control_gains.Kd
      quad_.Kpe = uav_control_gains.Kpe
      quad_.Kde = uav_control_gains.Kde
      quad_.Kxi = uav_control_gains.Kxi
      quad_.Kw = uav_control_gains.Kw
      quad_params.append(quad_)

      uav_name = quad_.uav_name
      if uav_name in uav_type: 
        uav_type_idx = uav_type.index(uav_name)
        total_num_uav_of_current_type = len(uav_in_team[uav_type_idx])
        uav_name_with_id = uav_name + str(total_num_uav_of_current_type+1)
        uav_in_team[uav_type_idx].append(uav_name_with_id)
      else: 
        uav_type.append(uav_name)
        uav_name_with_id = uav_name + str(1)
        uav_in_team.append([uav_name_with_id])

    #quad_params = self.read_uav_params(quad_params_path+robot_name+".yaml")
    params = self.read_payload_params(payload_params_path)
    params.uav_in_team = sum(uav_in_team, [])

    params.nquad = mechanism_params.num_of_robots
    params.mechanism_type = mechanism_params.mechanism_type
    params.rho_vec_list = mechanism_params.rho_vec_list
    params.Kp = payload_control_gains.Kp
    params.Kd = payload_control_gains.Kd
    params.Kpe = payload_control_gains.Kpe
    params.Kde = payload_control_gains.Kde
    params.rho_vec_asym_mat = np.hstack([utilslib.vec2asym(mechanism_params.rho_vec_list[:,k]) for k in range(0,params.nquad)])
    identity_stack_mat = np.hstack([np.eye(3) for k in range(0,params.nquad)])

    ## Set up parameters
    if params.mechanism_type == 'Cable':
        params.id = "Cable"
        ## This section sets up the essential controller parameters for cable suspended payload
        ## store cable length in both payload and uav params
        #quad_params.l = mechanism_params.cable_length[0] 
        params.cable_length = mechanism_params.cable_length
        if params.payload_type == 'Rigid Body':
          # Geometric parameters matrix for cooperative geometric controller
          P = np.vstack((identity_stack_mat,params.rho_vec_asym_mat))
          params.pseudo_inv_P = LA.pinv(P) # np.matmul(P.T, LA.inv(np.matmul(P, P.T)))
          params.P = P
          for robot_idx, robot_name in enumerate(mechanism_params.robot_list):
            quad_params[robot_idx].l = mechanism_params.cable_length[robot_idx] 
    else:
        if params.mechanism_type == 'Rigid Link':
          params.id = "Rigid Link"
          params.cable_length = np.zeros((mechanism_params.num_of_robots, ), dtype=float)
          # This section sets up the essential controller parameters for payload with rigid links
          ## Physical properties of structure
          # Calculate the inertia and mass of the entire structure
          params.struct_I = params.I
          params.struct_mass = params.mass
          rho_c = np.zeros((1,3), dtype=float)
          total_quad_mass = 0.0
          for uav_idx in range(params.nquad):
              params.struct_mass = params.struct_mass + quad_params[uav_idx].mass
              rho_c = rho_c + quad_params[uav_idx].mass * mechanism_params.rho_vec_list[:,uav_idx].T
              total_quad_mass += quad_params[uav_idx].mass

          rho_c = rho_c / (total_quad_mass + params.mass)
          rho_c = rho_c.T
 
          params.struct_I = params.struct_I + params.mass * np.array([[rho_c[1, 0]**2,0,0],[0,rho_c[0, 0]**2,0],[0,0,rho_c[0, 0]**2+rho_c[1, 0]**2]])

          ## Calculate the geometric constraints of the structure
          A = np.zeros((4,0), dtype=float)
          for k in range(params.nquad):
              rho = mechanism_params.rho_vec_list[:, k] - rho_c.T
              rho = rho.T
              R = np.transpose(RPYtoRot_ZXY(0,0,mechanism_params.yaw_list[k]))
              params.struct_I = params.struct_I + R @ quad_params[k].I @ R.T + quad_params[k].mass * np.array([[rho[1,0]**2,0,0],[0,rho[0,0]**2,0],[0,0,rho[0,0]**2+rho[1,0]**2]])
              A = np.hstack((A, np.vstack((np.array([1,0,0,0]), np.hstack((np.array([[rho[1,0]],[rho[0,0]],[0.0]]), R))))))
          params.rho_load = -rho_c
          params.rho_robot = mechanism_params.rho_vec_list - rho_c
          params.A = A

          ## Distribution matrix
          W = np.zeros((0,0), dtype=float)
          for k in np.arange(1,params.nquad+1):
              W = sLA.block_diag(W, np.array([[1,0,0,0],[0,10,0,0],[0,0,10,0],[0,0,0,10]]))

          invW = LA.inv(W)
          params.thrust_moment_distribution_mat = invW @ np.transpose(A) @ LA.inv(A @ invW @ np.transpose(A))

    return params, quad_params

  def read_uav_control_gains(self, path = None):
      params_dict = self.yaml_to_dict(path)
      params = self.dict_to_class(pl_params_class, pl_params_class, params_dict)
      params.Kp = np.array([[params_dict["pos"]["x"], 0,  0],
                            [0, params_dict["pos"]["y"],  0],
                            [0, 0,  params_dict["pos"]["z"]]])
      params.Kd = np.array([[params_dict["vel"]["x"], 0,  0],
                            [0, params_dict["vel"]["y"],  0],
                            [0, 0,  params_dict["vel"]["z"]]])
      params.Kpe = np.array([[params_dict["rot"]["x"], 0,  0],
                            [0, params_dict["rot"]["y"],  0],
                            [0, 0,  params_dict["rot"]["z"]]])
      params.Kde = np.array([[params_dict["ang"]["x"], 0,  0],
                            [0, params_dict["ang"]["y"],  0],
                            [0, 0,  params_dict["ang"]["z"]]])
      params.Kxi = np.array([[params_dict["xi"]["x"], 0,  0],
                            [0, params_dict["xi"]["y"],  0],
                            [0, 0,  params_dict["xi"]["z"]]])
      params.Kw = np.array([[params_dict["omg"]["x"], 0,  0],
                            [0, params_dict["omg"]["y"],  0],
                            [0, 0,  params_dict["omg"]["z"]]])
      return params

  def read_payload_control_gains(self, path = None):
      params_dict = self.yaml_to_dict(path)
      params = self.dict_to_class(pl_params_class, pl_params_class, params_dict)
      params.Kp = np.array([[params_dict["pos"]["x"], 0,  0],
                              [0, params_dict["pos"]["y"],  0],
                              [0, 0,  params_dict["pos"]["z"]]])
      params.Kd = np.array([[params_dict["vel"]["x"], 0,  0],
                              [0, params_dict["vel"]["y"],  0],
                              [0, 0,  params_dict["vel"]["z"]]])
      params.Kpe = np.array([[params_dict["rot"]["x"], 0,  0],
                              [0, params_dict["rot"]["y"],  0],
                              [0, 0,  params_dict["rot"]["z"]]])
      params.Kde = np.array([[params_dict["ang"]["x"], 0,  0],
                              [0, params_dict["ang"]["y"],  0],
                              [0, 0,  params_dict["ang"]["z"]]])
      return params

  def read_payload_params(self, path = None,params = None): 
      params_dict = self.yaml_to_dict(path)
      params = self.dict_to_class(pl_params_class, pl_params_class, params_dict)
      
      if params.inertia is not None:  
          params.payload_type = 'Rigid Body'
          params.I = np.array([[params.inertia['Ixx'], params.inertia['Ixy'], params.inertia['Ixz']], 
                               [params.inertia['Iyx'], params.inertia['Iyy'], params.inertia['Iyz']],
                               [params.inertia['Izx'], params.inertia['Izy'], params.inertia['Izz']]])
          params.invI = LA.inv(params.I)
      else:
          params.payload_type = 'Point Mass'
          params.I = np.zeros((3,3))
          params.invI = np.zeros((3,3))

      params.grav = 9.81
      ## Sim Parameters
      params.sim_start = False

      return params

  def read_mechanism_params(self, path = None): 

      params_dict = self.yaml_to_dict(path)
      params = self.dict_to_class(mechanism_params_class, mechanism_params_class, params_dict)

      ## Specific Parameters for different mechanism
      if params.mechanism_type == 'Cable':
          params.cable_length = np.array(params.cable_length)
      else:
          if params.mechanism_type == 'Rigid Link':
              params.yaw_list = np.array(params.yaw_list)
          else:
              raise Exception('Invalid attach mechanism')
      
      ## Attach Position on the payload
      rho_vec_list = []
      for i in range(0,params.num_of_robots):
          rho_vec_list.append(np.array([params.rho[i]['x'],params.rho[i]['y'],params.rho[i]['z']]))

      params.rho_vec_list = np.array(rho_vec_list).T
      return params

  def read_uav_params(self,path):
    params_dict = self.yaml_to_dict(path)
    params = self.dict_to_class(uav_params_class, uav_params_class, params_dict)
    params.I = np.array([[params.inertia['Ixx'], params.inertia['Ixy'], params.inertia['Ixz']], 
                         [params.inertia['Iyx'], params.inertia['Iyy'], params.inertia['Iyz']],
                         [params.inertia['Izx'], params.inertia['Izy'], params.inertia['Izz']]])

    params.invI = LA.inv(params.I)
    params.grav = 9.81
    params.maxangle = params.max_angle * np.pi/180
    params.maxF = params.num_props * params.motor_coefficients * params.max_rpm**2 
    params.minF = params.num_props * params.motor_coefficients * params.min_rpm**2
    self.uav_params = params
    return params

if __name__ == '__main__':
  uav_params_path = 'config/uav_params/snapdragonfly.yaml'
  payload_params_path = 'config/load_params/fedex_box_payload.yaml'
  mechanism_params_path = 'config/attach_mechanism/6_robots_cable_mechanism.yaml'
  
  read_params_funcs = read_params()
  pl_params, quad_params = read_params_funcs.system_setup(payload_params_path,uav_params_path,mechanism_params_path)


    
