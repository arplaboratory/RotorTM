#!/usr/bin/python3
#from MpcControl import *
import sys
import numpy as np
from pathlib import Path
from rotor_tm_utils import read_params
import simulation_base


if __name__ == '__main__':
  payload_params_path = sys.argv[1]
  uav_params_path = sys.argv[2]
  mechanism_params_path = sys.argv[3]
  payload_control_gain_path = sys.argv[4]
  uav_control_gain_path = sys.argv[5]

  read_params_funcs = read_params.read_params()
  pl_params, quad_params = read_params_funcs.system_setup(payload_params_path,uav_params_path,mechanism_params_path,payload_control_gain_path, uav_control_gain_path)
  
  rotortm_simulation_base = simulation_base.simulation_base(pl_params,quad_params)
    
  
   
    
