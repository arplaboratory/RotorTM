#! /usr/bin/env python

from importlib.resources import path
import numpy as np


class options:
    def __init__(self):
        self.total_traj_num = None
        self.traj_num = None
        self.pt_num = None
        self.dim = None
        self.num_coeff = None
        self.pt_num = None
        self.max_iteration = None
        self.max_vel = None
        self.max_acc = None
        self.cor_constraint = None
        self.max_exponent = None
        self.max_diff = None
        self.cor_wid = None
        self.nc = None

    def create_default_option(self, pathlength=5):
        self.max_diff = 4
        self.max_exponent = 6
        self.max_vel = 2.0  # 3.0
        self.max_acc = 5.0  # 10.0
        self.dim = 3
        self.cor_wid = 0.2
        self.nc = 10
        self.cor_constraint = False
        self.max_iteration = 300
        self.num_coeff = self.max_exponent + 1
        self.total_traj_num = pathlength - 1
        self.pt_num = pathlength
        self.traj_num = pathlength - 1
