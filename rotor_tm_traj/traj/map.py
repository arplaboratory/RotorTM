#! /usr/bin/env python

import numpy as np


class map:
    def __init__(self):
        self.nodenumber = None
        self.blockflag = None
        self.margin = None
        self.segment = None
        self.boundary = None
        self.block = None
        self.resolution = None
        self.basicdata = None

    def load_map(self, MapNumber=0):
        if MapNumber == 0:
            self.blockflag = np.zeros((4001, 1))
            self.nodenumber = self.blockflag
            for i in range(4000):
                self.nodenumber[i] = i + 1
            self.margin = 1
            self.segment = np.array([[20, 20, 10]])
            self.boundary = np.array([[-10, -10, 0, 10, 10]])
            self.block = np.array([[]])
            self.resolution = np.array([[1, 1, 1]])
            self.basicdata = np.array([[-10, -10, 0, 10, 10]])

        # add more map as necessary
        '''elif MapNumber == 1:
			self.blockflag = np.zeros((4001, 1))
			self.nodenumber = self.blockflag
			for i in range(4000):
				self.nodenumber[i] = i+1
			self.margin = 1
			self.segment = np.array([20,20,10])
			self.boundary = np.array([-10, -10, 0, 10, 10])
			self.block = np.array([])
			self.resolution = np.array([1, 1, 1])
			self.basicdata = np.array([-10, -10, 0, 10, 10])'''
