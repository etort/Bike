"""
Simulate bicycle with elevation and gear ratio
"""
import numpy as np
import time
import sys
sys.path.append('..')
from XboxController import XboxController as XBC

class Bike(object):
    def __init__(self, M, R_wheel, R_pedal, R_gear_back, R_gear_front, C_friction)
        self.M     = M  # mass of bike [kg]
        self.Rw    = R_wheel  # wheel radius [m]
        self.Rp    = R_pedal  # length of pedal [m]
        self.Rgb   = np.sort(np.array(R_gear_back))[::-1]  # array of gear radii on back tire [m]
        self.Rgf   = np.sort(np.array(R_gear_front))  # array of gear radii on pedal [m]
        self.Cf    = C_friction  # coefficient of friction [kg/s]
        self.Ms    = M*0.03  # mass of spokes
        self.Mt    = M*0.1  # mass of tires
        self.Mp    = M*0.04  # mass of pedals
        self.D     = 2700  # density of Al [kg/m^3]
        self.Mgb   = np.pi*self.Rgb**2*0.006*self.D  # mass of gears [kg]
        self.Mgf   = np.pi*self.Rgf**2*0.006*self.D  # mass of gears [kg]
        self.g     = 9.81  # [m/s^2]
        self.theta = 0  # elevation
        self.a     = 0  # accel
        self.v     = 0  # velocity [m/s]
        self.x     = 0  # distance
        self.Iw    = self.Mt*self.Rw**2 + 3*self.Ms*self.Rw**2/3 + 1./2*sum(self.Mgb*self.Rgb**2)
        self.Ig    = self.Mp*self.Rp**2/3 + 0.5*sum(self.Mgf*self.Rgf**2)
        self.gb    = 0
        self.gf    = 0
        self.T     = [0]
        self.T0    = time.time()
        self.A     = [0]  # acceleration [m/s^2]
        self.V     = [0]  # velocity
        self.X     = [0]  # distance traveled [m/s^2]

    def update(self,Fp):
        self.T = self.T.append(self.t)
        self.a = (self.Rp*self.Rgb[self.gb]*Fp - self.M*self.Rw**2*self.Rgf[self.gf]*(self.g*np.sin(self.theta) +
                                                                                      self.Cf*self.v/self.M))/\
                 (self.Rgb[self.gb]*self.Ip + self.Rgf[self.gf]*self.Iw+self.M*self.Rw**2*self.Rgf[self.gf])
        self.A = self.A.append(self.a)
        self.v = sum(self.A[1:]*(self.t[1:]-self.t[:-1]))
        self.V = self.V.append(self.v)
        self.x = sum(self.V[1:]*(self.t[1:]-self.t[:-1]))
        self.X = self.X.append(self.x)

    def printval(self):
        print 'current acceleration = %f' %self.a
        print 'current speed = %f' %self.v
        print 'distance traveled = %f' %self.x

    def shift_front_up(self):
        if not self.Rgf[self.gf] == self.Rgf[-1]:
            self.gb = self.gb + 1
        else:
            print 'Already in highest gear'

    def shift_front_dn(self):
        if not self.Rgf[self.gf] == self.Rgf[0]:
            self.gb = self.gb - 1
        else:
            print 'Already in lowest gear'

