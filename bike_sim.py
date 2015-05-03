"""
Simulate bicycle with elevation and gear ratio
"""
import numpy as np
import time
import sys
import threading
#import multiprocessing
from XboxController import XboxController

class Bike(threading.Thread):

    def __init__(self,
                 Fm=800,
                 M=10,
                 R_wheel=0.3,
                 R_pedal=0.2,
                 R_gear_back=np.linspace(0.0254,0.0508,9),
                 R_gear_front=np.linspace(0.0635,0.1016,3),
                 C_friction=0.001):
        #setup threading
        threading.Thread.__init__(self)

        #persist values
        self.running = False

        # physical parameters
        self.Fm    = Fm  # max pedalling force [N]
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
        self.Iw    = self.Mt*self.Rw**2 + 3*self.Ms*self.Rw**2/3 + 1./2*sum(self.Mgb*self.Rgb**2)
        self.Ig    = self.Mp*self.Rp**2/3 + 0.5*sum(self.Mgf*self.Rgf**2)
        self.g     = 9.81  # [m/s^2]
        # starting values
        self.theta = 0.  # elevation
        self.a     = 0.  # acceleration [m/s^2]
        self.v     = 0.  # velocity [m/s]
        self.x     = 0.  # distance
        self.gb    = 0  # gear in back
        self.gf    = 0  # gear in front
        self.T0    = time.time()
        self.t     = 0.
        self.F     = 0.  # force on pedal
        self.phi   = 0.  # rotation of pedal

    def update(self):
        delt     = time.time() - self.t - self.T0
        delx     = self.v*delt + self.a*delt**2/2
        self.x   = self.x + delx
        self.phi = (self.phi + self.Rgb[self.gb]/self.Rgf[self.gf]*self.Rw*delx)%(2*np.pi)
        self.v   = self.v + self.a*delt
        self.a   = (self.Rw*self.Rgf[self.gf]*(self.Rp*self.Rgb[self.gb]*self.F
                                               + self.Rgf[self.gf]*self.Rw*self.M*self.g*np.sin(self.theta))/
                    (self.Rgf[self.gf]**2*(self.Iw + self.M*self.Rw**2) + self.Ig*self.Rgb[self.gb]**2))
        self.t   = self.t + delt

    def printval(self):
        #print 'current acceleration = %f' %self.a
        #print 'current speed = %f' %self.v
        #print 'distance traveled = %f' %self.x
        print self.F
        print self.theta

    def shift_front_up(self):
        if not self.gf == len(self.Rgf) - 1:
            self.gf = self.gf + 1
        else:
            print 'Already in highest gear'

    def shift_front_dn(self):
        if not self.gf == 0:
            self.gf = self.gf - 1
        else:
            print 'Already in lowest gear'

    def shift_back_up(self):
        if not self.gb == len(self.Rgb) - 1:
            self.gb = self.gb + 1
        else:
            print 'Already in highest gear'

    def shift_back_dn(self):
        if not self.gb == 0:
            self.gb = self.gb - 1
        else:
            print 'Already in lowest gear'

    def change_elev(self, deg):
        theta = self.theta + deg*np.pi/180.
        if abs(theta) <= 45:
            self.theta = theta

    def LT(self, value):
        if self.phi >= np.pi:
            self.F = value*self.Fm

    def RT(self, value):
        if self.phi < np.pi:
            self.F = value*self.Fm
            print '...'

    def LB(self, value):
        if value == 1:
            self.shift_front_up()

    def RB(self, value):
        if value == 1:
            self.shift_back_dn()

    def LTS(self, value):
        if value == 1:
            self.shift_front_dn()

    def RTS(self, value):
        if value == 1:
            self.shift_back_up()

    def DP(self, value):
        if value[1] != 0:
            self.change_elev(2.5*value[1])

    #called by the thread
    def run(self):
        self._start()

    #start the controller
    def _start(self):
        self.running = True
        while self.running:
            self.update()
            time.sleep(2)

    #stops the controller
    def stop(self):
        self.running = False


if __name__=='__main__':
    bike = Bike()
    c = XboxController(controllerCallBack=None, deadzone=30, scale=100, invertYAxis=True)
    c.setupControlCallback(c.XboxControls.LTRIGGER,   bike.LT)
    c.setupControlCallback(c.XboxControls.RTRIGGER,   bike.RT)
    c.setupControlCallback(c.XboxControls.LB,         bike.LB)
    c.setupControlCallback(c.XboxControls.RB,         bike.RB)
    c.setupControlCallback(c.XboxControls.LEFTTHUMB,  bike.LTS)
    c.setupControlCallback(c.XboxControls.RIGHTTHUMB, bike.RTS)
    c.setupControlCallback(c.XboxControls.DPAD,       bike.DP)

    try:
        #start the controller

        print "xbox controller running"
        while True:
            c.start()
            time.sleep(1)
            c.stop()
            bike.start()
            time.sleep(1)
            bike.stop()
            bike.printval()

    #Ctrl C
    except KeyboardInterrupt:
        print "User cancelled"

    #error
    except:
        print "Unexpected error:", sys.exc_info()[0]
        raise

    finally:
        #stop the controller
        c.stop()
