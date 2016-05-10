import os
import sys
sys.path.append(os.path.abspath(__file__).replace('raider-cosmobot16/raider.py', '') + 'pybotics/')

import time
import numpy as np
import math
import control.octosnake.octosnake as octosnake
import dynamixel
import PyKDL as kdl


class Raider(object):

    def __init__(self, trim=np.full(25, 0), name='raider'):

        # Configuration
        self._name = name
        self._trim = trim
        self.dxl = dynamixel.Dynamixel()
        self.joint_position = np.full(25, 512)

        # Oscillators
        self.osc = []
        for i in range(25):
            self.osc.append(octosnake.Oscillator())

        # Inverse kinematics
        self.leg = kdl.Chain()
        self.leg.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.RotX), kdl.Frame(kdl.Vector(0, 0, 0))))
        self.leg.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.RotY), kdl.Frame(kdl.Vector(0, 0, -75))))
        self.leg.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.RotY), kdl.Frame(kdl.Vector(0, 0, -75))))

        self.ik_solver = kdl.ChainIkSolverPos_LMA(self.leg)

        self.current_angles = kdl.JntArray(self.leg.getNrOfJoints())
        self.result_angles = kdl.JntArray(self.leg.getNrOfJoints())

        self.right_leg_YPP = [482, 612, 412]
        self.left_leg_YPP = [542, 412, 612]

        # self.alpha1=45
        # self.alpha2=-45
        # self.beta=self.alpha2-self.alpha1

    def move(self, id, position):
        self.dxl.com.write(self.dxl._coder(1, id, 30, int(position+self._trim[id])))
        if id == 17 or id == 18:
            self.dxl.com.write(self.dxl._coder(1, id+100, 30, int(position)+self._trim[id]))
        self.joint_position[id] = position

    def zero(self):

        for i in range(0, 11):
            self.move(i, 512)

        for i in range(13, 25):
            self.move(i, 512)

    def home(self, h=0, a=0):
        self.move(1, 512)
        self.move(2, 512)
        self.move(3, 512)
        self.move(4, 512)
        self.move(5, 262)
        self.move(6, 762)
        self.move(7, 462)
        self.move(8, 562)
        self.move(9, 62)
        self.move(10, 952)
        self.move(13, 512)
        self.move(14, 512)
        self.move(15, 512-a)
        self.move(16, 512+a)
        self.move(17, 512-h)
        self.move(18, 512+h)
        self.move(19, 512+h)
        self.move(20, 512-h)
        self.move(21, 512+h-18)
        self.move(22, 512-h+18)
        self.move(23, 512+a)
        self.move(24, 512-a)

    def stepL(self, steps):
        self.home(-140, 30)


        a_offset = 30
        h_offset = -140
        period = 250
        amplitude = [30, 10, 20]
        offset = [0, 0, 0]
        phase = [0, 180, 180]

        for i in range(3):
            self.osc[i].period = period
            self.osc[i].amplitude = amplitude[i]
            self.osc[i].phase = phase[i]
            self.osc[i].offset = offset[i]

        init_ref = time.time()
        final = init_ref + float(period*steps)/1000
        self.osc[0].ref_time = init_ref
        self.osc[1].ref_time = init_ref
        self.osc[2].ref_time = init_ref

        while time.time() < final:
            for i in range(3):
                self.osc[i].refresh()

            self.move(15, 512-a_offset+self.osc[0].output)
            self.move(16, 512+a_offset+self.osc[1].output)
            self.move(23, 512+a_offset+self.osc[1].output)
            self.move(24, 512-a_offset)

            self.move(17, 512-h_offset-self.osc[2].output)
            self.move(19, 512+h_offset+self.osc[2].output)
            self.move(21, 512+h_offset-20+self.osc[2].output)
            time.sleep(0.01)

    def stepR(self, steps):
        self.home(-140, 30)

        a_offset = 30
        h_offset = -140
        period = 250
        amplitude = [10, 30, 20]
        offset = [0, 0, 0]
        phase = [0, 180, 180]

        for i in range(3):
            self.osc[i].period = period
            self.osc[i].amplitude = amplitude[i]
            self.osc[i].phase = phase[i]
            self.osc[i].offset = offset[i]

        init_ref = time.time()
        final = init_ref + float(period*steps)/1000
        self.osc[0].ref_time = init_ref
        self.osc[1].ref_time = init_ref
        self.osc[2].ref_time = init_ref

        while time.time() < final:
            for i in range(3):
                self.osc[i].refresh()

            self.move(15, 512-a_offset+self.osc[0].output)
            self.move(16, 512+a_offset+self.osc[1].output)
            self.move(23, 512+a_offset)
            self.move(24, 512-a_offset+self.osc[0].output)

            self.move(18, 512+h_offset+self.osc[2].output)
            self.move(20, 512-h_offset-self.osc[2].output)
            self.move(22, 512-h_offset+20-self.osc[2].output)
            time.sleep(0.01)

    def turnR(self, steps):
        self.home(-140, 10)
        self.move(16, 512+30)

        h_offset = -120
        period = 300
        amplitude = [25, 25, 30, 10, 10]
        offset = [0, 0, 0, 0, 0]
        phase = [90, 270, 180, 0, 180]

        for i in range(5):
            self.osc[i].period = period
            self.osc[i].amplitude = amplitude[i]
            self.osc[i].phase = phase[i]
            self.osc[i].offset = offset[i]

        init_ref = time.time()
        final = init_ref + float(period*steps)/1000
        self.osc[0].ref_time = init_ref
        self.osc[1].ref_time = init_ref
        self.osc[2].ref_time = init_ref
        self.osc[3].ref_time = init_ref
        self.osc[4].ref_time = init_ref

        while time.time() < final:
            for i in range(5):
                self.osc[i].refresh()
            self.move(13, 512+self.osc[0].output)
            self.move(14, 512+self.osc[1].output)
            self.move(2, 512+self.osc[2].output)
            self.move(17, 512-h_offset-self.osc[3].output)
            self.move(19, 512+h_offset+self.osc[3].output)
            self.move(21, 512+h_offset-18+self.osc[3].output)
            self.move(18, 512+h_offset+self.osc[4].output)
            self.move(20, 512-h_offset-self.osc[4].output)
            self.move(22, 512-h_offset+18-self.osc[4].output)
            time.sleep(0.01)

    def turnL(self, steps):
        self.home(-140, 10)

        self.move(15, 512-30)

        h_offset = -120
        period = 300
        amplitude = [25, 25, 30, 10, 10]
        offset = [0, 0, 0, 0, 0]
        phase = [270, 90, 180, 0, 180]

        for i in range(5):
            self.osc[i].period = period
            self.osc[i].amplitude = amplitude[i]
            self.osc[i].phase = phase[i]
            self.osc[i].offset = offset[i]

        init_ref = time.time()
        final = init_ref + float(period*steps)/1000
        self.osc[0].ref_time = init_ref
        self.osc[1].ref_time = init_ref
        self.osc[2].ref_time = init_ref
        self.osc[3].ref_time = init_ref
        self.osc[4].ref_time = init_ref

        while time.time() < final:
            for i in range(5):
                self.osc[i].refresh()
            self.move(13, 512+self.osc[0].output)
            self.move(14, 512+self.osc[1].output)
            self.move(2, 512+self.osc[2].output)
            self.move(17, 512-h_offset-self.osc[3].output)
            self.move(19, 512+h_offset+10+self.osc[3].output)
            self.move(21, 512+h_offset-18+self.osc[3].output)
            self.move(18, 512+h_offset+self.osc[4].output)
            self.move(20, 512-h_offset-10-self.osc[4].output)
            self.move(22, 512-h_offset+18-self.osc[4].output)
            time.sleep(0.01)

    def punchL(self):
        self.home(-140, 30)
        time.sleep(0.1)

        a_R = 70
        a_L = 60
        h_R = -70
        h_L = -160

        self.move(15, 512-a_R)
        self.move(16, 512+a_L)
        self.move(17, 512-h_R)
        self.move(18, 512+h_L)
        self.move(19, 512+h_R)
        self.move(20, 512-h_L)
        self.move(21, 512+h_R-35)
        self.move(22, 512-h_L+35)
        self.move(23, 512+100)
        self.move(24, 512-25)#-a_R)

        self.move(4, 512)
        self.move(6, 762)
        self.move(8, 200)
        self.move(10, 412)

        time.sleep(0.25)

        self.move(4, 512)
        self.move(6, 512)
        self.move(8, 190)
        self.move(10, 562)

        a_R = 50
        a_L = 70
        h_R = -140
        h_L = -140

        self.move(15, 512-a_R)
        self.move(16, 512+a_L)
        self.move(17, 512-h_R)
        self.move(18, 512+h_L)
        self.move(19, 512+14+h_R)
        self.move(20, 512-14-h_L)
        self.move(21, 512+h_R-18)
        self.move(22, 512-h_L+18)
        self.move(23, 512+100)
        self.move(24, 512-a_R)

        time.sleep(0.5)

    def punchR(self):
        self.home(-140, 30)
        time.sleep(0.1)

        a_R = 60
        a_L = 70
        h_R = -160
        h_L = -70

        self.move(15, 512-a_R)
        self.move(16, 512+a_L)
        self.move(17, 512-h_R)
        self.move(18, 512+h_L)
        self.move(19, 512+h_R)
        self.move(20, 512-h_L)
        self.move(21, 512+h_R-35)
        self.move(22, 512-h_L+35)
        self.move(23, 512+25)
        self.move(24, 512-100)#-a_R)

        self.move(3, 512)
        self.move(5, 362)
        self.move(7, 824)
        self.move(9, 612)

        time.sleep(0.25)

        self.move(3, 512)
        self.move(5, 512)
        self.move(7, 834)
        self.move(9, 462)

        a_R = 70
        a_L = 50
        h_R = -140
        h_L = -140

        self.move(15, 512-a_R)
        self.move(16, 512+a_L)
        self.move(17, 512-h_R)
        self.move(18, 512+h_L)
        self.move(19, 512+14+h_R)
        self.move(20, 512-14-h_L)
        self.move(21, 512+h_R-18)
        self.move(22, 512-h_L+18)
        self.move(23, 512+a_L)
        self.move(24, 512-100)

        time.sleep(0.5)

    def getUp(self):
        self.home(-200, 0)
        time.sleep(0.5)
        self.move(17, 812)
        self.move(18, 212)
        time.sleep(0.1)

        self.move(4, 362)
        self.move(6, 812)
        self.move(8, 512)
        self.move(10, 412)
        self.move(3, 662)
        self.move(5, 212)
        self.move(7, 512)
        self.move(9, 612)
        time.sleep(0.3)

        self.move(4, 362)
        self.move(6, 812)
        self.move(8, 512)
        self.move(10, 412)
        self.move(3, 662)
        self.move(5, 212)
        self.move(7, 512)
        self.move(9, 612)
        time.sleep(0.3)

        self.move(4, 362)
        self.move(6, 812)
        self.move(8, 512)
        self.move(10, 412)
        self.move(3, 622)
        self.move(5, 212)
        self.move(7, 512)
        self.move(9, 612)
        time.sleep(0.5)

        self.move(2, 612)
        time.sleep(2)
        self.home(-140, 30)

    def backGetUp(self):
        self.home(-200, 0)
        time.sleep(0.5)
        self.move(17, 812)
        self.move(18, 212)
        self.move(15, 512-100)
        self.move(16, 512+100)
        time.sleep(0.5)

        self.move(4, 1023)
        self.move(8, 512)
        self.move(3, 1)
        self.move(7, 512)
        time.sleep(0.5)

        self.move(6, 812)
        self.move(10, 812)
        self.move(5, 212)
        self.move(9, 212)
        time.sleep(0.5)

        self.move(4, 812)
        self.move(10, 512)
        self.move(3, 212)
        self.move(9, 512)
        time.sleep(0.5)

        self.move(3, 392)
        self.move(4, 632)

        self.move(9, 612)
        self.move(10, 412)
        time.sleep(0.5)

        self.move(2, 732)
        self.move(5, 312)
        self.move(6, 812)
        time.sleep(0.7)

        self.move(5, 512)
        self.move(6, 512)
        self.move(2, 512)

    def angleToUnits(self, angle):
        return (int)((angle * 1024.0 / 300) + 512)

    def unitsToAngle(self, units):
        return (units - 512) * 300.0 / 1024

    def leftLegIK(self, x, y, z):
        # self.left_leg_YPP = [542, 412, 612]

        target_frame = kdl.Frame(kdl.Vector(x, y, z))

        self.current_angles[0] = np.deg2rad(0.0)
        self.current_angles[1] = np.deg2rad(15.0)
        self.current_angles[2] = np.deg2rad(-30.0)

        self.ik_solver.CartToJnt(self.current_angles, target_frame, self.result_angles)
        self.result_angles[2] = self.result_angles[1] + self.result_angles[2]

        self.result_angles = map(np.rad2deg, self.result_angles)

        return self.result_angles

    def rightLegIK(x, y):
        return 1



if __name__ == "__main__":

    trims=[0,0,0,0,0,0,0,0,0,0,0,0,0,3,-2,-5,5,0,0,-5,0,-6,0,0,0]
    robot = Raider(trims)

    x = 0
    y = 10
    z = -75
    result = robot.leftLegIK(x, y, z)
    print result
    print map(robot.angleToUnits, result)
