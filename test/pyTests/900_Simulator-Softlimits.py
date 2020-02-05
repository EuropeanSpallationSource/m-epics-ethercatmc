#!/usr/bin/env python
#

import unittest
import math
import os
import sys
import time
from motor_lib import motor_lib
lib = motor_lib()
import capv_lib
import inspect
def lineno():
    return inspect.currentframe().f_back.f_lineno

###

polltime = 0.1

# Values to be used for test
# Note: Make sure to use different values to hae a good
# test coverage
myVELO =  1.0   # positioning velocity
myACCL =  1.0   # Time to VELO, seconds
#myAR   = myVELO / myACCL # acceleration, mm/sec^2

myJVEL = 5.0    # Jogging velocity
myJAR  = 6.0    # Jogging acceleration, mm/sec^2

myBVEL = 2.0    # backlash velocity
myBACC = 1.5    # backlash acceleration, seconds
myBAR  = myBVEL / myBACC  # backlash acceleration, mm/sec^2
myBDST = 0  # backlash destination, mm

#Different values, high use even, low uses odd
#
myLowHardLimitPos  =   -9.0
myCfgDLLM          =   -7.0
myDLLM             =   -5.0
myStartposDial     =    0.0
myDHLM             =    4.0
myCfgDHLM          =    6.0
myHighHardLimitPos =    8.0

#Comparing floating points may fail because of rounding problems
maxdelta           = 0.01

# We need to test different combinations of
# - MRES > 0       ; MRES < 0
# DIR=0; OFF=X     ; DIR=1; OFF=X
# (Those above are the main loop)
#
#
# Controller with and without "read only limits"
# Write to DHLM, DLLM, HLM, LLM


def motorInitVeloAcc(self, motor, tc_no, encRel):
    msta             = int(capv_lib.capvget(motor + '.MSTA'))
    assert (msta & lib.MSTA_BIT_HOMED) #, 'MSTA.homed (Axis has been homed)')

    # Prepare parameters for jogging and backlash
    capv_lib.capvput(motor + '.VELO', myVELO)
    capv_lib.capvput(motor + '.ACCL', myACCL)

    capv_lib.capvput(motor + '.JVEL', myJVEL)
    capv_lib.capvput(motor + '.JAR', myJAR)

    capv_lib.capvput(motor + '.BVEL', myBVEL)
    capv_lib.capvput(motor + '.BACC', myBACC)
    capv_lib.capvput(motor + '.BDST', myBDST)
    capv_lib.capvput(motor + '.UEIP', encRel)
    # Move the motor to 0, to avoid limit swicth activation
    capv_lib.capvput(motor + '.DVAL', myStartposDial)
    # Bit speed it up, by setting the position in the simulator
    lib.setValueOnSimulator(motor, tc_no, "fActPosition", myStartposDial)
    # and wait for the movement to finish
    ret3 = lib.waitForStop(motor, tc_no, 2.0)


def motorInitLimitsNoC(self, motor, tc_no):
    capv_lib.capvput(motor + '-CfgDLLM', myCfgDLLM)
    capv_lib.capvput(motor + '-CfgDHLM', myCfgDHLM)
    capv_lib.capvput(motor + '-CfgDLLM-En', 0, wait=True)
    capv_lib.capvput(motor + '-CfgDHLM-En', 0, wait=True)

    capv_lib.capvput(motor + '.DHLM', myDHLM)
    capv_lib.capvput(motor + '.DLLM', myDLLM)

def motorInitLimitsWithC(self, motor, tc_no):
    capv_lib.capvput(motor + '-CfgDLLM-En', 0, wait=True)
    capv_lib.capvput(motor + '-CfgDHLM-En', 0, wait=True)
    capv_lib.capvput(motor + '-CfgDHLM', myCfgDHLM)
    capv_lib.capvput(motor + '-CfgDLLM', myCfgDLLM)
    capv_lib.capvput(motor + '-CfgDLLM-En', 1, wait=True)
    capv_lib.capvput(motor + '-CfgDHLM-En', 1, wait=True)

    capv_lib.capvput(motor + '.DHLM', myDHLM)
    capv_lib.capvput(motor + '.DLLM', myDLLM)

def setMresDirOff(self, motor, tc_no, mres, dir, off):
    capv_lib.capvput(motor + '.MRES', mres)
    capv_lib.capvput(motor + '.DIR',  dir)
    capv_lib.capvput(motor + '.OFF', off)


def readBackParamVerify(self, motor, tc_no, pvSuffix, expVal):
    pvname = motor + pvSuffix
    maxTime = 5 # 5 seconds maximum to poll all parameters
    testPassed = False
    maxDelta = math.fabs(expVal) * 0.02 # 2 % error tolerance margin
    while maxTime > 0:
        actVal = capv_lib.capvget(pvname)
        print('%s:%d %s expVal=%f actVal=%f' % (tc_no, lineno(), pvname, expVal, actVal))

        res = lib.calcAlmostEqual(motor, tc_no, expVal, actVal, maxDelta)
        print('%s:%d res=%s' % (tc_no, lineno(), res))
        if (res == True) or (res != 0) :
            return True
        else:
            time.sleep(polltime)
            maxTime = maxTime - polltime
    return False

def setLimit(self, motor, tc_no, field, value, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
    capv_lib.capvput(self.motor + '-DbgStrToLOG', "Start " + str(tc_no))
    oldDHLM = capv_lib.capvget(motor + '.DHLM', use_monitor=False)
    oldDLLM = capv_lib.capvget(motor + '.DLLM', use_monitor=False)

    capv_lib.capvput(motor + '.DHLM', oldDHLM)
    capv_lib.capvput(motor + '.DLLM', oldDLLM)
    capv_lib.capvput(motor + '.' + field, value)

    okDHLM = readBackParamVerify(self, motor, tc_no, '.DHLM', expDHLM)
    okDLLM = readBackParamVerify(self, motor, tc_no, '.DLLM', expDLLM)

    okHLM = readBackParamVerify(self, motor, tc_no, '.HLM', expHLM)
    okLLM = readBackParamVerify(self, motor, tc_no, '.LLM', expLLM)

    okM3rhlm = readBackParamVerify(self, motor, tc_no, '-M3RHLM', expM3rhlm)
    okM3rllm = readBackParamVerify(self, motor, tc_no, '-M3RLLM', expM3rllm)

    testPassed = okDHLM and okDLLM and okHLM and okLLM and okM3rhlm and okM3rllm
    if testPassed:
        capv_lib.capvput(self.motor + '-DbgStrToLOG', "Passed " + str(tc_no))
    else:
        capv_lib.capvput(self.motor + '-DbgStrToLOG', "Failed " + str(tc_no))
    assert (testPassed)


class Test(unittest.TestCase):
    hasROlimit = 0
    drvUseEGU_RB = None
    drvUseEGU = 0
    motor = os.getenv("TESTEDMOTORAXIS")
    capv_lib.capvput(motor + '-DbgStrToLOG', "Start " + os.path.basename(__file__)[0:20])
    vers = float(capv_lib.capvget(motor + '.VERS'))
    if vers >= 6.94 and vers <= 7.09 :
        hasROlimit = 1
        drvUseEGU_RB = capv_lib.capvget(motor + '-DrvUseEGU-RB')
        drvUseEGU = 0
        if drvUseEGU_RB == 1:
            capv_lib.capvput(motor + '-DrvUseEGU', drvUseEGU)
            #drvUseEGU = capv_lib.capvget(motor + '-DrvUseEGU-RB')

    def test_TC_90000(self):
        tc_no = 90010
        capv_lib.capvput(self.motor + '-DbgStrToLOG', "Start " + str(tc_no))
        lib.motorInitAllForBDST(self.motor, 90000)
        capv_lib.capvput(self.motor + '-DbgStrToLOG', "End " + str(tc_no))

    def test_TC_90010(self):
        tc_no = 90010
        encRel = 0
        capv_lib.capvput(self.motor + '-DbgStrToLOG', "Start " + str(tc_no))
        #vers = float(capv_lib.capvget(self.motor + '.VERS'))
        #print('%s vers=%g' %  (tc_no, vers))
        #self.assertEqual(0, 1, '1 != 0')
        testPassed = readBackParamVerify(self, self.motor, tc_no, '-DrvUseEGU-RB', 0)

        motorInitVeloAcc(self, self.motor, tc_no, encRel)
        mres = 0.1
        dir = 0
        off = 0.5
        setMresDirOff(self, self.motor, tc_no, mres, dir, off)
        motorInitLimitsNoC(self, self.motor, tc_no)
        if testPassed:
            capv_lib.capvput(self.motor + '-DbgStrToLOG', "Passed " + str(tc_no))
        else:
            capv_lib.capvput(self.motor + '-DbgStrToLOG', "Failed " + str(tc_no))
        assert (testPassed)

    def test_TC_90011(self):
        tc_no = 90011
        #setLimit(self, motor, tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, self.motor, tc_no, "DHLM", 4.1,  4.1,    -5.0,    4.6,    -4.5,   41.0,      -50.0)

    def test_TC_90012(self):
        tc_no = 90012
        #setLimit(self, motor, tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, self.motor, tc_no, "HLM",  4.7,  4.2,    -5.0,    4.7,    -4.5,   42.0,      -50.0)

    def test_TC_90013(self):
        tc_no = 90013
        #setLimit(self, motor, tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, self.motor, tc_no, "DLLM", -5.3, 4.2,     -5.3,    4.7,    -4.8,   42.0,      -53.0)

    def test_TC_90014(self):
        tc_no = 90014
        #setLimit(self, motor, tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, self.motor, tc_no, "LLM", -5.4, 4.2,     -5.9,    4.7,    -5.4,    42.0,      -59.0)

    ###################################################################################################################
    #Invert mres
    def test_TC_90020(self):
        tc_no = 90020
        capv_lib.capvput(self.motor + '-DbgStrToLOG', "Start " + str(tc_no))
        encRel = 0
        #                                       mres, dir,off, hlm, expHLM, expM3rhlm, expLLM, expM3rllm)
        motorInitVeloAcc(self, self.motor, tc_no, encRel)
        mres = -0.1
        dir = 0
        off = 0.5
        setMresDirOff(self, self.motor, tc_no, mres, dir, off)
        motorInitLimitsNoC(self, self.motor, tc_no)
        capv_lib.capvput(self.motor + '-DbgStrToLOG', "End " + str(tc_no))

    def test_TC_90021(self):
        tc_no = 90021
        #setLimit(self, motor, tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, self.motor, tc_no, "DHLM", 4.1,  4.1,    -5.0,    4.6,    -4.5,   50.0,      -41.0)


    def test_TC_90022(self):
        tc_no = 90022
        #setLimit(self, motor, tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, self.motor, tc_no, "HLM",  4.7,  4.2,    -5.0,    4.7,    -4.5,   50.0,      -42.0)


    def test_TC_90023(self):
        tc_no = 90023
        #setLimit(self, motor, tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, self.motor, tc_no, "DLLM", -5.3, 4.2,     -5.3,    4.7,    -4.8,   53.0,     -42.0)

    def test_TC_90024(self):
        tc_no = 90024
        #setLimit(self, motor, tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, self.motor, tc_no, "LLM", -5.4, 4.2,     -5.9,    4.7,    -5.4,    59.0,     -42.0)


    ###################################################################################################################
    #Invert dir
    def test_TC_90030(self):
        tc_no = 90030
        encRel = 0
        capv_lib.capvput(self.motor + '-DbgStrToLOG', "Start " + str(tc_no))
        #                                       mres, dir,off, hlm, expHLM, expM3rhlm, expLLM, expM3rllm)
        motorInitVeloAcc(self, self.motor, tc_no, encRel)
        mres = 0.1
        dir = 1
        off = 0.5
        setMresDirOff(self, self.motor, tc_no, mres, dir, off)
        motorInitLimitsNoC(self, self.motor, tc_no)
        capv_lib.capvput(self.motor + '-DbgStrToLOG', "End " + str(tc_no))

    def test_TC_90031(self):
        tc_no = 90031
        #setLimit(self, motor, tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, self.motor, tc_no, "DHLM", 4.1,  4.1,    -5.0,    5.5,    -3.6,   41.0,      -50.0)


    def test_TC_90032(self):
        tc_no = 90032
        #setLimit(self, motor, tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, self.motor, tc_no, "HLM",  4.7,  4.1,      -4.2,   4.7,    -3.6,   41.0,      -42.0)


    def test_TC_90033(self):
        tc_no = 90033
        #setLimit(self, motor, tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, self.motor, tc_no, "DLLM", -5.3, 4.1,     -5.3,    5.8,    -3.6,   41.0,     -53.0)

    def test_TC_90034(self):
        tc_no = 90034
        #setLimit(self, motor, tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, self.motor, tc_no, "LLM", -5.4, 5.9,     -5.3,    5.8,    -5.4,    59.0,     -53.0)

    ###################################################################################################################
    #Invert mres, invert dir
    def test_TC_90040(self):
        tc_no = 90040
        encRel = 0
        capv_lib.capvput(self.motor + '-DbgStrToLOG', "Start " + str(tc_no))
        #                                       mres, dir,off, hlm, expHLM, expM3rhlm, expLLM, expM3rllm)
        motorInitVeloAcc(self, self.motor, tc_no, encRel)
        mres = -0.1
        dir = 1
        off = 0.5
        setMresDirOff(self, self.motor, tc_no, mres, dir, off)
        motorInitLimitsNoC(self, self.motor, tc_no)
        capv_lib.capvput(self.motor + '-DbgStrToLOG', "End " + str(tc_no))

    def test_TC_90041(self):
        tc_no = 90041
        #setLimit(self, motor, tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, self.motor, tc_no, "DHLM", 4.1,  4.1,    -5.0,    5.5,    -3.6,    50.0,      -41.0)

    def test_TC_90042(self):
        tc_no = 90042
        #setLimit(self, motor, tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, self.motor, tc_no, "HLM",  4.7,  4.1,    -4.2,    4.7,    -3.6,    42.0,      -41.0)

    def test_TC_90043(self):
        tc_no = 90043
        #setLimit(self, motor, tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, self.motor, tc_no, "DLLM", -5.3, 4.1,     -5.3,    5.8,   -3.6,   53.0,      -41.0)

    def test_TC_90044(self):
        tc_no = 90044
        #setLimit(self, motor, tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, self.motor, tc_no, "LLM", -5.4, 5.9,     -5.3,    5.8,    -5.4,    53.0,      -59.0)


#########################################################################################################
# Test clipping
    def test_TC_90050(self):
        tc_no = 90050
        encRel = 0
        capv_lib.capvput(self.motor + '-DbgStrToLOG', "Start " + str(tc_no))

        print('%s vers=%g hasROlimit=%d' %  (tc_no, self.vers, self.hasROlimit))
        self.assertEqual(1, self.hasROlimit, 'motorRecord supports RO soft limits')

        #                                       mres, dir,off, hlm, expHLM, expM3rhlm, expLLM, expM3rllm)
        motorInitVeloAcc(self, self.motor, tc_no, encRel)
        mres = 0.1
        dir = 0
        off = 0.5
        setMresDirOff(self, self.motor, tc_no, mres, dir, off)
        motorInitLimitsWithC(self, self.motor, tc_no)
        capv_lib.capvput(self.motor + '-DbgStrToLOG', "End " + str(tc_no))

    def test_TC_90051(self):
        self.assertEqual(1, self.hasROlimit, 'motorRecord supports RO soft limits')
        tc_no = 90051
        #setLimit(self, motor, tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, self.motor, tc_no, "DHLM", 10,  0.6,    -0.7,     1.1,    -0.2,   6.0,      -7.0)

    def test_TC_90052(self):
        self.assertEqual(1, self.hasROlimit, 'motorRecord supports RO soft limits')
        tc_no = 90052
        #setLimit(self, motor, tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, self.motor, tc_no, "HLM",  10,  0.6,    -0.7,     1.1,    -0.2,   6.0,      -7.0)

    def test_TC_90053(self):
        self.assertEqual(1, self.hasROlimit, 'motorRecord supports RO soft limits')
        tc_no = 90053
        #setLimit(self, motor, tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, self.motor, tc_no, "DLLM", -10.0, 0.6,  -0.7,     1.1,    -0.2,   6.0,      -7.0)

    def test_TC_90054(self):
        self.assertEqual(1, self.hasROlimit, 'motorRecord supports RO soft limits')
        tc_no = 90054
        #setLimit(self, motor, tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, self.motor, tc_no, "LLM", -10.0,  0.6,   -0.7,     1.1,    -0.2,   6.0,      -7.0)

    #Invert mres
    def test_TC_90060(self):
        self.assertEqual(1, self.hasROlimit, 'motorRecord supports RO soft limits')
        tc_no = 90060
        capv_lib.capvput(self.motor + '-DbgStrToLOG', "Start " + str(tc_no))
        encRel = 0
        #                                       mres, dir,off, hlm, expHLM, expM3rhlm, expLLM, expM3rllm)
        motorInitVeloAcc(self, self.motor, tc_no, encRel)
        mres = -0.1
        dir = 0
        off = 0.5
        setMresDirOff(self, self.motor, tc_no, mres, dir, off)
        motorInitLimitsWithC(self, self.motor, tc_no)
        capv_lib.capvput(self.motor + '-DbgStrToLOG', "End " + str(tc_no))

    def test_TC_90061(self):
        self.assertEqual(1, self.hasROlimit, 'motorRecord supports RO soft limits')
        tc_no = 90061
        #setLimit(self, motor, tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, self.motor, tc_no, "DHLM", 10,  0.7,     -0.6,    1.2,    -0.1,    6.0,      -7.0)

    def test_TC_90062(self):
        self.assertEqual(1, self.hasROlimit, 'motorRecord supports RO soft limits')
        tc_no = 90062
        #setLimit(self, motor, tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, self.motor, tc_no, "HLM",  10,  0.7,     -0.6,    1.2,    -0.1,    6.0,      -7.0)

    def test_TC_90063(self):
        self.assertEqual(1, self.hasROlimit, 'motorRecord supports RO soft limits')
        tc_no = 90063
        #setLimit(self, motor, tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, self.motor, tc_no, "DLLM", -10.0, 0.7,    -0.6,    1.2,    -0.1,   6.0,      -7.0)

    def test_TC_90064(self):
        self.assertEqual(1, self.hasROlimit, 'motorRecord supports RO soft limits')
        tc_no = 90064
        #setLimit(self, motor, tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, self.motor, tc_no, "LLM", -10.0,  0.7,    -0.6,    1.2,    -0.1,   6.0,      -7.0)

    #Invert dir
    def test_TC_90070(self):
        self.assertEqual(1, self.hasROlimit, 'motorRecord supports RO soft limits')
        tc_no = 90070
        encRel = 0
        capv_lib.capvput(self.motor + '-DbgStrToLOG', "Start " + str(tc_no))
        #                                       mres, dir,off, hlm, expHLM, expM3rhlm, expLLM, expM3rllm)
        motorInitVeloAcc(self, self.motor, tc_no, encRel)
        mres = 0.1
        dir = 1
        off = 0.5
        setMresDirOff(self, self.motor, tc_no, mres, dir, off)
        motorInitLimitsWithC(self, self.motor, tc_no)
        capv_lib.capvput(self.motor + '-DbgStrToLOG', "End " + str(tc_no))

    def test_TC_90071(self):
        self.assertEqual(1, self.hasROlimit, 'motorRecord supports RO soft limits')
        tc_no = 90071
        #setLimit(self, motor, tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, self.motor, tc_no, "DHLM", 10,  0.6,     -0.7,     1.2,    -0.1,    6.0,      -7.0)

    def test_TC_90072(self):
        self.assertEqual(1, self.hasROlimit, 'motorRecord supports RO soft limits')
        tc_no = 90072
        #setLimit(self, motor, tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, self.motor, tc_no, "HLM",  10,   0.6,     -0.7,     1.2,     -0.1,    6.0,      -7.0)

    def test_TC_90073(self):
        self.assertEqual(1, self.hasROlimit, 'motorRecord supports RO soft limits')
        tc_no = 90073
        #setLimit(self, motor, tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, self.motor, tc_no, "DLLM", -10.0, 0.6,     -0.7,     1.2,     -0.1,    6.0,      -7.0)

    def test_TC_90074(self):
        self.assertEqual(1, self.hasROlimit, 'motorRecord supports RO soft limits')
        tc_no = 90074
        #setLimit(self, motor, tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, self.motor, tc_no, "LLM", -10.0,  0.6,     -0.7,     1.2,     -0.1,    6.0,      -7.0)

    #Invert MRES and dir
    def test_TC_90080(self):
        self.assertEqual(1, self.hasROlimit, 'motorRecord supports RO soft limits')
        tc_no = 90080
        capv_lib.capvput(self.motor + '-DbgStrToLOG', "Start " + str(tc_no))
        encRel = 0
        #                                       mres, dir,off, hlm, expHLM, expM3rhlm, expLLM, expM3rllm)
        motorInitVeloAcc(self, self.motor, tc_no, encRel)
        mres = -0.1
        dir = 1
        off = 0.5
        setMresDirOff(self, self.motor, tc_no, mres, dir, off)
        motorInitLimitsWithC(self, self.motor, tc_no)
        capv_lib.capvput(self.motor + '-DbgStrToLOG', "End " + str(tc_no))

    def test_TC_90081(self):
        self.assertEqual(1, self.hasROlimit, 'motorRecord supports RO soft limits')
        tc_no = 90081
        #setLimit(self, motor, tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, self.motor, tc_no, "DHLM", 10,  0.7,      -0.6,    1.1,    -0.2,   6.0,      -7.0)

    def test_TC_90082(self):
        self.assertEqual(1, self.hasROlimit, 'motorRecord supports RO soft limits')
        tc_no = 90082
        #setLimit(self, motor, tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, self.motor, tc_no, "HLM",  10,  0.7,     -0.6,    1.1,    -0.2,    6.0,      -7.0)

    def test_TC_90083(self):
        self.assertEqual(1, self.hasROlimit, 'motorRecord supports RO soft limits')
        tc_no = 90083
        #setLimit(self, motor, tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, self.motor, tc_no, "DLLM", -10, 0.7,     -0.6,    1.1,    -0.2,    6.0,      -7.0)

    def test_TC_90084(self):
        self.assertEqual(1, self.hasROlimit, 'motorRecord supports RO soft limits')
        tc_no = 90084
        #setLimit(self, motor, tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, self.motor, tc_no, "LLM", -10,  0.7,     -0.6,    1.1,    -0.2,    6.0,      -7.0)


    def test_TC_900999(self):
        if self.drvUseEGU_RB == 1:
            capv_lib.capvput(self.motor + '-DrvUseEGU', 1)

