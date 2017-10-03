#!/usr/bin/env python
#

import epics
import unittest
import os
import sys
import time
from motor_lib import motor_lib
###



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
myECLLM             =   -7.0
myDLLM             =   -5.0
myStartposDial     =    0.0
myDHLM             =    4.0
myECHLM             =    6.0
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


def motorInitVeloAcc(tself, motor, tc_no, encRel):
    msta             = int(epics.caget(motor + '.MSTA'))
    assert (msta & tself.lib.MSTA_BIT_HOMED) #, 'MSTA.homed (Axis has been homed)')

    # Prepare parameters for jogging and backlash
    epics.caput(motor + '.VELO', myVELO)
    epics.caput(motor + '.ACCL', myACCL)

    epics.caput(motor + '.JVEL', myJVEL)
    epics.caput(motor + '.JAR', myJAR)

    epics.caput(motor + '.BVEL', myBVEL)
    epics.caput(motor + '.BACC', myBACC)
    epics.caput(motor + '.BDST', myBDST)
    epics.caput(motor + '.UEIP', encRel)
    epics.caput(motor + '.DVAL', myStartposDial, wait=True)


def motorInitLimitsNoC(tself, motor, tc_no):
    epics.caput(motor + '-ECLLM', myECLLM)
    epics.caput(motor + '-ECHLM', myECHLM)
    epics.caput(motor + '-ECLLM-En', 0, wait=True)
    epics.caput(motor + '-ECHLM-En', 0, wait=True)

    epics.caput(motor + '.DHLM', myDHLM)
    epics.caput(motor + '.DLLM', myDLLM)

def motorInitLimitsWithC(tself, motor, tc_no):
    epics.caput(motor + '-ECLLM-En', 0, wait=True)
    epics.caput(motor + '-ECHLM-En', 0, wait=True)
    epics.caput(motor + '-ECHLM', myECHLM)
    epics.caput(motor + '-ECLLM', myECLLM)
    epics.caput(motor + '-ECLLM-En', 1, wait=True)
    epics.caput(motor + '-ECHLM-En', 1, wait=True)

    epics.caput(motor + '.DHLM', myDHLM)
    epics.caput(motor + '.DLLM', myDLLM)

def setMresDirOff(tself, motor, tc_no, mres, dir, off):
    epics.caput(motor + '.MRES', mres)
    epics.caput(motor + '.DIR',  dir)
    epics.caput(motor + '.OFF', off)

def setLimit(tself, motor, tc_no, field, value, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
    oldDHLM = epics.caget(motor + '.DHLM', use_monitor=False)
    oldDLLM = epics.caget(motor + '.DLLM', use_monitor=False)

    epics.caput(motor + '.DHLM', oldDHLM)
    epics.caput(motor + '.DLLM', oldDLLM)
    epics.caput(motor + '.' + field, value)

    time.sleep(0.5)

    actDHLM = epics.caget(motor + '.DHLM', use_monitor=False)
    actDLLM = epics.caget(motor + '.DLLM', use_monitor=False)
    actHLM = epics.caget(motor + '.HLM', use_monitor=False)
    actLLM = epics.caget(motor + '.LLM', use_monitor=False)

    actM3rhlm = epics.caget(motor + '-M3RHLM', use_monitor=False)
    actM3rllm = epics.caget(motor + '-M3RLLM', use_monitor=False)
    print '%s expDHLM=%g expDLLM=%g expHLM=%g expLLM=%g expM3rhlm=%g expM3rllm=%g' % \
        (tc_no, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm)
    print '%s actDHLM=%g actDLLM=%g actHLM=%g actLLM=%g actM3rhlm=%g actM3rllm=%g' % \
        (tc_no, actDHLM, actDLLM, actHLM, actLLM, actM3rhlm, actM3rllm)
    okDHLM   = tself.lib.calcAlmostEqual(motor, tc_no, expDHLM,  actDHLM, maxdelta)
    okDLLM   = tself.lib.calcAlmostEqual(motor, tc_no, expDLLM,  actDLLM, maxdelta)
    okHLM    = tself.lib.calcAlmostEqual(motor, tc_no, expHLM,    actHLM, maxdelta)
    okLLM    = tself.lib.calcAlmostEqual(motor, tc_no, expLLM,    actLLM, maxdelta)
    okM3rhlm = tself.lib.calcAlmostEqual(motor, tc_no, expM3rhlm, actM3rhlm, maxdelta)
    okM3rllm = tself.lib.calcAlmostEqual(motor, tc_no, expM3rllm, actM3rllm, maxdelta)

    assert (okDHLM and okDLLM and okHLM and okLLM and okM3rhlm and okM3rllm)


class Test(unittest.TestCase):
    lib = motor_lib()
    motor = os.getenv("TESTEDMOTORAXIS")

    def test_TC_90010(self):
        tc_no = 90010
        encRel = 0
        #                                       mres, dir,off, hlm, expHLM, expM3rhlm, expLLM, expM3rllm)
        motorInitVeloAcc(self, self.motor, tc_no, encRel)
        mres = 0.1
        dir = 0
        off = 0.5
        setMresDirOff(self, self.motor, tc_no, mres, dir, off)
        motorInitLimitsNoC(self, self.motor, tc_no)

    def test_TC_90011(self):
        tc_no = 90011
        #setLimit(tself, motor, tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, self.motor, tc_no, "DHLM", 4.1,  4.1,    -5.0,    4.6,    -4.5,   41.0,      -50.0)

    def test_TC_90012(self):
        tc_no = 90012
        #setLimit(tself, motor, tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, self.motor, tc_no, "HLM",  4.7,  4.2,    -5.0,    4.7,    -4.5,   42.0,      -50.0)

    def test_TC_90013(self):
        tc_no = 90013
        #setLimit(tself, motor, tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, self.motor, tc_no, "DLLM", -5.3, 4.2,     -5.3,    4.7,    -4.8,   42.0,      -53.0)

    def test_TC_90014(self):
        tc_no = 90014
        #setLimit(tself, motor, tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, self.motor, tc_no, "LLM", -5.4, 4.2,     -5.9,    4.7,    -5.4,    42.0,      -59.0)

    ###################################################################################################################
    #Invert mres
    def test_TC_90020(self):
        tc_no = 90020
        encRel = 0
        #                                       mres, dir,off, hlm, expHLM, expM3rhlm, expLLM, expM3rllm)
        motorInitVeloAcc(self, self.motor, tc_no, encRel)
        mres = -0.1
        dir = 0
        off = 0.5
        setMresDirOff(self, self.motor, tc_no, mres, dir, off)
        motorInitLimitsNoC(self, self.motor, tc_no)

    def test_TC_90021(self):
        tc_no = 90021
        #setLimit(tself, motor, tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, self.motor, tc_no, "DHLM", 4.1,  4.1,    -5.0,    4.6,    -4.5,   50.0,      -41.0)


    def test_TC_90022(self):
        tc_no = 90022
        #setLimit(tself, motor, tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, self.motor, tc_no, "HLM",  4.7,  4.2,    -5.0,    4.7,    -4.5,   50.0,      -42.0)


    def test_TC_90023(self):
        tc_no = 90023
        #setLimit(tself, motor, tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, self.motor, tc_no, "DLLM", -5.3, 4.2,     -5.3,    4.7,    -4.8,   53.0,     -42.0)

    def test_TC_90024(self):
        tc_no = 90024
        #setLimit(tself, motor, tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, self.motor, tc_no, "LLM", -5.4, 4.2,     -5.9,    4.7,    -5.4,    59.0,     -42.0)


    ###################################################################################################################
    #Invert dir
    def test_TC_90030(self):
        tc_no = 90030
        encRel = 0
        #                                       mres, dir,off, hlm, expHLM, expM3rhlm, expLLM, expM3rllm)
        motorInitVeloAcc(self, self.motor, tc_no, encRel)
        mres = 0.1
        dir = 1
        off = 0.5
        setMresDirOff(self, self.motor, tc_no, mres, dir, off)
        motorInitLimitsNoC(self, self.motor, tc_no)

    def test_TC_90031(self):
        tc_no = 90031
        #setLimit(tself, motor, tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, self.motor, tc_no, "DHLM", 4.1,  4.1,    -5.0,    5.5,    -3.6,   41.0,      -50.0)


    def test_TC_90032(self):
        tc_no = 90032
        #setLimit(tself, motor, tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, self.motor, tc_no, "HLM",  4.7,  4.1,      -4.2,   4.7,    -3.6,   41.0,      -42.0)


    def test_TC_90033(self):
        tc_no = 90033
        #setLimit(tself, motor, tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, self.motor, tc_no, "DLLM", -5.3, 4.1,     -5.3,    5.8,    -3.6,   41.0,     -53.0)

    def test_TC_90034(self):
        tc_no = 90034
        #setLimit(tself, motor, tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, self.motor, tc_no, "LLM", -5.4, 5.9,     -5.3,    5.8,    -5.4,    59.0,     -53.0)

    ###################################################################################################################
    #Invert mres, invert dir
    def test_TC_90040(self):
        tc_no = 90040
        encRel = 0
        #                                       mres, dir,off, hlm, expHLM, expM3rhlm, expLLM, expM3rllm)
        motorInitVeloAcc(self, self.motor, tc_no, encRel)
        mres = -0.1
        dir = 1
        off = 0.5
        setMresDirOff(self, self.motor, tc_no, mres, dir, off)
        motorInitLimitsNoC(self, self.motor, tc_no)

    def test_TC_90041(self):
        tc_no = 90041
        #setLimit(tself, motor, tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, self.motor, tc_no, "DHLM", 4.1,  4.1,    -5.0,    5.5,    -3.6,    50.0,      -41.0)

    def test_TC_90042(self):
        tc_no = 90042
        #setLimit(tself, motor, tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, self.motor, tc_no, "HLM",  4.7,  4.1,    -4.2,    4.7,    -3.6,    42.0,      -41.0)

    def test_TC_90043(self):
        tc_no = 90043
        #setLimit(tself, motor, tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, self.motor, tc_no, "DLLM", -5.3, 4.1,     -5.3,    5.8,   -3.6,   53.0,      -41.0)

    def test_TC_90044(self):
        tc_no = 90044
        #setLimit(tself, motor, tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, self.motor, tc_no, "LLM", -5.4, 5.9,     -5.3,    5.8,    -5.4,    53.0,      -59.0)


#########################################################################################################
# Test clipping
    def test_TC_90050(self):
        tc_no = 90050
        encRel = 0
        #                                       mres, dir,off, hlm, expHLM, expM3rhlm, expLLM, expM3rllm)
        motorInitVeloAcc(self, self.motor, tc_no, encRel)
        mres = 0.1
        dir = 0
        off = 0.5
        setMresDirOff(self, self.motor, tc_no, mres, dir, off)
        motorInitLimitsWithC(self, self.motor, tc_no)

    def test_TC_90051(self):
        tc_no = 90051
        #setLimit(tself, motor, tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, self.motor, tc_no, "DHLM", 10,  0.6,    -0.7,     1.1,    -0.2,   6.0,      -7.0)

    def test_TC_90052(self):
        tc_no = 90052
        #setLimit(tself, motor, tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, self.motor, tc_no, "HLM",  10,  0.6,    -0.7,     1.1,    -0.2,   6.0,      -7.0)

    def test_TC_90053(self):
        tc_no = 90053
        #setLimit(tself, motor, tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, self.motor, tc_no, "DLLM", -10.0, 0.6,  -0.7,     1.1,    -0.2,   6.0,      -7.0)

    def test_TC_90054(self):
        tc_no = 90054
        #setLimit(tself, motor, tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, self.motor, tc_no, "LLM", -10.0,  0.6,   -0.7,     1.1,    -0.2,   6.0,      -7.0)

    #Invert mres
    def test_TC_90060(self):
        tc_no = 90060
        encRel = 0
        #                                       mres, dir,off, hlm, expHLM, expM3rhlm, expLLM, expM3rllm)
        motorInitVeloAcc(self, self.motor, tc_no, encRel)
        mres = -0.1
        dir = 0
        off = 0.5
        setMresDirOff(self, self.motor, tc_no, mres, dir, off)
        motorInitLimitsWithC(self, self.motor, tc_no)

    def test_TC_90061(self):
        tc_no = 90061
        #setLimit(tself, motor, tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, self.motor, tc_no, "DHLM", 10,  0.7,     -0.6,    1.2,    -0.1,    6.0,      -7.0)

    def test_TC_90062(self):
        tc_no = 90062
        #setLimit(tself, motor, tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, self.motor, tc_no, "HLM",  10,  0.7,     -0.6,    1.2,    -0.1,    6.0,      -7.0)

    def test_TC_90063(self):
        tc_no = 90063
        #setLimit(tself, motor, tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, self.motor, tc_no, "DLLM", -10.0, 0.7,    -0.6,    1.2,    -0.1,   6.0,      -7.0)

    def test_TC_90064(self):
        tc_no = 90064
        #setLimit(tself, motor, tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, self.motor, tc_no, "LLM", -10.0,  0.7,    -0.6,    1.2,    -0.1,   6.0,      -7.0)

    #Invert dir
    def test_TC_90070(self):
        tc_no = 90070
        encRel = 0
        #                                       mres, dir,off, hlm, expHLM, expM3rhlm, expLLM, expM3rllm)
        motorInitVeloAcc(self, self.motor, tc_no, encRel)
        mres = 0.1
        dir = 1
        off = 0.5
        setMresDirOff(self, self.motor, tc_no, mres, dir, off)
        motorInitLimitsWithC(self, self.motor, tc_no)

    def test_TC_90071(self):
        tc_no = 90071
        #setLimit(tself, motor, tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, self.motor, tc_no, "DHLM", 10,  0.6,     -0.7,     1.2,    -0.1,    6.0,      -7.0)

    def test_TC_90072(self):
        tc_no = 90072
        #setLimit(tself, motor, tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, self.motor, tc_no, "HLM",  10,   0.6,     -0.7,     1.2,     -0.1,    6.0,      -7.0)

    def test_TC_90073(self):
        tc_no = 90073
        #setLimit(tself, motor, tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, self.motor, tc_no, "DLLM", -10.0, 0.6,     -0.7,     1.2,     -0.1,    6.0,      -7.0)

    def test_TC_90074(self):
        tc_no = 90074
        #setLimit(tself, motor, tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, self.motor, tc_no, "LLM", -10.0,  0.6,     -0.7,     1.2,     -0.1,    6.0,      -7.0)

    #Invert MRES and dir
    def test_TC_90080(self):
        tc_no = 90080
        encRel = 0
        #                                       mres, dir,off, hlm, expHLM, expM3rhlm, expLLM, expM3rllm)
        motorInitVeloAcc(self, self.motor, tc_no, encRel)
        mres = -0.1
        dir = 1
        off = 0.5
        setMresDirOff(self, self.motor, tc_no, mres, dir, off)
        motorInitLimitsWithC(self, self.motor, tc_no)

    def test_TC_90081(self):
        tc_no = 90081
        #setLimit(tself, motor, tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, self.motor, tc_no, "DHLM", 10,  0.7,      -0.6,    1.1,    -0.2,   6.0,      -7.0)

    def test_TC_90082(self):
        tc_no = 90082
        #setLimit(tself, motor, tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, self.motor, tc_no, "HLM",  10,  0.7,     -0.6,    1.1,    -0.2,    6.0,      -7.0)

    def test_TC_90083(self):
        tc_no = 90083
        #setLimit(tself, motor, tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, self.motor, tc_no, "DLLM", -10, 0.7,     -0.6,    1.1,    -0.2,    6.0,      -7.0)

    def test_TC_90084(self):
        tc_no = 90084
        #setLimit(tself, motor, tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, self.motor, tc_no, "LLM", -10,  0.7,     -0.6,    1.1,    -0.2,    6.0,      -7.0)

