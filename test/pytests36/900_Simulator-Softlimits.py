#!/usr/bin/env python
#

import datetime
import unittest
import os
import sys
from AxisMr import AxisMr
from AxisCom import AxisCom

import time
import math
import inspect

filnam = "900xx.py"
###


def lineno():
    return inspect.currentframe().f_back.f_lineno


polltime = 0.1

# Values to be used for test
# Note: Make sure to use different values to hae a good
# test coverage
myVELO = 1.0  # positioning velocity
myACCL = 1.0  # Time to VELO, seconds
# myAR   = myVELO / myACCL # acceleration, mm/sec^2

myJVEL = 5.0  # Jogging velocity
myJAR = 6.0  # Jogging acceleration, mm/sec^2

myBVEL = 2.0  # backlash velocity
myBACC = 1.5  # backlash acceleration, seconds
myBAR = myBVEL / myBACC  # backlash acceleration, mm/sec^2
myBDST = 0  # backlash destination, mm

# Different values, high use even, low uses odd
#
myLowHardLimitPos = -9.0
myCfgDLLM = -7.0
myDLLM = -5.0
myStartposDial = 0.0
myDHLM = 4.0
myCfgDHLM = 6.0
myHighHardLimitPos = 8.0

# Comparing floating points may fail because of rounding problems
maxdelta = 0.01

# We need to test different combinations of
# - MRES > 0       ; MRES < 0
# DIR=0; OFF=X     ; DIR=1; OFF=X
# (Those above are the main loop)
#
#
# Controller with and without "read only limits"
# Write to DHLM, DLLM, HLM, LLM


def InitVeloAcc(self, tc_no, encRel):
    msta = int(self.axisCom.get(".MSTA"))
    assert msta & self.axisMr.MSTA_BIT_HOMED  # , 'MSTA.homed (Axis has been homed)')

    # Prepare parameters for jogging and backlash
    self.axisCom.put(".VELO", myVELO)
    self.axisCom.put(".ACCL", myACCL)

    self.axisCom.put(".JVEL", myJVEL)
    self.axisCom.put(".JAR", myJAR)

    self.axisCom.put(".BVEL", myBVEL)
    self.axisCom.put(".BACC", myBACC)
    self.axisCom.put(".BDST", myBDST)
    self.axisCom.put(".UEIP", encRel)
    # Move the  to 0, to avoid limit switch activation
    self.axisCom.put(".DVAL", myStartposDial)
    # Speed it up, by setting the position in the simulator
    self.axisMr.setValueOnSimulator(tc_no, "fActPosition", myStartposDial)
    # and wait for the movement to finish
    self.axisMr.waitForStop(tc_no, 2.0)


def InitLimitsNoROlimits(self, tc_no):
    self.axisMr.setValueOnSimulator(tc_no, "fHighSoftLimitPos", myCfgDHLM)
    self.axisMr.setValueOnSimulator(tc_no, "fLowSoftLimitPos", myCfgDLLM)
    self.axisCom.put("-CfgDLLM-En", 0, wait=True)
    self.axisCom.put("-CfgDHLM-En", 0, wait=True)

    ## XXX self.axisCom.put("-DbgStrToLOG", "initLim " + str(tc_no)[0:20], wait=True)
    maxTime = 5  # 5 seconds maximum to let read only parameters ripple through
    maxDelta = 0.05  # 5 % error tolerance margin
    while maxTime > 0:
        self.axisCom.put(".DHLM", myDHLM)
        self.axisCom.put(".DLLM", myDLLM)

        actDHLM = self.axisCom.get(".DHLM")
        actDLLM = self.axisCom.get(".DLLM")

        print(
            "%s:%d expDHLM=%f actDHLM=%f expDLLM=%f actDLLM=%f"
            % (tc_no, lineno(), myDHLM, actDHLM, myDLLM, actDLLM)
        )

        resH = self.axisMr.calcAlmostEqual(tc_no, myDHLM, actDHLM, maxDelta)
        resL = self.axisMr.calcAlmostEqual(tc_no, myDLLM, actDLLM, maxDelta)
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no}:{int(lineno())} resH={resH} resL={resL}"
        )
        if (resH == True) and (resL == True):
            return True

        time.sleep(polltime)
        maxTime = maxTime - polltime
    return False


def InitLimitsWithROlimits(self, tc_no):
    # Depending on the value of MRES, and its sign (!)
    # we need to calculate the expected values for DHLM/DLLM
    # in the record may be swapped (e.g CfgDLLM -> DHLM)
    mres = self.axisCom.get(".MRES")
    if mres < 0:
        # Swap High and low
        expDHLM = myCfgDLLM * mres
        expDLLM = myCfgDHLM * mres
    else:
        expDHLM = myCfgDHLM * mres
        expDLLM = myCfgDLLM * mres

    ## XXX self.axisCom.put("-DbgStrToLOG", "initLim " + str(tc_no)[0:20], wait=True)
    maxTime = 5  # 5 seconds maximum to let read only parameters ripple through
    maxDelta = 0.05  # 5 % error tolerance margin
    self.axisCom.put("-CfgDLLM-En", 0, wait=True)
    self.axisCom.put("-CfgDHLM-En", 0, wait=True)
    self.axisMr.setValueOnSimulator(tc_no, "fHighSoftLimitPos", myCfgDHLM)
    self.axisMr.setValueOnSimulator(tc_no, "fLowSoftLimitPos", myCfgDLLM)
    self.axisCom.put("-CfgDHLM", myCfgDHLM)
    self.axisCom.put("-CfgDLLM", myCfgDLLM)
    self.axisCom.put("-CfgDLLM-En", 1, wait=True)
    self.axisCom.put("-CfgDHLM-En", 1, wait=True)

    # Wait until ".DHLM" and ".DLLM" have rippled through the poller
    # and the processing in the motorRecord
    while maxTime > 0:
        actDHLM = self.axisCom.get(".DHLM", myDHLM)
        actDLLM = self.axisCom.get(".DLLM", myDLLM)

        debug_text = f"{tc_no}:{lineno()} expDHLM={expDHLM} actDHLM={actDHLM} expDLLM={expDLLM} actDLLM={actDLLM} mres={mres}"
        print(debug_text)

        resH = self.axisMr.calcAlmostEqual(tc_no, expDHLM, actDHLM, maxDelta)
        resL = self.axisMr.calcAlmostEqual(tc_no, expDLLM, actDLLM, maxDelta)
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no}:{int(lineno())} resH={resH} resL={resL}"
        )
        if (resH == True) and (resL == True):
            return
        time.sleep(polltime)
        maxTime = maxTime - polltime

    raise Exception(debug_text)
    assert False


def setMresDirOff(self, tc_no, mres, dir, off):
    self.axisCom.put(".MRES", mres)
    self.axisCom.put(".DIR", dir)
    self.axisCom.put(".OFF", off)


def readBackParamVerify(self, tc_no, field_name, expVal):
    maxTime = 5  # 5 seconds maximum to poll all parameters
    testPassed = False
    maxDelta = math.fabs(expVal) * 0.02  # 2 % error tolerance margin
    while maxTime > 0:
        actVal = self.axisCom.get(field_name)
        print(
            f"{tc_no}:{int(lineno())} {field_name} expVal={expVal:f} actVal={actVal:f}"
        )

        res = self.axisMr.calcAlmostEqual(tc_no, expVal, actVal, maxDelta)
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no}:{int(lineno())} res={res}"
        )
        if (res == True) or (res != 0):
            return True
        else:
            time.sleep(polltime)
            maxTime = maxTime - polltime
    return False


def setLimit(
    self,
    tc_no,
    field,
    value,
    expDHLM,
    expDLLM,
    expHLM,
    expLLM,
    expM3rhlm,
    expM3rllm,
):
    self.axisCom.put("-DbgStrToLOG", "Start " + str(tc_no), wait=True)
    oldDHLM = self.axisCom.get(".DHLM", use_monitor=False)
    oldDLLM = self.axisCom.get(".DLLM", use_monitor=False)

    self.axisCom.put(".DHLM", oldDHLM)
    self.axisCom.put(".DLLM", oldDLLM)
    self.axisCom.put("." + field, value)

    okDHLM = readBackParamVerify(self, tc_no, ".DHLM", expDHLM)
    okDLLM = readBackParamVerify(self, tc_no, ".DLLM", expDLLM)

    okHLM = readBackParamVerify(self, tc_no, ".HLM", expHLM)
    okLLM = readBackParamVerify(self, tc_no, ".LLM", expLLM)

    okM3rhlm = readBackParamVerify(self, tc_no, "-M3RHLM", expM3rhlm)
    okM3rllm = readBackParamVerify(self, tc_no, "-M3RLLM", expM3rllm)

    testPassed = okDHLM and okDLLM and okHLM and okLLM and okM3rhlm and okM3rllm
    if testPassed:
        self.axisCom.put("-DbgStrToLOG", "Passed " + str(tc_no), wait=True)
    else:
        self.axisCom.put("-DbgStrToLOG", "Failed " + str(tc_no), wait=True)
    assert testPassed


class Test(unittest.TestCase):
    hasROlimit = 0
    drvUseEGU_RB = None
    drvUseEGU = 0
    url_string = os.getenv("TESTEDMOTORAXIS")
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} url_string={url_string}"
    )

    axisCom = AxisCom(url_string, log_debug=False)
    axisMr = AxisMr(axisCom)

    # self.axisCom.put('-DbgStrToLOG', "Start " + os.path.basename(__file__)[0:20], wait=True)
    vers = float(axisCom.get(".VERS"))
    if vers >= 6.94 and vers <= 7.09:
        hasROlimit = 1
        drvUseEGU_RB = axisCom.get("-DrvUseEGU-RB")
        drvUseEGU = 0
        if drvUseEGU_RB == 1:
            axisCom.put("-DrvUseEGU", drvUseEGU)
            # drvUseEGU = self.axisCom.get('-DrvUseEGU-RB')
        axisCom.put(".SPAM", 255)

    def test_TC_90000(self):
        tc_no = 90000
        self.axisCom.put("-DbgStrToLOG", "Start " + str(tc_no), wait=True)
        self.axisMr.motorInitAllForBDST(tc_no)
        self.axisCom.put("-DbgStrToLOG", "End " + str(tc_no), wait=True)

    def test_TC_90010(self):
        tc_no = 90010
        encRel = 0
        self.axisCom.put("-DbgStrToLOG", "Start " + str(tc_no), wait=True)
        # vers = float(self.axisCom.get("(self. + '.VERS'))
        # print('%s vers=%g' %  (tc_no, vers))
        # self.assertEqual(0, 1, '1 != 0')
        testPassed = readBackParamVerify(self, tc_no, "-DrvUseEGU-RB", 0)

        InitVeloAcc(self, tc_no, encRel)
        mres = 0.1
        dir = 0
        off = 0.5
        setMresDirOff(self, tc_no, mres, dir, off)
        InitLimitsNoROlimits(self, tc_no)
        if testPassed:
            self.axisCom.put("-DbgStrToLOG", "Passed " + str(tc_no), wait=True)
        else:
            self.axisCom.put("-DbgStrToLOG", "Failed " + str(tc_no), wait=True)
        assert testPassed

    def test_TC_90011(self):
        tc_no = 90011
        # setLimit(self,  tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, tc_no, "DHLM", 4.1, 4.1, -5.0, 4.6, -4.5, 41.0, -50.0)

    def test_TC_90012(self):
        tc_no = 90012
        # setLimit(self,  tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, tc_no, "HLM", 4.7, 4.2, -5.0, 4.7, -4.5, 42.0, -50.0)

    def test_TC_90013(self):
        tc_no = 90013
        # setLimit(self,  tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, tc_no, "DLLM", -5.3, 4.2, -5.3, 4.7, -4.8, 42.0, -53.0)

    def test_TC_90014(self):
        tc_no = 90014
        # setLimit(self,  tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, tc_no, "LLM", -5.4, 4.2, -5.9, 4.7, -5.4, 42.0, -59.0)

    ###################################################################################################################
    # Invert mres
    def test_TC_90020(self):
        tc_no = 90020
        self.axisCom.put("-DbgStrToLOG", "Start " + str(tc_no), wait=True)
        encRel = 0
        #                                       mres, dir,off, hlm, expHLM, expM3rhlm, expLLM, expM3rllm)
        InitVeloAcc(self, tc_no, encRel)
        mres = -0.1
        dir = 0
        off = 0.5
        setMresDirOff(self, tc_no, mres, dir, off)
        InitLimitsNoROlimits(self, tc_no)
        self.axisCom.put("-DbgStrToLOG", "End " + str(tc_no), wait=True)

    def test_TC_90021(self):
        tc_no = 90021
        # setLimit(self,  tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, tc_no, "DHLM", 4.1, 4.1, -5.0, 4.6, -4.5, 50.0, -41.0)

    def test_TC_90022(self):
        tc_no = 90022
        # setLimit(self,  tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, tc_no, "HLM", 4.7, 4.2, -5.0, 4.7, -4.5, 50.0, -42.0)

    def test_TC_90023(self):
        tc_no = 90023
        # setLimit(self,  tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, tc_no, "DLLM", -5.3, 4.2, -5.3, 4.7, -4.8, 53.0, -42.0)

    def test_TC_90024(self):
        tc_no = 90024
        # setLimit(self,  tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, tc_no, "LLM", -5.4, 4.2, -5.9, 4.7, -5.4, 59.0, -42.0)

    ###################################################################################################################
    # Invert dir
    def test_TC_90030(self):
        tc_no = 90030
        encRel = 0
        self.axisCom.put("-DbgStrToLOG", "Start " + str(tc_no), wait=True)
        #                                       mres, dir,off, hlm, expHLM, expM3rhlm, expLLM, expM3rllm)
        InitVeloAcc(self, tc_no, encRel)
        mres = 0.1
        dir = 1
        off = 0.5
        setMresDirOff(self, tc_no, mres, dir, off)
        InitLimitsNoROlimits(self, tc_no)
        self.axisCom.put("-DbgStrToLOG", "End " + str(tc_no), wait=True)

    def test_TC_90031(self):
        tc_no = 90031
        # setLimit(self,  tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, tc_no, "DHLM", 4.1, 4.1, -5.0, 5.5, -3.6, 41.0, -50.0)

    def test_TC_90032(self):
        tc_no = 90032
        # setLimit(self,  tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, tc_no, "HLM", 4.7, 4.1, -4.2, 4.7, -3.6, 41.0, -42.0)

    def test_TC_90033(self):
        tc_no = 90033
        # setLimit(self,  tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, tc_no, "DLLM", -5.3, 4.1, -5.3, 5.8, -3.6, 41.0, -53.0)

    def test_TC_90034(self):
        tc_no = 90034
        # setLimit(self,  tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, tc_no, "LLM", -5.4, 5.9, -5.3, 5.8, -5.4, 59.0, -53.0)

    ###################################################################################################################
    # Invert mres, invert dir
    def test_TC_90040(self):
        tc_no = 90040
        encRel = 0
        self.axisCom.put("-DbgStrToLOG", "Start " + str(tc_no), wait=True)
        #                                       mres, dir,off, hlm, expHLM, expM3rhlm, expLLM, expM3rllm)
        InitVeloAcc(self, tc_no, encRel)
        mres = -0.1
        dir = 1
        off = 0.5
        setMresDirOff(self, tc_no, mres, dir, off)
        InitLimitsNoROlimits(self, tc_no)
        self.axisCom.put("-DbgStrToLOG", "End " + str(tc_no), wait=True)

    def test_TC_90041(self):
        tc_no = 90041
        # setLimit(self,  tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, tc_no, "DHLM", 4.1, 4.1, -5.0, 5.5, -3.6, 50.0, -41.0)

    def test_TC_90042(self):
        tc_no = 90042
        # setLimit(self,  tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, tc_no, "HLM", 4.7, 4.1, -4.2, 4.7, -3.6, 42.0, -41.0)

    def test_TC_90043(self):
        tc_no = 90043
        # setLimit(self,  tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, tc_no, "DLLM", -5.3, 4.1, -5.3, 5.8, -3.6, 53.0, -41.0)

    def test_TC_90044(self):
        tc_no = 90044
        # setLimit(self,  tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, tc_no, "LLM", -5.4, 5.9, -5.3, 5.8, -5.4, 53.0, -59.0)

    #########################################################################################################
    # Test clipping
    def test_TC_90050(self):
        tc_no = 90050
        encRel = 0
        self.axisCom.put("-DbgStrToLOG", "Start " + str(tc_no), wait=True)

        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no} vers={self.vers:g} hasROlimit={int(self.hasROlimit)}"
        )
        self.assertEqual(1, self.hasROlimit, "motorRecord supports RO soft limits")

        #                                       mres, dir,off, hlm, expHLM, expM3rhlm, expLLM, expM3rllm)
        InitVeloAcc(self, tc_no, encRel)
        mres = 0.1
        dir = 0
        off = 0.5
        setMresDirOff(self, tc_no, mres, dir, off)
        InitLimitsWithROlimits(self, tc_no)
        self.axisCom.put("-DbgStrToLOG", "End " + str(tc_no), wait=True)

    def test_TC_90051(self):
        self.assertEqual(1, self.hasROlimit, "motorRecord supports RO soft limits")
        tc_no = 90051
        # setLimit(self,  tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, tc_no, "DHLM", 10, 0.6, -0.7, 1.1, -0.2, 6.0, -7.0)

    def test_TC_90052(self):
        self.assertEqual(1, self.hasROlimit, "motorRecord supports RO soft limits")
        tc_no = 90052
        # setLimit(self,  tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, tc_no, "HLM", 10, 0.6, -0.7, 1.1, -0.2, 6.0, -7.0)

    def test_TC_90053(self):
        self.assertEqual(1, self.hasROlimit, "motorRecord supports RO soft limits")
        tc_no = 90053
        # setLimit(self,  tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, tc_no, "DLLM", -10.0, 0.6, -0.7, 1.1, -0.2, 6.0, -7.0)

    def test_TC_90054(self):
        self.assertEqual(1, self.hasROlimit, "motorRecord supports RO soft limits")
        tc_no = 90054
        # setLimit(self,  tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, tc_no, "LLM", -10.0, 0.6, -0.7, 1.1, -0.2, 6.0, -7.0)

    # Invert mres
    def test_TC_90060(self):
        self.assertEqual(1, self.hasROlimit, "motorRecord supports RO soft limits")
        tc_no = 90060
        self.axisCom.put("-DbgStrToLOG", "Start " + str(tc_no), wait=True)
        encRel = 0

        InitVeloAcc(self, tc_no, encRel)
        mres = -0.1
        dir = 0
        off = 0.5
        setMresDirOff(self, tc_no, mres, dir, off)
        InitLimitsWithROlimits(self, tc_no)
        self.axisCom.put("-DbgStrToLOG", "End " + str(tc_no), wait=True)

    def test_TC_90061(self):
        self.assertEqual(1, self.hasROlimit, "motorRecord supports RO soft limits")
        tc_no = 90061
        # setLimit(self,  tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, tc_no, "DHLM", 10, 0.7, -0.6, 1.2, -0.1, 6.0, -7.0)

    def test_TC_90062(self):
        self.assertEqual(1, self.hasROlimit, "motorRecord supports RO soft limits")
        tc_no = 90062
        # setLimit(self,  tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, tc_no, "HLM", 10, 0.7, -0.6, 1.2, -0.1, 6.0, -7.0)

    def test_TC_90063(self):
        self.assertEqual(1, self.hasROlimit, "motorRecord supports RO soft limits")
        tc_no = 90063
        # setLimit(self,  tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, tc_no, "DLLM", -10.0, 0.7, -0.6, 1.2, -0.1, 6.0, -7.0)

    def test_TC_90064(self):
        self.assertEqual(1, self.hasROlimit, "motorRecord supports RO soft limits")
        tc_no = 90064
        # setLimit(self,  tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, tc_no, "LLM", -10.0, 0.7, -0.6, 1.2, -0.1, 6.0, -7.0)

    # Invert dir
    def test_TC_90070(self):
        self.assertEqual(1, self.hasROlimit, "motorRecord supports RO soft limits")
        tc_no = 90070
        encRel = 0
        self.axisCom.put("-DbgStrToLOG", "Start " + str(tc_no), wait=True)
        #                                       mres, dir,off, hlm, expHLM, expM3rhlm, expLLM, expM3rllm)
        InitVeloAcc(self, tc_no, encRel)
        mres = 0.1
        dir = 1
        off = 0.5
        setMresDirOff(self, tc_no, mres, dir, off)
        InitLimitsWithROlimits(self, tc_no)
        self.axisCom.put("-DbgStrToLOG", "End " + str(tc_no), wait=True)

    def test_TC_90071(self):
        self.assertEqual(1, self.hasROlimit, "motorRecord supports RO soft limits")
        tc_no = 90071
        # setLimit(self,  tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, tc_no, "DHLM", 10, 0.6, -0.7, 1.2, -0.1, 6.0, -7.0)

    def test_TC_90072(self):
        self.assertEqual(1, self.hasROlimit, "motorRecord supports RO soft limits")
        tc_no = 90072
        # setLimit(self,  tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, tc_no, "HLM", 10, 0.6, -0.7, 1.2, -0.1, 6.0, -7.0)

    def test_TC_90073(self):
        self.assertEqual(1, self.hasROlimit, "motorRecord supports RO soft limits")
        tc_no = 90073
        # setLimit(self,  tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, tc_no, "DLLM", -10.0, 0.6, -0.7, 1.2, -0.1, 6.0, -7.0)

    def test_TC_90074(self):
        self.assertEqual(1, self.hasROlimit, "motorRecord supports RO soft limits")
        tc_no = 90074
        # setLimit(self,  tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, tc_no, "LLM", -10.0, 0.6, -0.7, 1.2, -0.1, 6.0, -7.0)

    # Invert MRES and dir
    def test_TC_90080(self):
        self.assertEqual(1, self.hasROlimit, "motorRecord supports RO soft limits")
        tc_no = 90080
        self.axisCom.put("-DbgStrToLOG", "Start " + str(tc_no), wait=True)
        encRel = 0
        #                                       mres, dir,off, hlm, expHLM, expM3rhlm, expLLM, expM3rllm)
        InitVeloAcc(self, tc_no, encRel)
        mres = -0.1
        dir = 1
        off = 0.5
        setMresDirOff(self, tc_no, mres, dir, off)
        InitLimitsWithROlimits(self, tc_no)
        self.axisCom.put("-DbgStrToLOG", "End " + str(tc_no), wait=True)

    def test_TC_90081(self):
        self.assertEqual(1, self.hasROlimit, "motorRecord supports RO soft limits")
        tc_no = 90081
        # setLimit(self,  tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, tc_no, "DHLM", 10, 0.7, -0.6, 1.1, -0.2, 6.0, -7.0)

    def test_TC_90082(self):
        self.assertEqual(1, self.hasROlimit, "motorRecord supports RO soft limits")
        tc_no = 90082
        # setLimit(self,  tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, tc_no, "HLM", 10, 0.7, -0.6, 1.1, -0.2, 6.0, -7.0)

    def test_TC_90083(self):
        self.assertEqual(1, self.hasROlimit, "motorRecord supports RO soft limits")
        tc_no = 90083
        # setLimit(self,  tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, tc_no, "DLLM", -10, 0.7, -0.6, 1.1, -0.2, 6.0, -7.0)

    def test_TC_90084(self):
        self.assertEqual(1, self.hasROlimit, "motorRecord supports RO soft limits")
        tc_no = 90084
        # setLimit(self,  tc_no,    field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimit(self, tc_no, "LLM", -10, 0.7, -0.6, 1.1, -0.2, 6.0, -7.0)

    def test_TC_900999(self):
        if self.drvUseEGU_RB == 1:
            self.axisCom.put("-DrvUseEGU", 1)
