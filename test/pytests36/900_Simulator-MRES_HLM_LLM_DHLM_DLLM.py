#!/usr/bin/env python
#

import datetime
import inspect
import unittest
import os
from AxisMr import AxisMr
from AxisCom import AxisCom

import time
import math

filnam = os.path.basename(__file__)[0:3]
###

polltime = 0.1


def lineno():
    return inspect.currentframe().f_back.f_lineno


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


def readDebugPrintLimits(self, tc_no, lineno=0):
    fnam = "readDebugPrintLimits"
    mres = float(self.axisCom.get(".MRES"))
    dir = int(self.axisCom.get(".DIR"))
    off = float(self.axisCom.get(".OFF"))
    M3RHLM = float(self.axisCom.get("-M3RHLM"))
    M3RLLM = float(self.axisCom.get("-M3RLLM"))
    DHLM = float(self.axisCom.get(".DHLM"))
    DLLM = float(self.axisCom.get(".DLLM"))
    HLM = float(self.axisCom.get(".HLM"))
    LLM = float(self.axisCom.get(".LLM"))
    if self.hasRhlmRllm:
        RHLM = float(self.axisCom.get(".RHLM"))
        RLLM = float(self.axisCom.get(".RLLM"))
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}/{fnam} {tc_no}:{int(lineno)} mres={mres} dir={dir!r} off={off!r} RHLM={RHLM:.2f} RLLM={RLLM:.2f} M3RHLM={M3RHLM:.2f} M3RLLM={M3RLLM:.2f} DHLM={DHLM:.2f} DLLM={DLLM:.2f} HLM={HLM:.2f} LLM={LLM:.2f}"
        )
    else:
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}/{fnam} {tc_no}:{int(lineno)} mres={mres} dir={dir!r} off={off!r} M3RHLM={M3RHLM:.2f} M3RLLM={M3RLLM:.2f} DHLM={DHLM:.2f} DLLM={DLLM:.2f} HLM={HLM:.2f} LLM={LLM:.2f}"
        )


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

    ## XXX self.axisCom.putDbgStrToLOG("initLim " + str(tc_no)[0:20], wait=True)
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
        if resH and resL:
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

    ## XXX self.axisCom.putDbgStrToLOG("initLim " + str(tc_no)[0:20], wait=True)
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
        actDHLM = self.axisCom.get(".DHLM")
        actDLLM = self.axisCom.get(".DLLM")

        debug_text = f"{tc_no}:{lineno()} expDHLM={expDHLM} actDHLM={actDHLM} expDLLM={expDLLM} actDLLM={actDLLM} mres={mres}"
        print(debug_text)

        resH = self.axisMr.calcAlmostEqual(tc_no, expDHLM, actDHLM, maxDelta)
        resL = self.axisMr.calcAlmostEqual(tc_no, expDLLM, actDLLM, maxDelta)
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no}:{int(lineno())} resH={resH} resL={resL}"
        )
        if resH and resL:
            return
        time.sleep(polltime)
        maxTime = maxTime - polltime

    raise Exception(debug_text)
    assert False


def setMresDirOff(self, tc_no, mres, dir, off):
    field_name = ".MRES"
    value = mres
    self.axisCom.put(field_name, value)
    readBackParamVerify(self, tc_no, field_name, value)

    field_name = ".DIR"
    value = dir
    self.axisCom.put(field_name, value)
    readBackParamVerify(self, tc_no, field_name, value)

    field_name = ".OFF"
    value = off
    self.axisCom.put(field_name, value)
    readBackParamVerify(self, tc_no, field_name, value)


def readBackParamVerify(self, tc_no, field_name, expVal):
    maxTime = 5  # 5 seconds maximum to poll all parameters
    maxDelta = math.fabs(expVal) * 0.02  # 2 % error tolerance margin
    while maxTime > 0:
        actVal = self.axisCom.get(field_name, use_monitor=False)
        res = self.axisMr.calcAlmostEqual(
            tc_no, expVal, actVal, maxDelta, doPrint=False
        )
        if res or (res != 0):
            # self.axisMr.calcAlmostEqual(tc_no, expVal, actVal, maxDelta, doPrint=True)
            print(
                f"{tc_no}:{int(lineno())} {field_name} expVal={expVal:f} actVal={actVal:f} inrange=True"
            )
            return True
        else:
            time.sleep(polltime)
            maxTime = maxTime - polltime
    print(
        f"{tc_no}:{int(lineno())} {field_name} expVal={expVal:f} actVal={actVal:f} inrange=False"
    )
    return False


def setLimitCheckResult(
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
    self.axisCom.putDbgStrToLOG("Start " + str(tc_no), wait=True)
    mres0 = float(self.axisCom.get(".MRES"))
    dir0 = int(self.axisCom.get(".DIR"))
    off0 = float(self.axisCom.get(".OFF"))
    m3rhlm = float(self.axisCom.get("-M3RHLM"))
    m3rllm = float(self.axisCom.get("-M3RLLM"))
    dhlm = float(self.axisCom.get(".DHLM"))
    dllm = float(self.axisCom.get(".DLLM"))
    hlm = float(self.axisCom.get(".HLM"))
    llm = float(self.axisCom.get(".LLM"))
    if self.hasRhlmRllm:
        rhlm = float(self.axisCom.get(".RHLM"))
        rllm = float(self.axisCom.get(".RLLM"))
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no}:{int(lineno())} mres0={mres0} dir0={dir0!r} off0={off0!r} rhlm={rhlm:.2f} rllm={rllm:.2f} m3rhlm={m3rhlm:.2f} m3rllm={m3rllm:.2f} dhlm={dhlm:.2f} dllm={dllm:.2f} hlm={hlm:.2f} llm={llm:.2f}"
        )
    else:
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no}:{int(lineno())} mres0={mres0} dir0={dir0!r} off0={off0!r} m3rhlm={m3rhlm:.2f} m3rllm={m3rllm:.2f} dhlm={dhlm:.2f} dllm={dllm:.2f} hlm={hlm:.2f} llm={llm:.2f}"
        )

    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no}:{int(lineno())} mres0={mres0} dir0={dir0} off0={off0!r} expDHLM={expDHLM:.2f} expDLLM={expDLLM:.2f} expHLM={expHLM:.2f} expLLM={expLLM:.2f} expM3rhlm={expM3rhlm:.2f} expM3rhlm={expM3rhlm:.2f}"
    )

    self.axisCom.put("." + field, value)

    okDHLM = readBackParamVerify(self, tc_no, ".DHLM", expDHLM)
    okDLLM = readBackParamVerify(self, tc_no, ".DLLM", expDLLM)

    okHLM = readBackParamVerify(self, tc_no, ".HLM", expHLM)
    okLLM = readBackParamVerify(self, tc_no, ".LLM", expLLM)

    okM3rhlm = readBackParamVerify(self, tc_no, "-M3RHLM", expM3rhlm)
    okM3rllm = readBackParamVerify(self, tc_no, "-M3RLLM", expM3rllm)

    testPassed = okDHLM and okDLLM and okHLM and okLLM and okM3rhlm and okM3rllm
    if self.hasRhlmRllm:
        okM0rhlm = readBackParamVerify(self, tc_no, ".RHLM", expM3rhlm)
        okM0rllm = readBackParamVerify(self, tc_no, ".RLLM", expM3rllm)
        testPassed = testPassed and okM0rhlm and okM0rllm

    if testPassed:
        self.axisCom.putDbgStrToLOG("Passed " + str(tc_no), wait=True)
    else:
        DHLM = self.axisCom.get(".DHLM")
        DLLM = self.axisCom.get(".DLLM")
        HLM = self.axisCom.get(".HLM")
        LLM = self.axisCom.get(".LLM")
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no}:{int(lineno())} HLM={HLM!r} LLM={LLM!r} DHLM={DHLM!r} DLLM={DLLM!r}"
        )
        self.axisCom.putDbgStrToLOG("Failed " + str(tc_no), wait=True)
    assert testPassed


def setLimitWrapper(
    self,
    tc_no,
    field,
    value,
    expDHLM0,
    expDLLM0,
    expHLM0,
    expLLM0,
    expM3rhlm0,
    expM3rllm0,
):
    mres0 = float(self.axisCom.get(".MRES"))
    dir0 = int(self.axisCom.get(".DIR"))
    off0 = float(self.axisCom.get(".OFF"))
    tc_no = tc_no + 1
    expDHLM = expDHLM0
    expDLLM = expDLLM0
    expHLM = expHLM0
    expLLM = expLLM0
    expM3rhlm = expM3rhlm0
    expM3rllm = expM3rllm0
    setLimitCheckResult(
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
    )
    if not self.hasRhlmRllm:
        return
    tc_no = tc_no + 1
    factor = 2
    mres = mres0 * factor
    expDHLM = expDHLM0 * factor
    expDLLM = expDLLM0 * factor

    # From DIAL to USER coordinates with DIR and OFF
    if dir0 == 0:
        expHLM = expDHLM + off0
        expLLM = expDLLM + off0
    elif dir0 == 1:
        expHLM = 0 - expDLLM + off0
        expLLM = 0 - expDHLM + off0
    else:
        expHLM = None
        expLLM = None
    setLimitCheckResult(
        self,
        tc_no,
        "MRES",
        mres,
        expDHLM,
        expDLLM,
        expHLM,
        expLLM,
        expM3rhlm,
        expM3rllm,
    )

    self.axisCom.put(".MRES", mres0)
    time.sleep(2.0)


class Test(unittest.TestCase):
    tc_no = 900000
    hasROlimit = False
    hasRhlmRllm = False
    drvUseEGU_RB = None
    drvUseEGU = 0
    url_string = os.getenv("TESTEDMOTORAXIS")
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} url_string={url_string}"
    )

    axisCom = AxisCom(url_string, log_debug=False)
    axisMr = AxisMr(axisCom)

    # self.axisCom.put('-DbgStrToLOG', "Start " + os.path.basename(__file__)[0:20], wait=True)
    oldSPAM = axisMr.getFieldSPAM(tc_no)
    axisMr.setFieldSPAM(tc_no, 255)
    hasROlimit = axisMr.hasROlimit
    if hasROlimit:
        drvUseEGU_RB = axisCom.get("-DrvUseEGU-RB")
        drvUseEGU = 0
        if drvUseEGU_RB == 1:
            axisCom.put("-DrvUseEGU", drvUseEGU)
            # drvUseEGU = self.axisCom.get('-DrvUseEGU-RB')

    try:
        floatGet = axisCom.get(".RHLM")
        hasRhlmRllm = True
    except Exception:
        hasRhlmRllm = False

    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} hasROlimit={hasROlimit} hasROlimit={int(hasROlimit)} hasRhlmRllm={hasRhlmRllm}"
    )

    def test_TC_900010(self):
        tc_no = 900010
        self.axisCom.putDbgStrToLOG("Start " + str(tc_no), wait=True)
        self.axisMr.motorInitAllForBDST(tc_no)
        self.axisCom.putDbgStrToLOG("End " + str(tc_no), wait=True)

    def test_TC_900100(self):
        tc_no = 900100
        encRel = 0
        self.axisCom.putDbgStrToLOG("Start " + str(tc_no), wait=True)
        testPassed = readBackParamVerify(self, tc_no, "-DrvUseEGU-RB", 0)

        InitVeloAcc(self, tc_no, encRel)
        mres = 0.1
        dir = 0
        off = 0.5
        setMresDirOff(self, tc_no, mres, dir, off)
        InitLimitsNoROlimits(self, tc_no)
        if testPassed:
            self.axisCom.putDbgStrToLOG("Passed " + str(tc_no), wait=True)
        else:
            self.axisCom.putDbgStrToLOG("Failed " + str(tc_no), wait=True)
        assert testPassed

    def test_TC_900110(self):
        tc_no = 900110
        # setLimitWrapper(self,tc_no,field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimitWrapper(self, tc_no, "DHLM", 4.1, 4.1, -5.0, 4.6, -4.5, 41.0, -50.0)

    def test_TC_900120(self):
        tc_no = 900120
        # setLimitWrapper(self,tc_no,field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimitWrapper(self, tc_no, "HLM", 4.7, 4.2, -5.0, 4.7, -4.5, 42.0, -50.0)

    def test_TC_900130(self):
        tc_no = 900130
        # setLimitWrapper(self,tc_no,field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimitWrapper(self, tc_no, "DLLM", -5.3, 4.2, -5.3, 4.7, -4.8, 42.0, -53.0)

    def test_TC_900140(self):
        tc_no = 900140
        # setLimitWrapper(self,tc_no,field, val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimitWrapper(self, tc_no, "LLM", -5.4, 4.2, -5.9, 4.7, -5.4, 42.0, -59.0)

    ###################################################################################################################
    # Invert mres
    def test_TC_900200(self):
        tc_no = 900200
        self.axisCom.putDbgStrToLOG("Start " + str(tc_no), wait=True)
        encRel = 0
        InitVeloAcc(self, tc_no, encRel)
        mres = -0.1
        dir = 0
        off = 0.5
        setMresDirOff(self, tc_no, mres, dir, off)
        InitLimitsNoROlimits(self, tc_no)
        readDebugPrintLimits(self, tc_no, lineno=lineno())
        self.axisCom.putDbgStrToLOG("End " + str(tc_no), wait=True)

    def test_TC_900210(self):
        tc_no = 900210
        # setLimitWrapper(self, tc_no, field, val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimitWrapper(self, tc_no, "DHLM", 4.1, 4.1, -5.0, 4.6, -4.5, 50.0, -41.0)

    def test_TC_900220(self):
        tc_no = 900220
        # setLimitWrapper(self, tc_no, field, val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimitWrapper(self, tc_no, "HLM", 4.7, 4.2, -5.0, 4.7, -4.5, 50.0, -42.0)

    def test_TC_900230(self):
        tc_no = 900230
        # setLimitWrapper(self, tc_no, field, val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimitWrapper(self, tc_no, "DLLM", -5.3, 4.2, -5.3, 4.7, -4.8, 53.0, -42.0)

    def test_TC_900240(self):
        tc_no = 900240
        # setLimitWrapper(self, tc_no, field, val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimitWrapper(self, tc_no, "LLM", -5.4, 4.2, -5.9, 4.7, -5.4, 59.0, -42.0)

    ###################################################################################################################
    # Invert dir
    def test_TC_900300(self):
        tc_no = 900300
        encRel = 0
        self.axisCom.putDbgStrToLOG("Start " + str(tc_no), wait=True)
        InitVeloAcc(self, tc_no, encRel)
        mres = 0.1
        dir = 1
        off = 0.5
        setMresDirOff(self, tc_no, mres, dir, off)
        InitLimitsNoROlimits(self, tc_no)
        self.axisCom.putDbgStrToLOG("End " + str(tc_no), wait=True)

    def test_TC_900310(self):
        tc_no = 900310
        # setLimitWrapper(self,tc_no,field, val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimitWrapper(self, tc_no, "DHLM", 4.1, 4.1, -5.0, 5.5, -3.6, 41.0, -50.0)

    def test_TC_900320(self):
        tc_no = 900320
        # setLimitWrapper(self,tc_no,field, val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimitWrapper(self, tc_no, "HLM", 4.7, 4.1, -4.2, 4.7, -3.6, 41.0, -42.0)

    def test_TC_900330(self):
        tc_no = 900330
        # setLimitWrapper(self,tc_no,field, val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimitWrapper(self, tc_no, "DLLM", -5.3, 4.1, -5.3, 5.8, -3.6, 41.0, -53.0)

    def test_TC_900340(self):
        tc_no = 900340
        # setLimitWrapper(self,tc_no,field, val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimitWrapper(self, tc_no, "LLM", -5.4, 5.9, -5.3, 5.8, -5.4, 59.0, -53.0)

    ###################################################################################################################
    # Invert mres, invert dir
    def test_TC_900400(self):
        tc_no = 900400
        encRel = 0
        self.axisCom.putDbgStrToLOG("Start " + str(tc_no), wait=True)
        #                                       mres, dir,off, hlm, expHLM, expM3rhlm, expLLM, expM3rllm)
        InitVeloAcc(self, tc_no, encRel)
        mres = -0.1
        dir = 1
        off = 0.5
        setMresDirOff(self, tc_no, mres, dir, off)
        InitLimitsNoROlimits(self, tc_no)
        self.axisCom.putDbgStrToLOG("End " + str(tc_no), wait=True)

    def test_TC_900410(self):
        tc_no = 900410
        # setLimitWrapper(self,tc_no,field,  val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimitWrapper(self, tc_no, "DHLM", 4.1, 4.1, -5.0, 5.5, -3.6, 50.0, -41.0)

    def test_TC_900420(self):
        tc_no = 900420
        # setLimitWrapper(self,tc_no,field, val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimitWrapper(self, tc_no, "HLM", 4.7, 4.1, -4.2, 4.7, -3.6, 42.0, -41.0)

    def test_TC_900430(self):
        tc_no = 900430
        # setLimitWrapper(self,tc_no,field, val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimitWrapper(self, tc_no, "DLLM", -5.3, 4.1, -5.3, 5.8, -3.6, 53.0, -41.0)

    def test_TC_900440(self):
        tc_no = 900440
        # setLimitWrapper(self,tc_no,field, val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
        setLimitWrapper(self, tc_no, "LLM", -5.4, 5.9, -5.3, 5.8, -5.4, 53.0, -59.0)

    #########################################################################################################
    # Test clipping
    def test_TC_900500(self):
        if self.hasROlimit:
            tc_no = 900500
            encRel = 0
            self.axisCom.putDbgStrToLOG("Start " + str(tc_no), wait=True)

            print(
                f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no} hasROlimit={int(self.hasROlimit)}"
            )
            self.assertEqual(1, self.hasROlimit, "motorRecord supports RO soft limits")

            #                                       mres, dir,off, hlm, expHLM, expM3rhlm, expLLM, expM3rllm)
            InitVeloAcc(self, tc_no, encRel)
            mres = 0.1
            dir = 0
            off = 0.5
            setMresDirOff(self, tc_no, mres, dir, off)
            InitLimitsWithROlimits(self, tc_no)
            self.axisCom.putDbgStrToLOG("End " + str(tc_no), wait=True)

    def test_TC_900510(self):
        if self.hasROlimit:
            tc_no = 900510
            # setLimitWrapper(self,tc_no,field, val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
            setLimitWrapper(self, tc_no, "DHLM", 10, 0.6, -0.7, 1.1, -0.2, 6.0, -7.0)

    def test_TC_900520(self):
        if self.hasROlimit:
            tc_no = 900520
            # setLimitWrapper(self,tc_no,field, val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
            setLimitWrapper(self, tc_no, "HLM", 10, 0.6, -0.7, 1.1, -0.2, 6.0, -7.0)

    def test_TC_900530(self):
        if self.hasROlimit:
            tc_no = 900530
            # setLimitWrapper(self,tc_no,field, val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
            setLimitWrapper(self, tc_no, "DLLM", -10.0, 0.6, -0.7, 1.1, -0.2, 6.0, -7.0)

    def test_TC_900540(self):
        if self.hasROlimit:
            tc_no = 900540
            # setLimitWrapper(self,tc_no,field, val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
            setLimitWrapper(self, tc_no, "LLM", -10.0, 0.6, -0.7, 1.1, -0.2, 6.0, -7.0)

    # Invert mres
    def test_TC_900600(self):
        if self.hasROlimit:
            tc_no = 900600
            self.axisCom.putDbgStrToLOG("Start " + str(tc_no), wait=True)
            encRel = 0

            InitVeloAcc(self, tc_no, encRel)
            mres = -0.1
            dir = 0
            off = 0.5
            setMresDirOff(self, tc_no, mres, dir, off)
            InitLimitsWithROlimits(self, tc_no)
            self.axisCom.putDbgStrToLOG("End " + str(tc_no), wait=True)

    def test_TC_900610(self):
        if self.hasROlimit:
            tc_no = 900610
            # setLimitWrapper(self,tc_no,field, val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
            setLimitWrapper(self, tc_no, "DHLM", 10, 0.7, -0.6, 1.2, -0.1, 6.0, -7.0)

    def test_TC_900620(self):
        if self.hasROlimit:
            tc_no = 900620
            # setLimitWrapper(self,tc_no,field, val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
            setLimitWrapper(self, tc_no, "HLM", 10, 0.7, -0.6, 1.2, -0.1, 6.0, -7.0)

    def test_TC_900630(self):
        if self.hasROlimit:
            tc_no = 900630
            # setLimitWrapper(self,tc_no,field, val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
            setLimitWrapper(self, tc_no, "DLLM", -10.0, 0.7, -0.6, 1.2, -0.1, 6.0, -7.0)

    def test_TC_900640(self):
        if self.hasROlimit:
            tc_no = 900640
            # setLimitWrapper(self,tc_no,field, val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
            setLimitWrapper(self, tc_no, "LLM", -10.0, 0.7, -0.6, 1.2, -0.1, 6.0, -7.0)

    # Invert dir
    def test_TC_900700(self):
        if self.hasROlimit:
            tc_no = 900700
            encRel = 0
            self.axisCom.putDbgStrToLOG("Start " + str(tc_no), wait=True)
            #                                       mres, dir,off, hlm, expHLM, expM3rhlm, expLLM, expM3rllm)
            InitVeloAcc(self, tc_no, encRel)
            mres = 0.1
            dir = 1
            off = 0.5
            setMresDirOff(self, tc_no, mres, dir, off)
            InitLimitsWithROlimits(self, tc_no)
            self.axisCom.putDbgStrToLOG("End " + str(tc_no), wait=True)

    def test_TC_900710(self):
        if self.hasROlimit:
            tc_no = 900710
            # setLimitWrapper(self,tc_no,field, val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
            setLimitWrapper(self, tc_no, "DHLM", 10, 0.6, -0.7, 1.2, -0.1, 6.0, -7.0)

    def test_TC_900720(self):
        if self.hasROlimit:
            tc_no = 900720
            # setLimitWrapper(self,tc_no,field,val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
            setLimitWrapper(self, tc_no, "HLM", 10, 0.6, -0.7, 1.2, -0.1, 6.0, -7.0)

    def test_TC_900730(self):
        if self.hasROlimit:
            tc_no = 900730
            # setLimitWrapper(self,tc_no,field,val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
            setLimitWrapper(self, tc_no, "DLLM", -10.0, 0.6, -0.7, 1.2, -0.1, 6.0, -7.0)

    def test_TC_900740(self):
        if self.hasROlimit:
            tc_no = 900740
            # setLimitWrapper(self,tc_no,field, val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
            setLimitWrapper(self, tc_no, "LLM", -10.0, 0.6, -0.7, 1.2, -0.1, 6.0, -7.0)

    # Invert MRES and dir
    def test_TC_900800(self):
        if self.hasROlimit:
            tc_no = 900800
            self.axisCom.putDbgStrToLOG("Start " + str(tc_no), wait=True)
            encRel = 0
            #                                       mres, dir,off, hlm, expHLM, expM3rhlm, expLLM, expM3rllm)
            InitVeloAcc(self, tc_no, encRel)
            mres = -0.1
            dir = 1
            off = 0.5
            setMresDirOff(self, tc_no, mres, dir, off)
            InitLimitsWithROlimits(self, tc_no)
            self.axisCom.putDbgStrToLOG("End " + str(tc_no), wait=True)

    def test_TC_900810(self):
        if self.hasROlimit:
            tc_no = 900810
            # setLimitWrapper(self,tc_no,field, val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
            setLimitWrapper(self, tc_no, "DHLM", 10, 0.7, -0.6, 1.1, -0.2, 6.0, -7.0)

    def test_TC_900820(self):
        if self.hasROlimit:
            tc_no = 900820
            # setLimitWrapper(self,tc_no,field, val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
            setLimitWrapper(self, tc_no, "HLM", 10, 0.7, -0.6, 1.1, -0.2, 6.0, -7.0)

    def test_TC_900830(self):
        if self.hasROlimit:
            tc_no = 900830
            # setLimitWrapper(self, tc_no, field, val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
            setLimitWrapper(self, tc_no, "DLLM", -10, 0.7, -0.6, 1.1, -0.2, 6.0, -7.0)

    def test_TC_900840(self):
        if self.hasROlimit:
            tc_no = 900840
            # setLimitWrapper(self,tc_no, field, val, expDHLM, expDLLM, expHLM, expLLM, expM3rhlm, expM3rllm):
            setLimitWrapper(self, tc_no, "LLM", -10, 0.7, -0.6, 1.1, -0.2, 6.0, -7.0)

    def test_TC_900999(self):
        tc_no = 900999
        self.axisMr.setFieldSPAM(tc_no, self.oldSPAM)
        if self.drvUseEGU_RB == 1:
            self.axisCom.put("-DrvUseEGU", 1)

    def teardown_class(self):
        tc_no = int(filnam) * 10000 + 9999
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} {tc_no} teardown_class"
        )
        self.axisCom.close()
