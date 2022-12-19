#!/usr/bin/env python
#

import datetime
import inspect
import unittest
import math
import os
import sys
import time
from AxisMr import AxisMr
from AxisCom import AxisCom


filnam = os.path.basename(__file__)[0:3]
###

#
# Test cases to check if JVEL and JAR is send correct to the controller
# The readout is done with additional Records, which already
# exist in the (old) ethercatmcAxis driver.
# This driver (and the corresponding MCU software) allow
# the readout of the commanded velocity and acceleration
# for both "(EPICS) jogging" and positioning.
#
#


def lineno():
    return inspect.currentframe().f_back.f_lineno


polltime = 0.1

# Values to be used for test
# Note: Make sure to use different values to hae a good
# test coverage
myVELO = 20.0  # positioning velocity
myACCL = 1.0  # Time to VELO, seconds

# Different values, high use even, low uses odd
#
myLowHardLimitPos = -19.0
myDLLM = 0.0
myStartposDial = 0.0
myDHLM = 0.0
myHighHardLimitPos = 18.0

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


def InitAllFor950(self, tc_no):
    # msta = int(self.axisCom.get(".MSTA"))
    # assert msta & self.axisMr.MSTA_BIT_HOMED  # , 'MSTA.homed (Axis has been homed)')

    # Prepare parameters for jogging and backlash
    self.axisCom.put(".VELO", myVELO)
    self.axisCom.put(".ACCL", myACCL)

    self.axisCom.put(".BDST", 0.0)
    self.axisMr.setValueOnSimulator(tc_no, "bAxisHomed", 1)
    # Move the  to 0, to avoid limit switch activation
    self.axisMr.moveWait(tc_no, myStartposDial)
    # Speed it up, by setting the position in the simulator
    # and wait for the movement to finish
    self.axisMr.waitForStop(tc_no, 2.0)
    self.axisMr.setValueOnSimulator(tc_no, "nAmplifierPercent", 100)
    self.axisMr.setValueOnSimulator(tc_no, "fHighHardLimitPos", myHighHardLimitPos)
    self.axisMr.setValueOnSimulator(tc_no, "fHighSoftLimitPos", myHighHardLimitPos)
    self.axisMr.setValueOnSimulator(tc_no, "fLowHardLimitPos", myLowHardLimitPos)
    self.axisMr.setValueOnSimulator(tc_no, "fLowSoftLimitPos", myLowHardLimitPos)
    InitLimitsNoROlimits(self, tc_no)


def InitLimitsNoROlimits(self, tc_no):
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
        if (resH == True) and (resL == True):
            return True

        time.sleep(polltime)
        maxTime = maxTime - polltime
    return False


def jogTheMotorToLS(
    self,
    tc_no,
    mres=0,
    dir=-1,
    jogX=0,
    jvel=0,
    jar=0,
    drvUseEGU=0,
    velRB=0,
    accRB=0,
    dhls=-1,
    dlls=-1,
    hls=-1,
    lls=-1,
):
    self.axisCom.putDbgStrToLOG("Start " + str(tc_no), wait=True)
    self.assertNotEqual(0, mres, str(tc_no) + "mres must not be 0")
    self.assertNotEqual(-1, dir, str(tc_no) + "dir must not be -1")
    self.assertNotEqual(0, jogX, str(tc_no) + "jogx must not be 0")
    self.assertNotEqual(0, jvel, str(tc_no) + "jvel must not be 0")
    self.assertNotEqual(0, jar, str(tc_no) + "jar must not be 0")
    self.assertNotEqual(0, velRB, str(tc_no) + "velRB must not be 0")
    self.assertNotEqual(0, accRB, str(tc_no) + "accRB must not be 0")
    self.assertNotEqual(-1, dhls, str(tc_no) + "dhls must not be -1")
    self.assertNotEqual(-1, dlls, str(tc_no) + "dlls must not be -1")
    self.assertNotEqual(-1, hls, str(tc_no) + "hls must not be -1")
    self.assertNotEqual(-1, lls, str(tc_no) + "lls must not be -1")
    self.axisCom.put("." + jogX, 0)
    # Always start in the middle
    destination = 0.0
    self.axisMr.moveWait(tc_no, destination)
    # wait_for_stop = 30
    # self.axisMr.waitForStop(tc_no, wait_for_stop)
    mresAct = self.axisCom.get(".MRES")
    dirAct = self.axisCom.get(".DIR")
    if mresAct != mres:
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} tc_no={tc_no} mresAct={mresAct}"
        )
        # The motorRecord adjusts VELO, BVEL VBAS
        # and this is not a bug. but an old feature
        velo = self.axisCom.get(".VELO")
        bvel = self.axisCom.get(".BVEL")
        vbas = self.axisCom.get(".VBAS")

        self.axisCom.put(".MRES", mres)

        self.axisCom.put(".VELO", mres)
        self.axisCom.put(".BVEL", bvel)
        self.axisCom.put(".VBAS", vbas)

    if dirAct != dir:
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} tc_no={tc_no} dirAct={dirAct}"
        )
        self.axisCom.put(".DIR", dir)

    time.sleep(1.0)
    self.axisCom.put(".JVEL", jvel)
    self.axisCom.put(".JAR", jar)
    self.axisCom.put("." + jogX, 1)
    time.sleep(2.0)
    wait_for_done = 60
    self.axisMr.waitForStartAndDone(str(tc_no) + "B", wait_for_done)

    actHLS = self.axisCom.get(".HLS")
    actLLS = self.axisCom.get(".LLS")
    actVelRB = self.axisCom.get("-Vel-RB")
    actAccRB = self.axisCom.get("-Acc-RB")
    msta = int(self.axisCom.get(".MSTA"))
    if msta & self.axisMr.MSTA_BIT_PLUS_LS:
        actDHLS = 1
    else:
        actDHLS = 0
    if msta & self.axisMr.MSTA_BIT_MINUS_LS:
        actDLLS = 1
    else:
        actDLLS = 0

    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} tc_no={tc_no} mres={mres} dir={dir} jogX={jogX} drvUseEGU={drvUseEGU}"
    )
    print(
        f"tc_no={tc_no} expVelRB={velRB} expAccRB={accRB} expHLS={hls} expLLS={lls} expDHLS={dhls} expDLLS={dlls}"
    )
    print(
        f"tc_no={tc_no} actVelRB={actVelRB} actAccRB={actAccRB} actHLS={actHLS} actLLS={actLLS} actDHLS={actDHLS} actDLLS={actDLLS}"
    )
    maxdelta = 0.1
    okVelRB = self.axisMr.calcAlmostEqual(
        str(tc_no) + " actVelRB", velRB, actVelRB, maxdelta
    )
    okAccRB = self.axisMr.calcAlmostEqual(
        str(tc_no) + " actAccRB", accRB, actAccRB, maxdelta
    )
    msta = int(self.axisCom.get(".MSTA"))
    okNoProblem = True
    if msta & self.axisMr.MSTA_BIT_PROBLEM:
        okNoProblem = False

    testPassed = (
        okNoProblem
        and okVelRB
        and okAccRB
        and (hls == actHLS)
        and (lls == actLLS)
        and (dhls == actDHLS)
        and (dlls == actDLLS)
    )
    if testPassed:
        self.axisCom.putDbgStrToLOG("Passed " + str(tc_no), wait=True)
    else:
        self.axisCom.putDbgStrToLOG("Failed " + str(tc_no), wait=True)
    assert testPassed


def jogTheMotorChangeJVEL(
    self,
    tc_no,
    jogX=0,
    jvel=0,
    jar=0,
    velRB=0,
    accRB=0,
):
    self.axisCom.putDbgStrToLOG("Start " + str(tc_no), wait=True)
    self.assertNotEqual(0, jogX, str(tc_no) + "jogx must not be 0")
    self.assertNotEqual(0, jvel, str(tc_no) + "jvel must not be 0")
    self.assertNotEqual(0, jar, str(tc_no) + "jar must not be 0")
    self.assertNotEqual(0, velRB, str(tc_no) + "velRB must not be 0")
    self.assertNotEqual(0, accRB, str(tc_no) + "accRB must not be 0")
    # Start jogging
    self.axisCom.put("." + jogX, 1)
    time.sleep(2.0)
    wait_for_start = 2
    self.axisMr.waitForStart(tc_no, wait_for_start)

    # change JVEL while jogging
    self.axisCom.put(".JVEL", jvel)
    maxdelta = 0.1
    maxTime = 5
    while maxTime > 0:
        actVelRB = self.axisCom.get("-Vel-RB")
        actAccRB = self.axisCom.get("-Acc-RB")

        okVelRB = self.axisMr.calcAlmostEqual(
            str(tc_no) + "actVelRB", velRB, actVelRB, maxdelta
        )
        okAccRB = self.axisMr.calcAlmostEqual(
            str(tc_no) + "actAccRB", accRB, actAccRB, maxdelta
        )
        testPassed = okVelRB and okAccRB
        if testPassed:
            maxTime = 0
        else:
            time.sleep(polltime)
            maxTime = maxTime - polltime
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} tc_no={tc_no} expVelRB={velRB} expAccRB={accRB}"
    )
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} tc_no={tc_no} actVelRB={actVelRB} actAccRB={actAccRB}"
    )
    self.axisCom.put("." + jogX, 0)
    self.axisCom.put(".STOP", 1)
    wait_for_stop = 5
    self.axisMr.waitForStop(tc_no, wait_for_stop)

    if testPassed:
        self.axisCom.putDbgStrToLOG("Passed " + str(tc_no), wait=True)
    else:
        self.axisCom.putDbgStrToLOG("Failed " + str(tc_no), wait=True)
    assert testPassed


def jogTheMotorWithMRES(
    self, tc_no, mres=0, dir=-1, jogDir=-1, jvel=0, jar=0, drvUseEGU=0
):
    self.assertNotEqual(0, mres, str(tc_no) + "mres must not be 0")
    self.assertNotEqual(-1, dir, str(tc_no) + "dir must not be -1")
    self.assertNotEqual(-1, jogDir, str(tc_no) + "jogDir must not be -1")
    self.assertNotEqual(0, jvel, str(tc_no) + "jvel must not be 0")
    self.assertNotEqual(0, jar, str(tc_no) + "jar must not be 0")

    # The motorRecord User LS (hls/lls) follow the jogging direction
    hls = jogDir
    lls = 1 - jogDir
    # The "mechanical LS" are depending on MRES and DIR
    # See below
    dhls = hls
    dlls = lls

    if drvUseEGU == 0:
        # Classic motorRecord
        velRB = jvel / mres
        accRB = jar / math.fabs(mres)
    else:
        # motorRecord using EGU
        velRB = jvel * math.fabs(mres) / mres  # Keep the sign
        accRB = jar

    # Command to the motorRecord
    if jogDir == 1:
        jogX = "JOGF"
        hls = 1
        lls = 0
    else:
        self.assertEqual(0, jogDir, str(tc_no) + "jogDir must be 0 or 1")
        jogX = "JOGR"
        velRB = 0 - velRB  # Change sign
        hls = 0
        lls = 1

    if mres < 0:
        dhls = 1 - dhls
        dlls = 1 - dlls
    if dir > 0:
        # dir == 1 means "invers dial <--> user direction
        dhls = 1 - dhls
        dlls = 1 - dlls
        velRB = 0 - velRB  # Change sign

    # Now we have calulated all data
    jogTheMotorToLS(
        self,
        tc_no,
        mres=mres,
        dir=dir,
        jogX=jogX,
        jvel=jvel,
        jar=jar,
        drvUseEGU=drvUseEGU,
        velRB=velRB,
        accRB=accRB,
        dhls=dhls,
        dlls=dlls,
        hls=hls,
        lls=lls,
    )
    # 2nd round, jog away from LS and, once moving,
    # change JVEL
    tc_no = tc_no + 1
    jvel2 = jvel / 2.0
    velRB2 = (0 - velRB) / 2.0
    accRB2 = accRB
    if jogDir == 1:
        jogX2 = "JOGR"
    else:
        self.assertEqual(0, jogDir, str(tc_no) + "jogDir must be 0 or 1")
        jogX2 = "JOGF"
    jogTheMotorChangeJVEL(
        self,
        tc_no,
        jogX=jogX2,
        jvel=jvel2,
        jar=jar,
        velRB=velRB2,
        accRB=accRB2,
    )


def jogTheMotorTestWrapper(self, tc_no, mres=0, dir=-1, jvel=8.0, jar=3.0):
    jogDir = 1
    jogTheMotorTestWrapperJogDir(
        self, tc_no, mres=mres, dir=dir, jogDir=jogDir, jvel=jvel, jar=jar
    )
    tc_no = tc_no + 100
    jogDir = 0
    jogTheMotorTestWrapperJogDir(
        self, tc_no, mres=mres, dir=dir, jogDir=jogDir, jvel=jvel, jar=jar
    )


def jogTheMotorTestWrapperJogDir(self, tc_no, mres=0, dir=-1, jogDir=-1, jvel=0, jar=0):
    # Check which motorRecord version we have
    vers = float(self.axisCom.get(".VERS"))
    if vers >= 6.94 and vers <= 7.09:
        # Special feature: The motorRecord can talk to the driver
        # in EGU. Changing MRES does NOT affect velocity
        hasDrvUseEGUfeature = True
        self.axisCom.put("-DrvUseEGU", 0)
    else:
        hasDrvUseEGUfeature = False

    jogTheMotorWithMRES(
        self, tc_no, mres=mres, dir=dir, jogDir=jogDir, jvel=jvel, jar=jar
    )

    # Check if mres changes the raw velocity/acceleration
    tc_no = tc_no + 2
    mres2 = mres * 2
    jogTheMotorWithMRES(
        self, tc_no, mres=mres2, dir=dir, jogDir=jogDir, jvel=jvel, jar=jar
    )
    if hasDrvUseEGUfeature:
        drvUseEGU = 1
        self.axisCom.put("-DrvUseEGU", drvUseEGU)
        tc_no = tc_no + 2
        mres3 = mres * 3
        jogTheMotorWithMRES(
            self,
            tc_no,
            mres=mres3,
            dir=dir,
            jogDir=jogDir,
            jvel=jvel,
            jar=jar,
            drvUseEGU=drvUseEGU,
        )


class Test(unittest.TestCase):
    drvUseEGU_RB = None
    drvUseEGU = 0
    url_string = os.getenv("TESTEDMOTORAXIS")
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} url_string={url_string}"
    )

    axisCom = AxisCom(url_string, log_debug=False)
    axisMr = AxisMr(axisCom)

    # Initialize
    def test_TC_9500000(self):
        tc_no = 9500000
        self.axisCom.putDbgStrToLOG("Start " + str(tc_no), wait=True)
        self.axisCom.put(".SPAM", 255)
        InitAllFor950(self, tc_no)
        self.axisCom.putDbgStrToLOG("End " + str(tc_no), wait=True)

    # DIR = 0
    def test_TC_9500010(self):
        tc_no = 9500010
        self.assertEqual(
            0,
            int(self.axisCom.get(".MSTA")) & self.axisMr.MSTA_BIT_PROBLEM,
            "MSTA.Problem should be 0",
        )
        mres = 1.0
        dir = 0
        jogTheMotorTestWrapper(self, tc_no, mres=mres, dir=dir)

    # DIR = 1
    def test_TC_9500020(self):
        tc_no = 9500020
        self.assertEqual(
            0,
            int(self.axisCom.get(".MSTA")) & self.axisMr.MSTA_BIT_PROBLEM,
            "MSTA.Problem should be 0",
        )
        mres = 1.0
        dir = 1

        jogTheMotorTestWrapper(self, tc_no, mres=mres, dir=dir)

    # DIR = 1, MRES = - 1.0
    def test_TC_9500030(self):
        tc_no = 9500030
        mres = -1.0
        dir = 1

        jogTheMotorTestWrapper(self, tc_no, mres=mres, dir=dir)

    # DIR = 0, MRES = - 1.0
    def test_TC_9500040(self):
        tc_no = 9500040
        self.assertEqual(
            0,
            int(self.axisCom.get(".MSTA")) & self.axisMr.MSTA_BIT_PROBLEM,
            "MSTA.Problem should be 0",
        )
        mres = -1.0
        dir = 0
        jogDir = 1

        jogTheMotorTestWrapper(self, tc_no, mres=mres, dir=dir)

    def teardown_class(self):
        tc_no = int(filnam) * 10000 + 9999
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} {tc_no} teardown_class"
        )
        self.axisCom.close()
