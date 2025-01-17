#!/usr/bin/env python
#

import datetime
import inspect
import unittest
import os
from AxisMr import AxisMr
from AxisCom import AxisCom

filnam = os.path.basename(__file__)[0:3]
###


###

myFRAC = 1.5
myBDST = 24.0

motorRMOD_D = 0  # "Default"
motorRMOD_A = 1  # "Arithmetic"
motorRMOD_G = 2  # "Geometric"
motorRMOD_I = 3  # "In-Position"

#
# How we move: Absolute (without encoder) or relative (with encode via UEIP)
use_abs = 0
use_rel = 1
# Note: motorRMOD_I is always absolute !


def lineno():
    return inspect.currentframe().f_back.f_lineno


def positionAndBacklash(
    self,
    tc_no,
    bdst,
    rmod,
    encRel,
    motorStartPos,
    motorEndPos,
    rtry=None,
    need_007_017_tweak=False,
):
    ###########
    self.axisCom.putDbgStrToLOG("Start " + str(tc_no), wait=True)
    self.axisMr.motorInitAllForBDSTIfNeeded(tc_no)
    self.axisCom.put(".RMOD", rmod)
    self.axisCom.put(".UEIP", encRel)
    if rtry is not None:
        self.axisCom.put(".RTRY", rtry)
    self.axisCom.put(".FRAC", myFRAC)
    self.axisCom.put(".BDST", bdst)

    mot = self.axisCom.getMotorPvName()
    fileName = "/tmp/" + mot.replace(":", "-") + "-" + str(tc_no)
    expFileName = fileName + ".exp"
    actFileName = fileName + ".act"

    self.axisMr.setFieldSPAM(tc_no, -1)
    passed = self.axisMr.setMotorStartPos(tc_no, motorStartPos)

    if not passed:
        self.axisCom.putDbgStrToLOG("FailedX " + str(tc_no), wait=True)
    assert passed

    self.axisMr.setValueOnSimulator(tc_no, "bManualSimulatorMode", 1)
    self.axisMr.setValueOnSimulator(tc_no, "log", actFileName)
    start_change_cnt_dmov_true = self.axisCom.get_change_cnts("dmov_true")
    start_change_cnt_dmov_false = self.axisCom.get_change_cnts("dmov_false")
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} {tc_no} start_change_cnt_dmov_true={start_change_cnt_dmov_true} start_change_cnt_dmov_false={start_change_cnt_dmov_false}"
    )
    self.axisCom.put(".VAL", motorEndPos, wait=True)

    # Create a "expected" file
    expFile = open(expFileName, "w")
    # Positioning
    # 2 different ways to move:
    # - Within the backlash distance and into the backlash direction:
    #   single move with back lash parameters
    # - against the backlash direction -or- bigger than the backlash distance:
    #   two moves, first with moving, second with backlash parameters

    self.axisMr.writeExpFileRMOD_X(
        tc_no,
        rmod,
        expFile,
        myFRAC,
        encRel,
        motorStartPos,
        motorEndPos,
        need_007_017_tweak=need_007_017_tweak,
    )

    expFile.close()
    # self.axisMr.setValueOnSimulator( tc_no, "dbgCloseLogFile", "1")

    time_to_wait = 100
    self.axisMr.waitForStop(tc_no, time_to_wait)
    self.axisMr.setValueOnSimulator(tc_no, "dbgCloseLogFile", "1")
    # time.sleep(2)
    self.axisMr.setValueOnSimulator(tc_no, "bManualSimulatorMode", 0)
    passed = self.axisMr.cmpUnlinkExpectedActualFile(tc_no, expFileName, actFileName)
    end_change_cnt_dmov_true = self.axisCom.get_change_cnts("dmov_true")
    end_change_cnt_dmov_false = self.axisCom.get_change_cnts("dmov_false")
    num_change_cnt_dmov_true = end_change_cnt_dmov_true - start_change_cnt_dmov_true
    num_change_cnt_dmov_false = end_change_cnt_dmov_false - start_change_cnt_dmov_false
    passed = passed and num_change_cnt_dmov_true == 1 and num_change_cnt_dmov_false == 1
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} {tc_no} end_change_cnt_dmov_true={end_change_cnt_dmov_true} end_change_cnt_dmov_false={end_change_cnt_dmov_false}"
    )
    if passed:
        self.axisCom.putDbgStrToLOG("Passed " + str(tc_no), wait=True)
    else:
        self.axisCom.putDbgStrToLOG("Failed " + str(tc_no), wait=True)
    assert passed


class Test(unittest.TestCase):
    url_string = os.getenv("TESTEDMOTORAXIS")
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} url_string={url_string}"
    )

    axisCom = AxisCom(url_string, log_debug=False)
    axisMr = AxisMr(axisCom)
    msta = int(axisCom.get(".MSTA"))

    axisCom.put("-DbgStrToLOG", "Start " + os.path.basename(__file__)[0:20])
    myPOSlow = axisMr.myPOSlow
    myPOSmid = axisMr.myPOSmid
    myPOShig = axisMr.myPOShig

    def test_TC_92000(self):
        self.axisMr.motorInitAllForBDST(920000)
        self.axisCom.put(".FRAC", myFRAC)

    # motorRMOD_D = 0 # "Default"
    # position forward & backlash compensation, absolute
    def test_TC_920001(self):
        positionAndBacklash(
            self, 920001, myBDST, motorRMOD_D, use_abs, self.myPOSlow, self.myPOShig
        )

    # position forward & backlash compensation, relative
    def test_TC_920002(self):
        positionAndBacklash(
            self, 920002, myBDST, motorRMOD_D, use_rel, self.myPOSlow, self.myPOShig
        )

    # position backward & backlash compensation, absolute
    def test_TC_920003(self):
        positionAndBacklash(
            self, 920003, myBDST, motorRMOD_D, use_abs, self.myPOShig, self.myPOSlow
        )

    # position backward & backlash compensation, relative
    def test_TC_920004(self):
        positionAndBacklash(
            self, 920004, myBDST, motorRMOD_D, use_rel, self.myPOShig, self.myPOSlow
        )

    # position forward inside backlash range, absolute
    def test_TC_920005(self):
        positionAndBacklash(
            self, 920005, myBDST, motorRMOD_D, use_abs, self.myPOSmid, self.myPOSlow
        )

    # position forward inside backlash range, relative
    def test_TC_920006(self):
        positionAndBacklash(
            self, 920006, myBDST, motorRMOD_D, use_rel, self.myPOSmid, self.myPOSlow
        )

    # position forward inside backlash range, absolute
    def test_TC_920007(self):
        positionAndBacklash(
            self,
            920007,
            myBDST,
            motorRMOD_D,
            use_abs,
            self.myPOSlow,
            self.myPOSmid,
            need_007_017_tweak=True,
        )

    # position forward inside backlash range, relative
    def test_TC_920008(self):
        positionAndBacklash(
            self, 920008, myBDST, motorRMOD_D, use_rel, self.myPOSlow, self.myPOSmid
        )

    ###############################################################################
    # motorRMOD_A
    # position forward & backlash compensation, absolute
    def test_TC_920011(self):
        positionAndBacklash(
            self, 920011, myBDST, motorRMOD_A, use_abs, self.myPOSlow, self.myPOShig
        )

    # position forward & backlash compensation, relative
    def test_TC_920012(self):
        positionAndBacklash(
            self, 920012, myBDST, motorRMOD_A, use_rel, self.myPOSlow, self.myPOShig
        )

    # position backward & backlash compensation, absolute
    def test_TC_920013(self):
        positionAndBacklash(
            self, 920013, myBDST, motorRMOD_A, use_abs, self.myPOShig, self.myPOSlow
        )

    # position backward & backlash compensation, relative
    def test_TC_920014(self):
        positionAndBacklash(
            self, 920014, myBDST, motorRMOD_A, use_rel, self.myPOShig, self.myPOSlow
        )

    # position forward inside backlash range, absolute
    def test_TC_920015(self):
        positionAndBacklash(
            self, 920015, myBDST, motorRMOD_A, use_abs, self.myPOSmid, self.myPOSlow
        )

    # position forward inside backlash range, relative
    def test_TC_920016(self):
        positionAndBacklash(
            self, 920016, myBDST, motorRMOD_A, use_rel, self.myPOSmid, self.myPOSlow
        )

    # position forward inside backlash range, absolute
    def test_TC_920017(self):
        positionAndBacklash(
            self, 920017, myBDST, motorRMOD_A, use_abs, self.myPOSlow, self.myPOSmid
        )

    # position forward inside backlash range, relative
    def test_TC_920018(self):
        positionAndBacklash(
            self,
            920018,
            myBDST,
            motorRMOD_A,
            use_rel,
            self.myPOSlow,
            self.myPOSmid,
            need_007_017_tweak=True,
        )

    ###############################################################################
    # motorRMOD_G
    # position forward & backlash compensation, absolute
    def test_TC_920021(self):
        positionAndBacklash(
            self, 920021, myBDST, motorRMOD_G, use_abs, self.myPOSlow, self.myPOShig
        )

    # position forward & backlash compensation, relative
    def test_TC_920022(self):
        positionAndBacklash(
            self, 920022, myBDST, motorRMOD_G, use_rel, self.myPOSlow, self.myPOShig
        )

    # position backward & backlash compensation, absolute
    def test_TC_920023(self):
        positionAndBacklash(
            self, 920023, myBDST, motorRMOD_G, use_abs, self.myPOShig, self.myPOSlow
        )

    # position backward & backlash compensation, relative
    def test_TC_920024(self):
        positionAndBacklash(
            self, 920024, myBDST, motorRMOD_G, use_rel, self.myPOShig, self.myPOSlow
        )

    # position forward inside backlash range, absolute
    def test_TC_920025(self):
        positionAndBacklash(
            self, 920025, myBDST, motorRMOD_G, use_abs, self.myPOSmid, self.myPOSlow
        )

    # position forward inside backlash range, relative
    def test_TC_920026(self):
        positionAndBacklash(
            self, 920026, myBDST, motorRMOD_G, use_rel, self.myPOSmid, self.myPOSlow
        )

    # position forward inside backlash range, absolute
    def test_TC_920027(self):
        positionAndBacklash(
            self, 920027, myBDST, motorRMOD_G, use_abs, self.myPOSlow, self.myPOSmid
        )

    # position forward inside backlash range, relative
    def test_TC_920028(self):
        positionAndBacklash(
            self, 920028, myBDST, motorRMOD_G, use_rel, self.myPOSlow, self.myPOSmid
        )

    ###############################################################################
    # motorRMOD_I
    # position forward & backlash compensation, absolute
    def test_TC_920031(self):
        positionAndBacklash(
            self, 920031, myBDST, motorRMOD_I, use_abs, self.myPOSlow, self.myPOShig
        )

    # position forward & backlash compensation, relative
    def test_TC_920032(self):
        positionAndBacklash(
            self, 920032, myBDST, motorRMOD_I, use_rel, self.myPOSlow, self.myPOShig
        )

    # position backward & backlash compensation, absolute
    def test_TC_920033(self):
        positionAndBacklash(
            self, 920033, myBDST, motorRMOD_I, use_abs, self.myPOShig, self.myPOSlow
        )

    # position backward & backlash compensation, relative
    def test_TC_920034(self):
        positionAndBacklash(
            self, 920034, myBDST, motorRMOD_I, use_rel, self.myPOShig, self.myPOSlow
        )

    # position forward inside backlash range, absolute
    def test_TC_920035(self):
        positionAndBacklash(
            self, 920035, myBDST, motorRMOD_I, use_abs, self.myPOSmid, self.myPOSlow
        )

    # position forward inside backlash range, relative
    def test_TC_920036(self):
        positionAndBacklash(
            self, 920036, myBDST, motorRMOD_I, use_rel, self.myPOSmid, self.myPOSlow
        )

    # position forward inside backlash range, absolute
    def test_TC_920037(self):
        positionAndBacklash(
            self, 920037, myBDST, motorRMOD_I, use_abs, self.myPOSlow, self.myPOSmid
        )

    # position forward inside backlash range, relative
    def test_TC_920038(self):
        positionAndBacklash(
            self, 920038, myBDST, motorRMOD_I, use_rel, self.myPOSlow, self.myPOSmid
        )

    ##############################################################################
    # Test with "negative backlash destination", relative movements
    # position backward & backlash compensation, relative
    def test_TC_920104(self):
        positionAndBacklash(
            self, 920104, -myBDST, motorRMOD_D, use_rel, self.myPOShig, self.myPOSlow
        )

    # position backward & backlash compensation, relative
    def test_TC_920114(self):
        positionAndBacklash(
            self, 920114, -myBDST, motorRMOD_A, use_rel, self.myPOShig, self.myPOSlow
        )

    # position backward & backlash compensation, relative
    def test_TC_920124(self):
        positionAndBacklash(
            self, 920124, -myBDST, motorRMOD_G, use_rel, self.myPOShig, self.myPOSlow
        )

    def teardown_class(self):
        tc_no = int(filnam) * 10000 + 9999
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} {tc_no} teardown_class"
        )
        self.axisCom.close()
