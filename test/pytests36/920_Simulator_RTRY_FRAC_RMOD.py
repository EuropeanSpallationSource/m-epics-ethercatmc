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


def motorInitTC(self, tc_no, rmod, encRel):
    self.axisCom.put(".RMOD", rmod)
    self.axisCom.put(".UEIP", encRel)


def positionAndBacklash(self, tc_no, bdst, rmod, encRel, motorStartPos, motorEndPos):
    ###########
    self.axisCom.putDbgStrToLOG("Start " + str(tc_no), wait=True)
    self.axisMr.motorInitAllForBDSTIfNeeded(tc_no)
    self.axisCom.put(".FRAC", myFRAC)
    self.axisCom.put(".BDST", bdst)

    mot = self.axisCom.getMotorPvName()
    fileName = "/tmp/" + mot.replace(":", "-") + "-" + str(tc_no)
    expFileName = fileName + ".exp"
    actFileName = fileName + ".act"

    motorInitTC(self, tc_no, rmod, encRel)
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

    axisCom = AxisCom(url_string, log_debug=False, monitor_list=[".DMOV"])
    axisMr = AxisMr(axisCom)
    msta = int(axisCom.get(".MSTA"))

    axisCom.put("-DbgStrToLOG", "Start " + os.path.basename(__file__)[0:20])
    myPOSlow = axisMr.myPOSlow
    myPOSmid = axisMr.myPOSmid
    myPOShig = axisMr.myPOShig

    def test_TC_92000(self):
        self.axisMr.motorInitAllForBDST(92000)
        self.axisCom.put(".FRAC", myFRAC)

    # motorRMOD_D = 0 # "Default"
    # position forward & backlash compensation, absolute
    def test_TC_92001(self):
        positionAndBacklash(
            self, 92001, myBDST, motorRMOD_D, use_abs, self.myPOSlow, self.myPOShig
        )

    # position forward & backlash compensation, relative
    def test_TC_92002(self):
        positionAndBacklash(
            self, 92002, myBDST, motorRMOD_D, use_rel, self.myPOSlow, self.myPOShig
        )

    # position backward & backlash compensation, absolute
    def test_TC_92003(self):
        positionAndBacklash(
            self, 92003, myBDST, motorRMOD_D, use_abs, self.myPOShig, self.myPOSlow
        )

    # position backward & backlash compensation, relative
    def test_TC_92004(self):
        positionAndBacklash(
            self, 92004, myBDST, motorRMOD_D, use_rel, self.myPOShig, self.myPOSlow
        )

    # position forward inside backlash range, absolute
    def test_TC_92005(self):
        positionAndBacklash(
            self, 92005, myBDST, motorRMOD_D, use_abs, self.myPOSmid, self.myPOSlow
        )

    # position forward inside backlash range, relative
    def test_TC_92006(self):
        positionAndBacklash(
            self, 92006, myBDST, motorRMOD_D, use_rel, self.myPOSmid, self.myPOSlow
        )

    # position forward inside backlash range, absolute
    def test_TC_92007(self):
        positionAndBacklash(
            self, 92007, myBDST, motorRMOD_D, use_abs, self.myPOSlow, self.myPOSmid
        )

    # position forward inside backlash range, relative
    def test_TC_92008(self):
        positionAndBacklash(
            self, 92008, myBDST, motorRMOD_D, use_rel, self.myPOSlow, self.myPOSmid
        )

    ###############################################################################
    # motorRMOD_A
    # position forward & backlash compensation, absolute
    def test_TC_92011(self):
        positionAndBacklash(
            self, 92011, myBDST, motorRMOD_A, use_abs, self.myPOSlow, self.myPOShig
        )

    # position forward & backlash compensation, relative
    def test_TC_92012(self):
        positionAndBacklash(
            self, 92012, myBDST, motorRMOD_A, use_rel, self.myPOSlow, self.myPOShig
        )

    # position backward & backlash compensation, absolute
    def test_TC_92013(self):
        positionAndBacklash(
            self, 92013, myBDST, motorRMOD_A, use_abs, self.myPOShig, self.myPOSlow
        )

    # position backward & backlash compensation, relative
    def test_TC_92014(self):
        positionAndBacklash(
            self, 92014, myBDST, motorRMOD_A, use_rel, self.myPOShig, self.myPOSlow
        )

    # position forward inside backlash range, absolute
    def test_TC_92015(self):
        positionAndBacklash(
            self, 92015, myBDST, motorRMOD_A, use_abs, self.myPOSmid, self.myPOSlow
        )

    # position forward inside backlash range, relative
    def test_TC_92016(self):
        positionAndBacklash(
            self, 92016, myBDST, motorRMOD_A, use_rel, self.myPOSmid, self.myPOSlow
        )

    # position forward inside backlash range, absolute
    def test_TC_92017(self):
        positionAndBacklash(
            self, 92017, myBDST, motorRMOD_A, use_abs, self.myPOSlow, self.myPOSmid
        )

    # position forward inside backlash range, relative
    def test_TC_92018(self):
        positionAndBacklash(
            self, 92018, myBDST, motorRMOD_A, use_rel, self.myPOSlow, self.myPOSmid
        )

    ###############################################################################
    # motorRMOD_G
    # position forward & backlash compensation, absolute
    def test_TC_92021(self):
        positionAndBacklash(
            self, 92021, myBDST, motorRMOD_G, use_abs, self.myPOSlow, self.myPOShig
        )

    # position forward & backlash compensation, relative
    def test_TC_92022(self):
        positionAndBacklash(
            self, 92022, myBDST, motorRMOD_G, use_rel, self.myPOSlow, self.myPOShig
        )

    # position backward & backlash compensation, absolute
    def test_TC_92023(self):
        positionAndBacklash(
            self, 92023, myBDST, motorRMOD_G, use_abs, self.myPOShig, self.myPOSlow
        )

    # position backward & backlash compensation, relative
    def test_TC_92024(self):
        positionAndBacklash(
            self, 92024, myBDST, motorRMOD_G, use_rel, self.myPOShig, self.myPOSlow
        )

    # position forward inside backlash range, absolute
    def test_TC_92025(self):
        positionAndBacklash(
            self, 92025, myBDST, motorRMOD_G, use_abs, self.myPOSmid, self.myPOSlow
        )

    # position forward inside backlash range, relative
    def test_TC_92026(self):
        positionAndBacklash(
            self, 92026, myBDST, motorRMOD_G, use_rel, self.myPOSmid, self.myPOSlow
        )

    # position forward inside backlash range, absolute
    def test_TC_92027(self):
        positionAndBacklash(
            self, 92027, myBDST, motorRMOD_G, use_abs, self.myPOSlow, self.myPOSmid
        )

    # position forward inside backlash range, relative
    def test_TC_92028(self):
        positionAndBacklash(
            self, 92028, myBDST, motorRMOD_G, use_rel, self.myPOSlow, self.myPOSmid
        )

    ###############################################################################
    # motorRMOD_I
    # position forward & backlash compensation, absolute
    def test_TC_92031(self):
        positionAndBacklash(
            self, 92031, myBDST, motorRMOD_I, use_abs, self.myPOSlow, self.myPOShig
        )

    # position forward & backlash compensation, relative
    def test_TC_92032(self):
        positionAndBacklash(
            self, 92032, myBDST, motorRMOD_I, use_rel, self.myPOSlow, self.myPOShig
        )

    # position backward & backlash compensation, absolute
    def test_TC_92033(self):
        positionAndBacklash(
            self, 92033, myBDST, motorRMOD_I, use_abs, self.myPOShig, self.myPOSlow
        )

    # position backward & backlash compensation, relative
    def test_TC_92034(self):
        positionAndBacklash(
            self, 92034, myBDST, motorRMOD_I, use_rel, self.myPOShig, self.myPOSlow
        )

    # position forward inside backlash range, absolute
    def test_TC_92035(self):
        positionAndBacklash(
            self, 92035, myBDST, motorRMOD_I, use_abs, self.myPOSmid, self.myPOSlow
        )

    # position forward inside backlash range, relative
    def test_TC_92036(self):
        positionAndBacklash(
            self, 92036, myBDST, motorRMOD_I, use_rel, self.myPOSmid, self.myPOSlow
        )

    # position forward inside backlash range, absolute
    def test_TC_92037(self):
        positionAndBacklash(
            self, 92037, myBDST, motorRMOD_I, use_abs, self.myPOSlow, self.myPOSmid
        )

    # position forward inside backlash range, relative
    def test_TC_92038(self):
        positionAndBacklash(
            self, 92038, myBDST, motorRMOD_I, use_rel, self.myPOSlow, self.myPOSmid
        )

    ##############################################################################
    # Test with "negative backlash destination", relative movements
    # position backward & backlash compensation, relative
    def test_TC_92104(self):
        positionAndBacklash(
            self, 92104, -myBDST, motorRMOD_D, use_rel, self.myPOShig, self.myPOSlow
        )

    # position backward & backlash compensation, relative
    def test_TC_92114(self):
        positionAndBacklash(
            self, 92114, -myBDST, motorRMOD_A, use_rel, self.myPOShig, self.myPOSlow
        )

    # position backward & backlash compensation, relative
    def test_TC_92124(self):
        positionAndBacklash(
            self, 92124, -myBDST, motorRMOD_G, use_rel, self.myPOShig, self.myPOSlow
        )

    def teardown_class(self):
        tc_no = int(filnam) * 10000 + 9999
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} {tc_no} teardown_class"
        )
        self.axisCom.close()
