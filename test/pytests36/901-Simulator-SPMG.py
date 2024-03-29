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


def lineno():
    return inspect.currentframe().f_back.f_lineno


motorSPMG_Stop = 0
motorSPMG_Pause = 1
motorSPMG_Move = 2
motorSPMG_Go = 3


polltime = 0.1

# Comparing floating points may fail because of rounding problems
maxdelta = 0.01


def spmgValChanged(self, tc_no, val, spmg=-1, jitteringPos=0.0):
    self.axisCom.putDbgStrToLOG("Start " + str(tc_no), wait=True)
    oldSPAM = self.axisMr.getFieldSPAM(tc_no)
    self.axisMr.setFieldSPAM(tc_no, 2047)
    self.axisCom.put(".SPMG", spmg)
    self.axisCom.put(".VAL", val)
    self.axisMr.setValueOnSimulator(tc_no, "fActPosition", float(jitteringPos))
    timeout = 3.0
    self.axisMr.waitForValueChanged(tc_no, ".RBV", jitteringPos, maxdelta, timeout)
    testPassed = True
    dmov = int(self.axisCom.get(".DMOV", use_monitor=False))
    miss = int(self.axisCom.get(".MISS", use_monitor=False))
    self.axisCom.put(".SPMG", motorSPMG_Go)

    self.axisMr.waitForValueChanged(tc_no, ".RBV", val, maxdelta, timeout)
    self.axisMr.setFieldSPAM(tc_no, oldSPAM)
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {tc_no} dmov={dmov} miss={miss}"
    )
    debug_text = f"miss={miss} dmov={dmov}"
    self.axisCom.putDbgStrToLOG(debug_text, wait=True)
    if miss or dmov:
        testPassed = False

    if testPassed:
        self.axisCom.putDbgStrToLOG("Passed " + str(tc_no), wait=True)
    else:
        self.axisCom.putDbgStrToLOG("Failed " + str(tc_no), wait=True)
    assert testPassed


class Test(unittest.TestCase):
    url_string = os.getenv("TESTEDMOTORAXIS")
    print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} url_string={url_string}")

    axisCom = AxisCom(url_string, log_debug=False)
    axisMr = AxisMr(axisCom)

    # Make sure that simulator is initialized
    def test_TC_9011(self):
        tc_no = "9011"
        self.axisCom.putDbgStrToLOG("Start " + str(tc_no), wait=True)
        self.axisCom.put(".SPMG", motorSPMG_Go)
        self.axisMr.motorInitAllForBDST(tc_no)
        self.axisCom.putDbgStrToLOG("End " + str(tc_no), wait=True)

    # SPMG pause & VAL changed, no jitter
    def test_TC_9012(self):
        tc_no = 9012
        rbv = self.axisCom.get(".RBV")
        jitteringPos = rbv
        val = jitteringPos + 1.0
        spmgValChanged(
            self, tc_no, val, spmg=motorSPMG_Pause, jitteringPos=jitteringPos
        )

    # SPMG pause & VAL changed, little jitter
    def test_TC_9013(self):
        tc_no = 9013
        rbv = self.axisCom.get(".RBV")
        rdbd = self.axisCom.get(".RDBD")
        val = rbv + 1.0
        jitteringPos = rbv + (rdbd / 4.0)
        spmgValChanged(
            self, tc_no, val, spmg=motorSPMG_Pause, jitteringPos=jitteringPos
        )

    # SPMG stop & VAL changed, no jitter
    def test_TC_9014(self):
        tc_no = 9014
        rbv = self.axisCom.get(".RBV")
        jitteringPos = rbv
        val = jitteringPos + 1.0
        spmgValChanged(self, tc_no, val, spmg=motorSPMG_Stop, jitteringPos=jitteringPos)

    # SPMG stop & VAL changed, little jitter
    # This does not work in the motorRecord

    #    def test_TC_9015(self):
    #        tc_no = 9015
    #        rbv = self.axisCom.get(".RBV")
    #        rdbd = self.axisCom.get(".RDBD")
    #        val = rbv + 1.0
    #        jitteringPos = rbv + (rdbd / 4.0)
    #        spmgValChanged(self, tc_no, val, spmg=motorSPMG_Stop, jitteringPos=jitteringPos)

    def teardown_class(self):
        tc_no = int(filnam) * 10000 + 9999
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} {tc_no} teardown_class"
        )
        self.axisCom.close()
