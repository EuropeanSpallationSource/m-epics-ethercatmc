#!/usr/bin/env python
#
import unittest
import os
import sys
from AxisMr import AxisMr
from AxisCom import AxisCom

import time

###


class Test(unittest.TestCase):
    url_string = os.getenv("TESTEDMOTORAXIS")
    print(f"url_string={url_string}")

    axisCom = AxisCom(url_string, log_debug=True)
    axisMr = AxisMr(axisCom)

    # self.axisCom.put('-DbgStrToLOG', "Start " + os.path.basename(__file__)[0:20])
    # 10% dialPosition
    def test_TC_501(self):
        tc_no = "TC_501-10-percent-dialPosition"
        print(f"{tc_no}")
        saved_HLM = self.axisCom.get(".HLM")
        saved_LLM = self.axisCom.get(".LLM")

        destination = (1 * saved_HLM + 9 * saved_LLM) / 10
        done = self.axisMr.moveWait(tc_no, destination)
        self.assertEqual(1, done, "moveWait should return done")

    # 10% dialPosition + X
    def test_TC_502(self):
        tc_no = "TC_502-10-percent-plus-1"
        print(f"{tc_no}")
        rbv = self.axisCom.get(".RBV")
        saved_DLY = self.axisCom.get(".DLY")
        saved_VELO = self.axisCom.get(".VELO")
        saved_ACCL = self.axisCom.get(".ACCL")

        self.axisCom.put(".DLY", 5.2)
        self.axisCom.put(".VELO", 1)
        self.axisCom.put(".ACCL", 1)
        self.axisCom.put(".VAL", rbv + 1.0, wait=False)

        time.sleep(4.0)
        movn1 = self.axisCom.get(".MOVN")
        self.axisCom.put(".STOP", 1)
        time.sleep(7.0)
        self.axisCom.put(".SPMG", 0)
        self.axisCom.put(".SPMG", 3)
        time.sleep(4.0)
        dmov = self.axisCom.get(".DMOV")
        print("%s: movn1=%d dmov=%d" % (tc_no, movn1, dmov))
        self.axisCom.put(".DLY", saved_DLY)
        self.axisCom.put(".VELO", saved_VELO)
        self.axisCom.put(".ACCL", saved_ACCL)
