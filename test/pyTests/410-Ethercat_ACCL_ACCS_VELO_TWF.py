#!/usr/bin/env python

import unittest
import os
import sys
from AxisMr import AxisMr
from AxisCom import AxisCom

import time

###


def getAccEGUfromMCU(self, tc_no):
    print("%s: getAccEGUfromMCU" % (tc_no))
    res = self.axisCom.get("-Acc-RB")
    return res


def check_VBAS_VELO_ACCL_ACCS_accEGU(self, tc_no, vbas, velo, accl, accs, expAccEGU):
    # Put the values which the test case wanted
    if vbas > -1:
        self.axisCom.put(".VBAS", vbas)
    if velo > -1:
        self.axisCom.put(".VELO", velo)
    if accl > -1:
        self.axisCom.put(".ACCL", accl)
    if accs > -1:
        self.axisCom.put(".ACCS", accs)
    # Move the  2 mm (hardcoded) + RDBD
    destination = 2.0 + self.axisCom.get(".VAL") + 2 * self.axisCom.get(".RDBD")
    done = self.axisMr.moveWait(tc_no, destination)
    resAccEGU = getAccEGUfromMCU(self, tc_no)
    print(
        "%s: check_accEGU_ACCS_ACCL_VELO vbas=%f velo=%f accl=%f accs=%f expAccEGU=%f resAccEGU=%f"
        % (tc_no, vbas, velo, accl, accs, expAccEGU, resAccEGU)
    )
    actVelo = self.axisCom.get(".VELO", use_monitor=False)
    actAccl = self.axisCom.get(".ACCL", use_monitor=False)
    actAccs = self.axisCom.get(".ACCS", use_monitor=False)
    expAccs = actVelo / actAccl
    expAccl = actVelo / actAccs
    print(
        "%s expAccl=%f expAccs=%f actVelo=%f actAccl=%f actAccs=%f"
        % (tc_no, expAccl, expAccs, actVelo, actAccl, actAccs)
    )
    assert self.axisMr.calcAlmostEqual(tc_no, expAccEGU, resAccEGU, 0.1)
    self.assertEqual(1, done, "moveWait should return done")

    # Check if VELO, ACCL and ACCS are aligned
    assert self.axisMr.calcAlmostEqual(tc_no, expAccl, actAccl, 0.1)
    assert self.axisMr.calcAlmostEqual(tc_no, expAccs, actAccs, 0.1)


class Test(unittest.TestCase):
    url_string = os.getenv("TESTEDMOTORAXIS")
    print("url_string=%s" % (url_string))

    axisCom = AxisCom(url_string, log_debug=True)
    axisMr = AxisMr(axisCom)

    # self.axisCom.put('-DbgStrToLOG', "Start " + os.path.basename(__file__)[0:20])
    hlm = axisCom.get(".HLM")
    llm = axisCom.get(".LLM")

    per10_UserPosition = round((9 * llm + 1 * hlm) / 10)
    per20_UserPosition = round((8 * llm + 2 * hlm) / 10)
    msta = int(axisCom.get(".MSTA"))
    accs = axisCom.get(".ACCS")
    homedAndPwrAndACCS = (
        (accs != None)
        and (msta & axisMr.MSTA_BIT_HOMED)
        and (msta & axisMr.MSTA_BIT_AMPON)
    )

    # Assert that motor is homed and has the ACCS field
    def test_TC_411(self):
        tc_no = "TC-411"
        if not (self.homedAndPwrAndACCS):
            self.assertNotEqual(
                self.msta & self.axisMr.MSTA_BIT_HOMED, 0, "Axis has been homed"
            )
            self.assertNotEqual(
                self.msta & self.axisMr.MSTA_BIT_AMPON, 0, "Amplifier is on"
            )
            self.assertNotEqual(self.accs, None, "ACCS field in record")

    # 10% dialPosition
    def test_TC_412(self):
        tc_no = "TC-412-10-percent"
        print("%s" % tc_no)
        if self.msta & self.axisMr.MSTA_BIT_HOMED:
            done = self.axisMr.moveWait(tc_no, self.per10_UserPosition)
            print("%s done=%s destination=%f" % (tc_no, done, self.per10_UserPosition))
            self.assertEqual(1, done, "moveWait should return done")

    def test_TC_41311(self):
        tc_no = "TC-41311"
        print("%s" % tc_no)
        if self.homedAndPwrAndACCS:
            #                                            vbas, velo. accl, accs, expAccEGU
            check_VBAS_VELO_ACCL_ACCS_accEGU(self, tc_no, 0, 6.0, 0.2, -1, 30)

    def test_TC_41312(self):
        tc_no = "TC-41312"
        print("%s" % tc_no)
        if self.homedAndPwrAndACCS:
            #                                           vbas, velo. accl, accs, expAccEGU
            check_VBAS_VELO_ACCL_ACCS_accEGU(self, tc_no, 1, 2.0, 0.2, -1, 5)

    def test_TC_41313(self):
        tc_no = "TC-41313"
        print("%s" % tc_no)
        if self.homedAndPwrAndACCS:
            #                                             vbas, velo. accl, accs, expAccEGU
            check_VBAS_VELO_ACCL_ACCS_accEGU(self, tc_no, 1, 2.0, 0.4, -1, 2.5)

    def test_TC_41314(self):
        tc_no = "TC-41314"
        print("%s" % tc_no)
        if self.homedAndPwrAndACCS:
            #                                             vbas, velo. accl, accs, expAccEGU
            check_VBAS_VELO_ACCL_ACCS_accEGU(self, tc_no, 4.0, 4.0, 0.5, -1, 8.0)

    def test_TC_41315(self):
        tc_no = "TC-41315"
        print("%s" % tc_no)
        if self.homedAndPwrAndACCS:
            #                                             vbas, velo. accl, accs, expAccEGU
            check_VBAS_VELO_ACCL_ACCS_accEGU(self, tc_no, 0.0, 8.0, 0.5, -1, 16.0)

    def test_TC_41316(self):
        tc_no = "TC-41316"
        print("%s" % tc_no)
        if self.homedAndPwrAndACCS:
            #                                             vbas, velo. accl, accs, expAccEGU
            check_VBAS_VELO_ACCL_ACCS_accEGU(self, tc_no, 0.0, 8.0, -1.0, 16.0, 16.0)

    # Keep ACCS and expAccEGU if velociy is changed
    def test_TC_41317(self):
        tc_no = "TC-41317"
        print("%s" % tc_no)
        if self.homedAndPwrAndACCS:
            #                                             vbas, velo. accl, accs, expAccEGU
            check_VBAS_VELO_ACCL_ACCS_accEGU(self, tc_no, 0.0, 4.0, -1.0, -1.0, 16.0)
