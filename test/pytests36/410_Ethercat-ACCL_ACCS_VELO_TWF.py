#!/usr/bin/env python

import datetime
import unittest
import os
import sys
from AxisMr import AxisMr
from AxisCom import AxisCom

import time

filnam = "410xx.py"
###


def getAccEGUfromMCU(self, tc_no):
    print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no}: getAccEGUfromMCU")
    res = self.axisCom.get("-Acc-RB")
    return res


def check_VBAS_VELO_ACCL_ACCS_accEGU(self, tc_no, vbas, velo, accl, accs, expAccEGU):
    self.axisCom.put("-DbgStrToLOG", "Start " + str(tc_no))

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
    self.axisMr.moveWait(tc_no, destination)
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
    accOK = self.axisMr.calcAlmostEqual(tc_no, expAccEGU, resAccEGU, 0.1)
    acclOK = self.axisMr.calcAlmostEqual(tc_no, expAccl, actAccl, 0.1)
    accsOK = self.axisMr.calcAlmostEqual(tc_no, expAccs, actAccs, 0.1)

    testPassed = accOK and acclOK and accsOK
    print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no} accOK={accOK} acclOK={acclOK} accsOK={accsOK}")
    if testPassed:
        self.axisCom.put("-DbgStrToLOG", "Passed " + str(tc_no))
    else:
        self.axisCom.put("-DbgStrToLOG", "Failed " + str(tc_no))
    assert testPassed


class Test(unittest.TestCase):
    url_string = os.getenv("TESTEDMOTORAXIS")
    print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} url_string={url_string}")

    axisCom = AxisCom(url_string, log_debug=True)
    axisMr = AxisMr(axisCom)

    # self.axisCom.put('-DbgStrToLOG', "Start " + os.path.basename(__file__)[0:20])
    hlm = axisCom.get(".HLM")
    llm = axisCom.get(".LLM")

    per10_UserPosition = round((9 * llm + 1 * hlm) / 10)
    per20_UserPosition = round((8 * llm + 2 * hlm) / 10)
    msta = int(axisCom.get(".MSTA"))
    vers = float(axisCom.get(".VERS"))
    if vers >= 6.94 and vers <= 7.09:
        hasACCSfield = True
    else:
        hasACCSfield = False

    # Make sure that motor is homed
    def test_TC_4101(self):
        tc_no = "4101"
        if not (self.msta & self.axisMr.MSTA_BIT_HOMED):
            self.axisMr.powerOnHomeAxis(tc_no)
            self.msta = int(self.axisCom.get(".MSTA"))
            self.assertNotEqual(
                0,
                self.msta & self.axisMr.MSTA_BIT_HOMED,
                "MSTA.homed (Axis is not homed)",
            )

    # 10% dialPosition
    def test_TC_4102(self):
        tc_no = "4102"
        print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no}")
        if self.msta & self.axisMr.MSTA_BIT_HOMED:
            self.axisMr.moveWait(tc_no, self.per10_UserPosition)
            print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no} destination={self.per10_UserPosition:f}")

    def test_TC_4103(self):
        tc_no = "4103"
        print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no}")
        if self.hasACCSfield:
            #                                            vbas, velo. accl, accs, expAccEGU
            check_VBAS_VELO_ACCL_ACCS_accEGU(self, tc_no, 0, 6.0, 0.2, -1, 30)

    def test_TC_4104(self):
        tc_no = "4104"
        print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no}")
        if self.hasACCSfield:
            #                                           vbas, velo. accl, accs, expAccEGU
            check_VBAS_VELO_ACCL_ACCS_accEGU(self, tc_no, 1, 2.0, 0.2, -1, 5)

    def test_TC_4105(self):
        tc_no = "4105"
        print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no}")
        if self.hasACCSfield:
            #                                             vbas, velo. accl, accs, expAccEGU
            check_VBAS_VELO_ACCL_ACCS_accEGU(self, tc_no, 1, 2.0, 0.4, -1, 2.5)

    def test_TC_4106(self):
        tc_no = "4106"
        print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no}")
        if self.hasACCSfield:
            #                                             vbas, velo. accl, accs, expAccEGU
            check_VBAS_VELO_ACCL_ACCS_accEGU(self, tc_no, 4.0, 4.0, 0.5, -1, 8.0)

    def test_TC_4107(self):
        tc_no = "4107"
        print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no}")
        if self.hasACCSfield:
            #                                             vbas, velo. accl, accs, expAccEGU
            check_VBAS_VELO_ACCL_ACCS_accEGU(self, tc_no, 0.0, 8.0, 0.5, -1, 16.0)

    def test_TC_4108(self):
        tc_no = "4108"
        print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no}")
        if self.hasACCSfield:
            #                                             vbas, velo. accl, accs, expAccEGU
            check_VBAS_VELO_ACCL_ACCS_accEGU(self, tc_no, 0.0, 8.0, -1.0, 16.0, 16.0)

    # Keep ACCS and expAccEGU if velociy is changed
    def test_TC_4109(self):
        tc_no = "4109"
        print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no}")
        if self.hasACCSfield:
            #                                             vbas, velo. accl, accs, expAccEGU
            check_VBAS_VELO_ACCL_ACCS_accEGU(self, tc_no, 0.0, 4.0, -1.0, -1.0, 16.0)
