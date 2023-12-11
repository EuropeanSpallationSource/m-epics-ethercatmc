#!/usr/bin/env python

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


def getAccEGUfromMCU(self, tc_no):
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no}: getAccEGUfromMCU"
    )
    res = self.axisCom.get("-Acc-RB")
    return res


def check_VBAS_VELO_ACCL_ACCS_accEGU(self, tc_no, vbas, velo, accl, accs, expAccEGU):
    self.axisCom.putDbgStrToLOG("Start " + str(tc_no), wait=True)

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
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no} accOK={accOK} acclOK={acclOK} accsOK={accsOK}"
    )
    if testPassed:
        self.axisCom.putDbgStrToLOG("Passed " + str(tc_no), wait=True)
    else:
        self.axisCom.putDbgStrToLOG("Failed " + str(tc_no), wait=True)
    assert testPassed


class Test(unittest.TestCase):
    url_string = os.getenv("TESTEDMOTORAXIS")
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} url_string={url_string}"
    )

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
    def test_TC_9311(self):
        tc_no = "9311"
        self.axisMr.powerOnHomeAxis(tc_no)

    # 10% dialPosition
    def test_TC_9312(self):
        tc_no = "9312"
        print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no}")
        if self.msta & self.axisMr.MSTA_BIT_HOMED:
            self.axisMr.moveWait(tc_no, self.per10_UserPosition)
            print(
                f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no} destination={self.per10_UserPosition:f}"
            )

    def test_TC_9313(self):
        tc_no = "9313"
        print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no}")
        if self.hasACCSfield:
            #                                            vbas, velo. accl, accs, expAccEGU
            check_VBAS_VELO_ACCL_ACCS_accEGU(self, tc_no, 0, 6.0, 0.2, -1, 30)

    def test_TC_9314(self):
        tc_no = "9314"
        print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no}")
        if self.hasACCSfield:
            #                                           vbas, velo. accl, accs, expAccEGU
            check_VBAS_VELO_ACCL_ACCS_accEGU(self, tc_no, 1, 2.0, 0.2, -1, 5)

    def test_TC_9315(self):
        tc_no = "9315"
        print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no}")
        if self.hasACCSfield:
            #                                             vbas, velo. accl, accs, expAccEGU
            check_VBAS_VELO_ACCL_ACCS_accEGU(self, tc_no, 1, 2.0, 0.4, -1, 2.5)

    def test_TC_9316(self):
        tc_no = "9316"
        print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no}")
        if self.hasACCSfield:
            #                                             vbas, velo. accl, accs, expAccEGU
            check_VBAS_VELO_ACCL_ACCS_accEGU(self, tc_no, 4.0, 4.0, 0.5, -1, 8.0)

    def test_TC_9317(self):
        tc_no = "9317"
        print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no}")
        if self.hasACCSfield:
            #                                             vbas, velo. accl, accs, expAccEGU
            check_VBAS_VELO_ACCL_ACCS_accEGU(self, tc_no, 0.0, 8.0, 0.5, -1, 16.0)

    def test_TC_9318(self):
        tc_no = "9318"
        print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no}")
        if self.hasACCSfield:
            #                                             vbas, velo. accl, accs, expAccEGU
            check_VBAS_VELO_ACCL_ACCS_accEGU(self, tc_no, 0.0, 8.0, -1.0, 16.0, 16.0)

    # Keep ACCS and expAccEGU if velocity is changed
    def test_TC_9319(self):
        tc_no = "9319"
        print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no}")
        if self.hasACCSfield:
            #                                             vbas, velo. accl, accs, expAccEGU
            check_VBAS_VELO_ACCL_ACCS_accEGU(self, tc_no, 0.0, 4.0, -1.0, -1.0, 16.0)

    def teardown_class(self):
        tc_no = int(filnam) * 10000 + 9999
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} {tc_no} teardown_class"
        )
        self.axisCom.close()
