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


class Test(unittest.TestCase):
    url_string = os.getenv("TESTEDMOTORAXIS")
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} url_string={url_string}"
    )

    axisCom = AxisCom(url_string, log_debug=True)
    axisMr = AxisMr(axisCom)

    # self.axisCom.put('-DbgStrToLOG', "Start " + os.path.basename(__file__)[0:20], wait=True)
    hlm = axisCom.get(".HLM")
    llm = axisCom.get(".LLM")

    per10_UserPosition = round((9 * llm + 1 * hlm) / 10)
    per20_UserPosition = round((8 * llm + 2 * hlm) / 10)
    msta = int(axisCom.get(".MSTA"))

    def getAcceleration(self, tc_no):
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no}: getAcceleration"
        )
        res = self.axisCom.get("-Acc-RB")
        return res

    # Make sure that motor is homed
    def test_TC_4001(self):
        tc_no = "4001"
        self.axisMr.powerOnHomeAxis(tc_no)

    # 10% dialPosition
    def test_TC_4002(self):
        tc_no = "4002"
        self.axisCom.putDbgStrToLOG("Start " + tc_no, wait=True)
        if self.msta & self.axisMr.MSTA_BIT_HOMED:
            self.axisMr.moveWait(tc_no, self.per10_UserPosition)
        self.axisCom.putDbgStrToLOG("End " + tc_no, wait=True)

    # 20% dialPosition
    def test_TC_4003(self):
        tc_no = "4003"
        self.axisCom.putDbgStrToLOG("Start " + tc_no, wait=True)
        if self.msta & self.axisMr.MSTA_BIT_HOMED:
            saved_ACCL = float(self.axisCom.get(".ACCL"))
            saved_VELO = float(self.axisCom.get(".VELO"))
            used_ACCL = saved_ACCL + 1.0  # Make sure we have an acceleration != 0
            self.axisCom.put(".ACCL", used_ACCL)
            self.axisMr.moveWait(tc_no, self.per20_UserPosition)
            resacc = self.getAcceleration(tc_no)
            expacc = saved_VELO / used_ACCL
            self.axisCom.put(".ACCL", saved_ACCL)
            testPassed = self.axisMr.calcAlmostEqual(tc_no, expacc, resacc, 2)
            print(
                f"{tc_no} ACCL={used_ACCL:f} expacc={expacc:f} resacc={resacc:f} testPassed=[testPassed]"
            )
            if testPassed:
                self.axisCom.putDbgStrToLOG("Passed " + str(tc_no), wait=True)
            else:
                self.axisCom.putDbgStrToLOG("Failed " + str(tc_no), wait=True)
            assert testPassed
        else:
            self.axisCom.putDbgStrToLOG("End " + str(tc_no), wait=True)

    # JOGR
    def test_TC_4004(self):
        tc_no = "4004"
        print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no}")
        if self.msta & self.axisMr.MSTA_BIT_HOMED:
            self.axisCom.putDbgStrToLOG("Start " + str(tc_no), wait=True)
            accl = float(self.axisCom.get(".ACCL"))
            jvel = float(self.axisCom.get(".JVEL"))
            saved_JAR = float(self.axisCom.get(".JAR"))
            used_JAR = jvel / (accl + 2.0)
            self.axisCom.put(".JAR", used_JAR)
            self.axisCom.put(".JOGR", 1)
            self.axisMr.waitForStart(tc_no, 2.0)

            resacc = self.getAcceleration(tc_no)
            expacc = used_JAR
            self.axisCom.put(".JOGR", 0)
            # Wait depending on JVEL (and positions)
            time_to_wait = (self.per20_UserPosition - self.llm) / jvel + 2 * accl + 1.0
            self.axisMr.waitForStop(tc_no, time_to_wait)
            self.axisCom.put(".JAR", saved_JAR)
            testPassed = self.axisMr.calcAlmostEqual(tc_no, expacc, resacc, 2)
            print(
                f"{tc_no} JAR={used_JAR:f} expacc={expacc:f} resacc={resacc:f} testPassed={testPassed}"
            )
            if testPassed:
                self.axisCom.putDbgStrToLOG("Passed " + str(tc_no), wait=True)
            else:
                self.axisCom.putDbgStrToLOG("Failed " + str(tc_no), wait=True)
            assert testPassed
        else:
            self.axisCom.putDbgStrToLOG("End " + str(tc_no), wait=True)

    def teardown_class(self):
        tc_no = int(filnam) * 10000 + 9999
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} {tc_no} teardown_class"
        )
        self.axisCom.close()
