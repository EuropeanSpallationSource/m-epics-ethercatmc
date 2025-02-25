#!/usr/bin/env python
#

import datetime
import inspect
import unittest
import os
from AxisMr import AxisMr
from AxisCom import AxisCom

filnam = os.path.basename(__file__)[0:3]


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

    saved_DLY = axisCom.get(".DLY")
    hlm = axisCom.get(".HLM")
    llm = axisCom.get(".LLM")
    jvel = axisCom.get(".JVEL")

    margin = 2.1
    # motorRecord stops jogging 1 second before reaching HLM
    jog_start_pos = hlm - jvel - margin

    msta = int(axisCom.get(".MSTA"))

    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} llm={llm:.2f} hlm={hlm:.2f} jog_start_pos={jog_start_pos:.2f}"
    )

    # Make sure that motor is homed
    def test_TC_1211(self):
        tc_no = "1211"
        self.axisCom.putDbgStrToLOG("Start " + str(int(tc_no)), wait=True)
        self.axisMr.powerOnHomeAxis(tc_no)
        self.axisMr.setSoftLimitsOn(tc_no, initAbsMinMax=True)
        self.axisCom.putDbgStrToLOG("Passed " + str(tc_no), wait=True)

    # close-toHLM
    def test_TC_1212(self):
        tc_no = "1212"
        self.axisCom.putDbgStrToLOG("Start " + str(int(tc_no)), wait=True)
        if self.msta & self.axisMr.MSTA_BIT_HOMED:
            print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no}")
            testPassed = self.axisMr.moveWait(tc_no, self.jog_start_pos)
            UserPosition = self.axisCom.get(".RBV", use_monitor=False)
            print(
                f"{tc_no} postion={UserPosition:.2f} jog_start_pos={self.jog_start_pos:.2f}"
            )
            if testPassed:
                self.axisCom.putDbgStrToLOG("passed " + str(tc_no), wait=True)
            else:
                self.axisCom.putDbgStrToLOG("Failed " + str(tc_no), wait=True)
            assert testPassed

    # High soft limit JOGF
    def test_TC_1213(self):
        tc_no = "1213"
        self.axisCom.putDbgStrToLOG("Start " + str(int(tc_no)), wait=True)
        if self.msta & self.axisMr.MSTA_BIT_HOMED:
            print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no}")
            self.axisCom.put(".DLY", 1.0)
            testPassed = self.axisMr.jogDirection(tc_no, 1)
            lvio = int(self.axisCom.get(".LVIO"))
            msta = int(self.axisCom.get(".MSTA"))
            miss = int(self.axisCom.get(".MISS"))

            self.axisCom.put(".DLY", self.saved_DLY)
            self.axisMr.waitForMipZero(tc_no, self.saved_DLY)

            if msta & self.axisMr.MSTA_BIT_PROBLEM:
                testPassed = False
            if msta & self.axisMr.MSTA_BIT_MINUS_LS:
                testPassed = False
            if msta & self.axisMr.MSTA_BIT_PLUS_LS:
                testPassed = False
            if miss != 0:
                testPassed = False
            if lvio != 1:
                testPassed = False
            print(
                f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no} testPassed={testPassed} lvio={lvio} miss={miss} msta={self.axisMr.getMSTAtext(msta)}"
            )
            if testPassed:
                self.axisCom.putDbgStrToLOG("passed " + str(tc_no), wait=True)
            else:
                self.axisCom.putDbgStrToLOG("Failed " + str(tc_no), wait=True)
            assert testPassed

    # close-toHLM
    def test_TC_1214(self):
        tc_no = "1214"
        self.axisCom.putDbgStrToLOG("Start " + str(int(tc_no)), wait=True)
        if self.msta & self.axisMr.MSTA_BIT_HOMED:
            print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no}")
            testPassed = self.axisMr.moveWait(tc_no, self.jog_start_pos)
            UserPosition = self.axisCom.get(".RBV", use_monitor=False)
            print(
                f"{tc_no} testPassed={testPassed} postion={UserPosition:.2f} jog_start_pos={self.jog_start_pos:.2f}"
            )
            if testPassed:
                self.axisCom.putDbgStrToLOG("passed " + str(tc_no), wait=True)
            else:
                self.axisCom.putDbgStrToLOG("Failed " + str(tc_no), wait=True)
            assert testPassed

    def test_TC_1215(self):
        tc_no = "1215"
        self.axisCom.putDbgStrToLOG("Start " + str(int(tc_no)), wait=True)
        if self.msta & self.axisMr.MSTA_BIT_HOMED:
            print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no}")
            self.axisCom.put(".DLY", 0.0)
            testPassed = self.axisMr.jogDirection(tc_no, 1)
            lvio = int(self.axisCom.get(".LVIO"))
            msta = int(self.axisCom.get(".MSTA"))
            miss = int(self.axisCom.get(".MISS"))

            self.axisCom.put(".DLY", self.saved_DLY)
            self.axisCom.put(".JOGF", 0)
            self.axisMr.waitForMipZero(tc_no, self.saved_DLY)
            if msta & self.axisMr.MSTA_BIT_PROBLEM:
                testPassed = False
            if msta & self.axisMr.MSTA_BIT_MINUS_LS:
                testPassed = False
            if msta & self.axisMr.MSTA_BIT_PLUS_LS:
                testPassed = False
            if miss != 0:
                testPassed = False
            if lvio != 1:
                testPassed = False
            print(
                f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no} testPassed={testPassed} lvio={lvio} miss={miss} msta={self.axisMr.getMSTAtext(msta)}"
            )
            if testPassed:
                self.axisCom.putDbgStrToLOG("passed " + str(tc_no), wait=True)
            else:
                self.axisCom.putDbgStrToLOG("Failed " + str(tc_no), wait=True)
            assert testPassed

    # High soft limt JOGF
    def test_TC_1216(self):
        tc_no = "1216"
        self.axisCom.putDbgStrToLOG("Start " + str(int(tc_no)), wait=True)
        if self.msta & self.axisMr.MSTA_BIT_HOMED:
            print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no}")
            self.axisCom.put(".DLY", 0.0)
            mip1 = int(self.axisCom.get(".MIP"))
            testPassed = self.axisMr.jogDirection(tc_no, 1)

            lvio = int(self.axisCom.get(".LVIO"))
            msta = int(self.axisCom.get(".MSTA"))
            miss = int(self.axisCom.get(".MISS"))
            self.axisMr.waitForMipZero(tc_no, self.saved_DLY)
            mip2 = int(self.axisCom.get(".MIP"))
            self.axisCom.put(".DLY", self.saved_DLY)
            print(
                f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no} mip1={mip1:x} mip2={mip2:x}"
            )
            if msta & self.axisMr.MSTA_BIT_PROBLEM:
                testPassed = False
            if msta & self.axisMr.MSTA_BIT_MINUS_LS:
                testPassed = False
            if msta & self.axisMr.MSTA_BIT_PLUS_LS:
                testPassed = False
            if miss != 0:
                testPassed = False
            if mip2 != 0:
                testPassed = False
            if lvio != 1:
                testPassed = False
            print(
                f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no} testPassed={testPassed} lvio={lvio} miss={miss} msta={self.axisMr.getMSTAtext(msta)}"
            )
            if testPassed:
                self.axisCom.putDbgStrToLOG("passed " + str(tc_no), wait=True)
            else:
                self.axisCom.putDbgStrToLOG("Failed " + str(tc_no), wait=True)
            assert testPassed

    # close-toHLM UserPosition
    def test_TC_1217(self):
        tc_no = "1217"
        self.axisCom.putDbgStrToLOG("Start " + str(int(tc_no)), wait=True)
        if self.msta & self.axisMr.MSTA_BIT_HOMED:
            print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no}")
            testPassed = self.axisMr.moveWait(tc_no, self.jog_start_pos)
            UserPosition = self.axisCom.get(".RBV", use_monitor=False)
            print(
                f"{tc_no} postion={UserPosition:.2f} jog_start_pos={self.jog_start_pos:.2f}"
            )
            if testPassed:
                self.axisCom.putDbgStrToLOG("passed " + str(tc_no), wait=True)
            else:
                self.axisCom.putDbgStrToLOG("Failed " + str(tc_no), wait=True)
            assert testPassed

    # High soft limit JOGR + DIR
    def test_TC_1218(self):
        tc_no = "1218"
        self.axisCom.putDbgStrToLOG("Start " + str(int(tc_no)), wait=True)
        if self.msta & self.axisMr.MSTA_BIT_HOMED:
            print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no}")
            saved_DIR = self.axisCom.get(".DIR")
            saved_FOFF = self.axisCom.get(".FOFF")
            self.axisCom.put(".FOFF", 1)
            self.axisCom.put(".DIR", 1)
            testPassed = self.axisMr.jogDirection(tc_no, 0)

            lvio = int(self.axisCom.get(".LVIO"))
            msta = int(self.axisCom.get(".MSTA"))
            miss = int(self.axisCom.get(".MISS"))
            self.axisMr.waitForMipZero(tc_no, self.saved_DLY)
            mip2 = int(self.axisCom.get(".MIP"))
            self.axisCom.put(".DIR", saved_DIR)
            self.axisCom.put(".FOFF", saved_FOFF)
            self.axisCom.put(".DLY", self.saved_DLY)
            print(
                f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no} mip2={mip2:x}"
            )
            if msta & self.axisMr.MSTA_BIT_PROBLEM:
                testPassed = False
            if msta & self.axisMr.MSTA_BIT_MINUS_LS:
                testPassed = False
            if msta & self.axisMr.MSTA_BIT_PLUS_LS:
                testPassed = False
            if miss != 0:
                testPassed = False
            if mip2 != 0:
                testPassed = False
            if lvio != 1:
                testPassed = False
            print(
                f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no} testPassed={testPassed} lvio={lvio} miss={miss} msta={self.axisMr.getMSTAtext(msta)}"
            )
            if testPassed:
                self.axisCom.putDbgStrToLOG("passed " + str(tc_no), wait=True)
            else:
                self.axisCom.putDbgStrToLOG("Failed " + str(tc_no), wait=True)
            assert testPassed

    def teardown_class(self):
        tc_no = int(filnam) * 10000 + 9999
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} {tc_no} teardown_class"
        )
        self.axisCom.close()
