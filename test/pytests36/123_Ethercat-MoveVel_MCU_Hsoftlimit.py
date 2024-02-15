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

    axisCom = AxisCom(url_string, log_debug=False)
    axisMr = AxisMr(axisCom)

    # self.axisCom.put('-DbgStrToLOG', "Start " + os.path.basename(__file__)[0:20], wait=True)

    hlm = axisCom.get(".HLM")
    llm = axisCom.get(".LLM")
    jvel = axisCom.get(".JVEL")

    margin = 1.1
    # motorRecord stops jogging 1 second before reaching HLM
    jog_start_pos = hlm - jvel - margin

    msta = int(axisCom.get(".MSTA"))

    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} llm={llm:f} hlm={hlm:f} jog_start_pos={jog_start_pos:f}"
    )

    # Assert that motor is homed
    def test_TC_1231(self):
        tc_no = "1231"
        self.axisMr.powerOnHomeAxis(tc_no)

    # per90 UserPosition
    def test_TC_1232(self):
        tc_no = "1232"
        self.axisCom.putDbgStrToLOG("Start " + str(int(tc_no)), wait=True)
        if self.msta & self.axisMr.MSTA_BIT_HOMED:
            print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no}")
            self.axisMr.moveWait(tc_no, self.jog_start_pos)
            UserPosition = self.axisCom.get(".RBV", use_monitor=False)
            print(
                f"{tc_no} postion={UserPosition:f} jog_start_pos={self.jog_start_pos:f}"
            )

    # High soft limit in controller when using MoveVel
    def test_TC_1233(self):
        tc_no = "1233"
        self.axisCom.putDbgStrToLOG("Start " + str(int(tc_no)), wait=True)
        if self.msta & self.axisMr.MSTA_BIT_HOMED:
            print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no}")

            jar = self.axisCom.get(".JAR")
            self.axisCom.put("-ACCS", jar)

            rbv = self.axisCom.get(".RBV")
            destination = self.axisCom.get(".HLM") + 1
            jvel = self.axisCom.get(".JVEL")
            timeout = self.axisMr.calcTimeOut(destination, jvel)
            print(
                f"{tc_no} rbv={rbv:f} destination={destination:f} timeout={timeout:f}"
            )
            self.axisCom.put("-MoveVel", jvel)
            # TODO: The -MoveVel PV is not always there ?
            # Investigations needed
            # if (res == None):
            #    print('%s caput -MoveVel res=None' % (tc_no))
            #    self.assertNotEqual(res, None, 'caput -MoveVel retuned not None. PV not found ?')
            # else:
            #    print('%s caput -MoveVel res=%d' % (tc_no, res))
            #    self.assertEqual(res, 1, 'caput -MoveVel returned 1')

            self.axisMr.waitForStartAndDone(tc_no, timeout)

            msta = int(self.axisCom.get(".MSTA"))
            miss = int(self.axisCom.get(".MISS"))

            if msta & self.axisMr.MSTA_BIT_PROBLEM:
                self.axisMr.resetAxis(tc_no)

            self.assertEqual(
                0,
                msta & self.axisMr.MSTA_BIT_MINUS_LS,
                "DLY Minus hard limit not reached MoveVel",
            )
            self.assertEqual(
                0,
                msta & self.axisMr.MSTA_BIT_PLUS_LS,
                "DLY Plus hard limit not reached MoveVel",
            )
            self.assertEqual(0, miss, "DLY MISS not set MoveVel")

    # per90 UserPosition
    def test_TC_1234(self):
        tc_no = "1234"
        self.axisCom.putDbgStrToLOG("Start " + str(int(tc_no)), wait=True)
        if self.msta & self.axisMr.MSTA_BIT_HOMED:
            print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no}")
            self.axisMr.moveWait(tc_no, self.jog_start_pos)
            UserPosition = self.axisCom.get(".RBV", use_monitor=False)
            print(
                f"{tc_no} postion={UserPosition:f} jog_start_pos={self.jog_start_pos:f}"
            )

    # High soft limit in controller when using MoveAbs
    def test_TC_1235(self):
        tc_no = "1235"
        self.axisCom.putDbgStrToLOG("Start " + str(int(tc_no)), wait=True)
        if self.msta & self.axisMr.MSTA_BIT_HOMED:
            print(
                f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no}: Start"
            )
            drvUseEGU = self.axisCom.get("-DrvUseEGU-RB")
            if drvUseEGU == 1:
                mres = 1.0
            else:
                mres = self.axisCom.get(".MRES")
            rbv = self.axisCom.get(".RBV")

            jar = self.axisCom.get(".JAR")
            self.axisCom.put("-ACCS", jar / mres)

            jvel = self.axisCom.get(".JVEL")
            self.axisCom.put("-VELO", jvel / mres)

            destination = self.hlm + 1
            timeout = self.axisMr.calcTimeOut(destination, jvel)
            print(
                f"{tc_no}: rbv={rbv:f} destination={destination:f} timeout={timeout:f}"
            )

            self.axisCom.put("-MoveAbs", (destination) / mres)
            # if (res == None):
            #    print('%s caput -Moveabs res=None' % (tc_no))
            #    self.assertNotEqual(res, None, 'caput -Moveabs retuned not None. PV not found ?')
            # else:
            #    print('%s caput -Moveabs res=%d' % (tc_no, res))
            #    self.assertEqual(res, 1, 'caput -Moveabs returned 1')

            self.axisMr.waitForStartAndDone(tc_no, timeout)

            msta = int(self.axisCom.get(".MSTA"))
            miss = int(self.axisCom.get(".MISS"))
            self.axisMr.verifyRBVinsideRDBD(tc_no, destination)

            if msta & self.axisMr.MSTA_BIT_PROBLEM:
                self.axisMr.resetAxis(tc_no)
            # TODO: Check error; errorId

            self.assertEqual(
                0,
                msta & self.axisMr.MSTA_BIT_MINUS_LS,
                "DLY Minus hard limit not reached Moveabs",
            )
            self.assertEqual(
                0,
                msta & self.axisMr.MSTA_BIT_PLUS_LS,
                "DLY Plus hard limit not reached Moveabs",
            )
            self.assertEqual(0, miss, "DLY MISS not set Moveabs")

    def teardown_class(self):
        tc_no = int(filnam) * 10000 + 9999
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} {tc_no} teardown_class"
        )
        self.axisCom.close()
