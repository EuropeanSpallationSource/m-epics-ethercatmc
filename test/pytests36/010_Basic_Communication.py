#!/usr/bin/env python

# EPICS/TwinCAT test
# This test does not test anything "real".
# It checks that (depending on the URL)
# - the connecttion to plc is working
# - channel access is working
# - pv access is working
# All python libraries/packages are installed
# The (motor) PV  does exist
# Or the PLC can be connected via ADS

import os
import unittest

from AxisCom import AxisCom

###


class Test(unittest.TestCase):

    url_string = os.getenv("TESTEDMOTORAXIS")
    print(f"url_string={url_string}")

    axisCom = AxisCom(url_string, log_debug=True)

    # Read variables (boolean and floating point)
    def test_TC_01001(self):
        url_string = self.url_string
        axisCom = self.axisCom

        tc_no = "TC-01001"
        rbv = None
        enabled = None
        print(f"{tc_no} Test RBV/CNEN url_string={url_string}")
        rbv = axisCom.get(".RBV")
        enabled = axisCom.get(".CNEN")

        print(f"{tc_no}/{url_string} rbv={rbv} enabled={int(enabled)}")

        self.assertNotEqual(None, rbv, "rbv must not be None")
        self.assertNotEqual(None, enabled, "enabled must not be None")

    # Read and write variables (boolean and floating point)
    def test_TC_01002(self):
        url_string = self.url_string
        axisCom = self.axisCom

        tc_no = "TC-01002"
        velo = None
        enabled = None
        veloPut = None
        enabledPut = None
        print(f"{tc_no} Test VELO/CNEN put url_string={url_string}")
        had_ex = False
        try:
            velo = axisCom.get(".VELO")
            veloPut = axisCom.put(".VELO", velo)
            enabled = axisCom.get(".CNEN")
            enabledPut = axisCom.put(".CNEN", enabled)

            print(
                "%s/%s velo=%s enabled=%d veloPut=%s enabledPut=%s"
                % (tc_no, url_string, velo, enabled, veloPut, enabledPut)
            )
        except:
            had_ex = True
        self.assertEqual(False, had_ex, "The should not have been an exception")

    # (Try to) write non read only a variable
    # (floating point)
    # Do expect an exception
    def test_TC_01003(self):
        url_string = self.url_string
        axisCom = self.axisCom

        tc_no = "TC-01003"
        floatGetEx = None
        floatPutEx = None
        print(f"{tc_no} Test FLOAT/actual non existing url_string={url_string}")
        value = 0.0
        try:
            floatGet = axisCom.get(".RBV")
        except Exception as ex:
            floatGetEx = ex
        try:
            floatPut = axisCom.put(".RBV", value)
        except Exception as ex:
            floatPutEx = ex
        print(f"{tc_no}/{url_string} floatGetEx={floatGetEx} floatPutEx={floatPutEx}")
        self.assertEqual(None, floatGetEx, "floatGetEx must be None")
        self.assertNotEqual(None, floatPutEx, "floatPutEx must not be None")

    # (Try to) read and write non existing variables
    # (boolean and floating point)
    # Do expect an exception
    def test_TC_01004(self):
        url_string = self.url_string
        axisCom = self.axisCom

        tc_no = "TC-01004"
        floatGetEx = None
        floatPutEx = None
        print(f"{tc_no} Test FLOAT/actual non existing url_string={url_string}")
        value = 0.0
        try:
            floatGet = axisCom.get(".fNONE")
        except Exception as ex:
            floatGetEx = ex
        try:
            floatPut = axisCom.put(".fNONE", value)
        except Exception as ex:
            floatPutEx = ex
        print(f"{tc_no}/{url_string} floatGetEx={floatGetEx} floatPutEx={floatPutEx}")
        self.assertNotEqual(None, floatGetEx, "floatGetEx must not be None")
        self.assertNotEqual(None, floatPutEx, "floatPutEx must not be None")
