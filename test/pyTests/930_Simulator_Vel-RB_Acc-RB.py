import unittest
import os
import sys
from AxisMr import AxisMr
from AxisCom import AxisCom

import time
import math
import inspect

###


def lineno():
    return inspect.currentframe().f_back.f_lineno


polltime = 0.1


def setAndReadBackParam(self, tc_no, field_name, paramInSimu):
    valRB = self.axisCom.get(field_name)
    newVal = round(valRB + 1, 2)

    print(f"{tc_no}:{lineno()} field_name={field_name} valRB={valRB} newVal={newVal}")

    self.axisMr.setValueOnSimulator(tc_no, paramInSimu, newVal)
    maxTime = 30  # 30 seconds maximum to poll all parameters
    testPassed = False
    maxDelta = 0.05  # 5 % error tolerance margin
    while maxTime > 0:
        newValRB = self.axisCom.get(field_name)
        print(
            "%s:%d %s newVal=%f newValRB=%f"
            % (tc_no, lineno(), field_name, newVal, newValRB)
        )

        if self.axisMr.calcAlmostEqual(tc_no, newVal, newValRB, maxDelta):
            testPassed = True
            maxTime = 0
        else:
            time.sleep(polltime)
            maxTime = maxTime - polltime

    # restore the original value
    # Avoid overlong strings (and therefore not working with channel access)
    # like "Sim.this.fAcceleration=2.0999999046325684"
    self.axisMr.setValueOnSimulator(tc_no, paramInSimu, round(valRB, 2))
    assert testPassed


class Test(unittest.TestCase):
    url_string = os.getenv("TESTEDMOTORAXIS")
    print("url_string=%s" % (url_string))

    axisCom = AxisCom(url_string, log_debug=False)
    axisMr = AxisMr(axisCom)

    axisCom.put("-DbgStrToLOG", "Start " + os.path.basename(__file__)[0:20])

    # Set and readback Vel
    def test_TC_9301(self):
        tc_no = "TC-9301"
        setAndReadBackParam(self, tc_no, "-Vel-RB", "fVelocity")

    # Set and readback Acc
    def test_TC_9302(self):
        tc_no = "TC-9302"
        setAndReadBackParam(self, tc_no, "-Acc-RB", "fAcceleration")

    # Set and readback high soft limit value
    def test_TC_9303(self):
        tc_no = "TC-9303"
        setAndReadBackParam(self, tc_no, "-CfgDHLM-RB", "fHighSoftLimitPos")

    ## Set and readback high soft limit enable
    # Cant run those, PILS has no enable bit
    # def test_TC_9304(self):
    #    tc_no = "TC-9304"
    #    setAndReadBackParam(self,  tc_no, '-CfgDHLM-En-RB', 'bEnableHighSoftLimit')

    # Set and readback low soft limit value
    def test_TC_9305(self):
        tc_no = "TC-9305"
        setAndReadBackParam(self, tc_no, "-CfgDLLM-RB", "fLowSoftLimitPos")

    ## Set and readback low soft limit enable
    # Cant run those, PILS has no enable bit
    # def test_TC_9306(self):
    #    tc_no = "TC-9306"
    #    setAndReadBackParam(self,  tc_no, '-CfgDLLM-En-RB', 'bEnableLowSoftLimit')
