#!/usr/bin/env python
#
import unittest
import os
import sys
import time
from motor_lib import motor_lib
lib = motor_lib()
import capv_lib
###

myFRAC   = 1.5

motorRMOD_D = 0 # "Default"
motorRMOD_A = 1 # "Arithmetic"
motorRMOD_G = 2 # "Geometric"
motorRMOD_I = 3 # "In-Position"

#
#How we move: Absolute (without encoder) or relative (with encode via UEIP)
use_abs = 0
use_rel = 1
# Note: motorRMOD_I is always absolute !




def motorInitTC(tself, motor, tc_no, rmod, encRel):
    capv_lib.capvput(motor + '.RMOD', rmod)
    capv_lib.capvput(motor + '.UEIP', encRel)


def setMotorStartPos(tself, motor, tc_no, startpos):
    lib.setValueOnSimulator(motor, tc_no, "fActPosition", startpos)
    # Run a status update and a sync
    # Run a status update and a sync
    lib.doSTUPandSYNC(motor, tc_no)




def positionAndBacklash(tself, motor, tc_no, rmod, encRel, motorStartPos, motorEndPos):
    ###########
    # expected and actual
    if motor.startswith('pva://'):
        mot = motor[6:]
    else:
        mot = motor
    fileName = "/tmp/" + mot.replace(':', '-') + "-" + str(tc_no)
    expFileName = fileName + ".exp"
    actFileName = fileName + ".act"
    dbgFileName = fileName + ".dbg"

    motorInitTC(tself, motor, tc_no, rmod, encRel)
    setMotorStartPos(tself, motor, tc_no, motorStartPos)
    lib.setValueOnSimulator(motor, tc_no, "bManualSimulatorMode", 1)
    #time.sleep(2)
    lib.setValueOnSimulator(motor, tc_no, "log", actFileName)
    #time.sleep(2)
    #
    capv_lib.capvput(motor + '.VAL', motorEndPos, wait=True)

    # Create a "expected" file
    expFile=open(expFileName, 'w')
    if dbgFileName != None:
        dbgFile=open(dbgFileName, 'w')
    else:
        dbgFile = None
    # Positioning
    # 2 different ways to move:
    # - Within the backlash distance and into the backlash direction:
    #   single move with back lash parameters
    # - against the backlash direction -or- bigger than the backlash distance:
    #   two moves, first with moving, second with backlash parameters

    cnt = 1 + int(capv_lib.capvget(motor + '.RTRY'))
    lib.writeExpFileRMOD_X(motor, tc_no, rmod, dbgFile, expFile, cnt, myFRAC, encRel, motorStartPos, motorEndPos)

    expFile.close()
    if dbgFileName != None:
        dbgFile.close()
    #lib.setValueOnSimulator(motor, tc_no, "dbgCloseLogFile", "1")

    time_to_wait = 100
    lib.waitForStop(motor, tc_no, time_to_wait)
    lib.setValueOnSimulator(motor, tc_no, "dbgCloseLogFile", "1")
    #time.sleep(2)
    lib.setValueOnSimulator(motor, tc_no, "bManualSimulatorMode", 0)
    lib.cmpUnlinkExpectedActualFile(tc_no, expFileName, actFileName)



class Test(unittest.TestCase):
    motor = os.getenv("TESTEDMOTORAXIS")
    capv_lib.capvput(motor + '-DbgStrToLOG', "Start " + os.path.basename(__file__)[0:20])
    myPOSlow = lib.myPOSlow
    myPOSmid = lib.myPOSmid
    myPOShig = lib.myPOShig

    def test_TC_92000(self):
        lib.motorInitAllForBDST(self.motor, 92000)
        capv_lib.capvput(self.motor + '.FRAC', myFRAC)


    # motorRMOD_D = 0 # "Default"
    # position forward & backlash compensation, absolute
    def test_TC_92001(self):
        positionAndBacklash(self, self.motor, 92001, motorRMOD_D, use_abs, self.myPOSlow, self.myPOShig)

    # position forward & backlash compensation, relative
    def test_TC_92002(self):
        positionAndBacklash(self, self.motor, 92002, motorRMOD_D, use_rel, self.myPOSlow, self.myPOShig)

    # position backward & backlash compensation, absolute
    def test_TC_92003(self):
        positionAndBacklash(self, self.motor, 92003, motorRMOD_D, use_abs, self.myPOShig, self.myPOSlow)

    # position backward & backlash compensation, relative
    def test_TC_92004(self):
        positionAndBacklash(self, self.motor, 92004, motorRMOD_D, use_rel, self.myPOShig, self.myPOSlow)

    # position forward inside backlash range, absolute
    def test_TC_92005(self):
        positionAndBacklash(self, self.motor, 92005, motorRMOD_D, use_abs, self.myPOSmid, self.myPOSlow)

    # position forward inside backlash range, relative
    def test_TC_92006(self):
        positionAndBacklash(self, self.motor, 92006, motorRMOD_D, use_rel, self.myPOSmid, self.myPOSlow)

    # position forward inside backlash range, absolute
    def test_TC_92007(self):
        positionAndBacklash(self, self.motor, 92007, motorRMOD_D, use_abs, self.myPOSlow, self.myPOSmid)

    # position forward inside backlash range, relative
    def test_TC_92008(self):
        positionAndBacklash(self, self.motor, 92008, motorRMOD_D, use_rel, self.myPOSlow, self.myPOSmid)

    ###############################################################################
    # motorRMOD_A
    # position forward & backlash compensation, absolute
    def test_TC_92011(self):
        positionAndBacklash(self, self.motor, 92011, motorRMOD_A, use_abs, self.myPOSlow, self.myPOShig)

    # position forward & backlash compensation, relative
    def test_TC_92012(self):
        positionAndBacklash(self, self.motor, 92012, motorRMOD_A, use_rel, self.myPOSlow, self.myPOShig)

    # position backward & backlash compensation, absolute
    def test_TC_92013(self):
        positionAndBacklash(self, self.motor, 92013, motorRMOD_A, use_abs, self.myPOShig, self.myPOSlow)

    # position backward & backlash compensation, relative
    def test_TC_92014(self):
        positionAndBacklash(self, self.motor, 92014, motorRMOD_A, use_rel, self.myPOShig, self.myPOSlow)

    # position forward inside backlash range, absolute
    def test_TC_92015(self):
        positionAndBacklash(self, self.motor, 92015, motorRMOD_A, use_abs, self.myPOSmid, self.myPOSlow)

    # position forward inside backlash range, relative
    def test_TC_92016(self):
        positionAndBacklash(self, self.motor, 92016, motorRMOD_A, use_rel, self.myPOSmid, self.myPOSlow)

    # position forward inside backlash range, absolute
    def test_TC_92017(self):
        positionAndBacklash(self, self.motor, 92017, motorRMOD_A, use_abs, self.myPOSlow, self.myPOSmid)

    # position forward inside backlash range, relative
    def test_TC_92018(self):
        positionAndBacklash(self, self.motor, 92018, motorRMOD_A, use_rel, self.myPOSlow, self.myPOSmid)

    ###############################################################################
    # motorRMOD_G
    # position forward & backlash compensation, absolute
    def test_TC_92021(self):
        positionAndBacklash(self, self.motor, 92021, motorRMOD_G, use_abs, self.myPOSlow, self.myPOShig)

    # position forward & backlash compensation, relative
    def test_TC_92022(self):
        positionAndBacklash(self, self.motor, 92022, motorRMOD_G, use_rel, self.myPOSlow, self.myPOShig)

    # position backward & backlash compensation, absolute
    def test_TC_92023(self):
        positionAndBacklash(self, self.motor, 92023, motorRMOD_G, use_abs, self.myPOShig, self.myPOSlow)

    # position backward & backlash compensation, relative
    def test_TC_92024(self):
        positionAndBacklash(self, self.motor, 92024, motorRMOD_G, use_rel, self.myPOShig, self.myPOSlow)

    # position forward inside backlash range, absolute
    def test_TC_92025(self):
        positionAndBacklash(self, self.motor, 92025, motorRMOD_G, use_abs, self.myPOSmid, self.myPOSlow)

    # position forward inside backlash range, relative
    def test_TC_92026(self):
        positionAndBacklash(self, self.motor, 92026, motorRMOD_G, use_rel, self.myPOSmid, self.myPOSlow)

    # position forward inside backlash range, absolute
    def test_TC_92027(self):
        positionAndBacklash(self, self.motor, 92027, motorRMOD_G, use_abs, self.myPOSlow, self.myPOSmid)

    # position forward inside backlash range, relative
    def test_TC_92028(self):
        positionAndBacklash(self, self.motor, 92028, motorRMOD_G, use_rel, self.myPOSlow, self.myPOSmid)


    ###############################################################################
    # motorRMOD_I
    # position forward & backlash compensation, absolute
    def test_TC_92031(self):
        positionAndBacklash(self, self.motor, 92031, motorRMOD_I, use_abs, self.myPOSlow, self.myPOShig)

    # position forward & backlash compensation, relative
    def test_TC_92032(self):
        positionAndBacklash(self, self.motor, 92032, motorRMOD_I, use_rel, self.myPOSlow, self.myPOShig)

    # position backward & backlash compensation, absolute
    def test_TC_92033(self):
        positionAndBacklash(self, self.motor, 92033, motorRMOD_I, use_abs, self.myPOShig, self.myPOSlow)

    # position backward & backlash compensation, relative
    def test_TC_92034(self):
        positionAndBacklash(self, self.motor, 92034, motorRMOD_I, use_rel, self.myPOShig, self.myPOSlow)

    # position forward inside backlash range, absolute
    def test_TC_92035(self):
        positionAndBacklash(self, self.motor, 92035, motorRMOD_I, use_abs, self.myPOSmid, self.myPOSlow)

    # position forward inside backlash range, relative
    def test_TC_92036(self):
        positionAndBacklash(self, self.motor, 92036, motorRMOD_I, use_rel, self.myPOSmid, self.myPOSlow)

    # position forward inside backlash range, absolute
    def test_TC_92037(self):
        positionAndBacklash(self, self.motor, 92037, motorRMOD_I, use_abs, self.myPOSlow, self.myPOSmid)

    # position forward inside backlash range, relative
    def test_TC_92038(self):
        positionAndBacklash(self, self.motor, 92038, motorRMOD_I, use_rel, self.myPOSlow, self.myPOSmid)

