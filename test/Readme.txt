What is this all about and good for ?
  - We use a EPICS base, asyn, motor and this module ethercatmc
    ethercatmc is a "model 3" motor driver
  - Tested features
    - Features of the motorRecord itself
       E.g. Retry- and backlash
    - Regression tests of ethercatmc
    - Testing against a simulator
    - Testing commissioned real hardware (some test cases)
      Drive with maximum velocity
      Don't hit a limit switch
      Does the motor reach the target position ?
      Can the motor be homed
  - What is the tested device ?
    An EPICS IOC connected to either a motion controller with motors
    (or the simulator)
  - Integration testing with pytest
  - How do we interface the tested device ?
    - EPICS records only
      No access to internal data at all -
         if needed, expose them via EPICS records (-debug.template)
    - channel access and pvaccess
      ESS is aiming for pvaccess. We use pyepics and p4p
      However, the communication is abstracted away (AxisCom.py)
  - How to debug the stuff
    Read the logs from pytest (normally hidden)
    Read the IOC-log
      What python code causes which IOC action, what is what ?
      see the "debug prints" from
        self.axisCom.put("-DbgStrToLOG", "Start " + str(tc_no))

  - How does it work ?
    Using pytest
    - send a value to a PV
    - wait
    - read other PVs
    - check result (no errors, no alarms states, position reached)
    
- General problems
  Different versions of python (3.x sometime 2.7)
  Platforms: Mix of oldish and newish SW (MacOs, Centos 7, Debian)
  Look for "virtual env", conda, miniconda
  Needed to write shell scripts to wrap calling pytest


- Start small
  Use a small test
  Increase test scope, debug code
  Improve/refactor things
  (That is how this code base looks like,
   some test cases had been refactored, some need to be cleaned up)

- Continues integration
  Travis, Github Actions: need another wrapper to start
  the simulator, the IOC and the python test cases

- Other test frameworks
  EPICS base
  EPICS asyn
  Lewis



