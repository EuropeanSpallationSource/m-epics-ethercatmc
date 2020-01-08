# ethercatmc

EPICS module: ethercatmc

This is a model 3 driver for EtherCAT based motion controllers.
It needs a patched version of the motorRecord:
https://github.com/EuropeanSpallationSource/motor

Why do we need a patched version?
1) When the low limit switch is activated during HOMF, the record
   will send a STOP in the middle of the homing procedure, and
   the homing procedure fails.
2) When a limit switch is hit, the motor record assumes that the
   motion has stopped completely.
   This is not always the case, the contoller may "ramp down" the
   axis, and then it is stopped.
3) Read only softlimits.
   If configured, the controller has soft limits defined.
   The ESS version of the motorRecord push those into the DHLM/DLLM
   field and will not allow a user to loosen them.
## Having the Simulation Up and Running

Open a terminal window and execute the following commands:

```
cd <local-repository>/ethercatmc/test
./run-ethercatmc-simulator.sh
```

Open another terminal window and execute the following commands:

```
cd <local-repository>/ethercatmc/test
./run-ethercatmc-ioc.sh SolAxis-SimCfgDbg
```

Pressing RETURN then executing `dbl` will show the list of available PVs.

