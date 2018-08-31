# EthercatMC

EPICS module: EthercatMC

This is a model 3 driver for EtherCAT based motion controllers.
It needs a patched version of the motorRecord (or the very old version 6.9)

## Having the Simulation Up and Running

Open a terminal window and execute the following commands:

```
cd <local-repository>/EthercatMC/test
./run-EthercatMC-simulator.sh
```

Open another terminal window and execute the following commands:

```
cd <local-repository>/axis
make install
cd test
./run-EthercatMC-ioc.sh SolAxis-SimCfgDbg
```

Pressing RETURN then executing `dbl` will show the list of available PVs.

