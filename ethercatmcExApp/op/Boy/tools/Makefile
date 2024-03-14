
GNERATEDOPIS = \
  ../motor-0-ptp.opi \
  ../motor-1.opi \
  ../motor-1-ptp.opi \
  ../motor-2.opi \
  ../motor-3.opi \
  ../motor-3-ptp.opi \
  ../motor-4.opi \
  ../motor-4-ptp.opi \
  ../motor-4-ecmc.opi \
  ../motor-5.opi \
  ../motor-6.opi  \
  ../motor-4x3.opi \
  ../motor-4x4.opi \
  ../motor-4x4-temp.opi \
  ../motor-estiaSel1-temp.opi \
  ../motor-estiaSel2-temp.opi \
  ../motor-ymir-mcs1-temp.opi \
  ../ethercatmcaxisConfig.opi \
  ../ethercatmcaxisExpert-ecmc.opi \
  ../ethercatmcaxisExpert-tc.opi \
  ../ethercatmcaxisExpert-tc-ptp.opi \
  ../ethercatmcPTPErrBits.opi \
  ../ethercatmcShutter.opi \
  ../ethercatmcStatusWord1802.opi \
  ../shutter-1.opi \
  ../IOC.opi \


#  ../ethercatmcaxisConfig-ecmc.opi \

#Main target:
all: checkwhitespace $(GNERATEDOPIS)

clean:
	rm -f $(GNERATEDOPIS)


DEPENDENCIES_FOR_CONFIG_OPI = \
        ethercatmcaxisConfig.start \
        ethercatmcaxisConfig.mid \
        ethercatmcaxisConfig.end


# Depencies, when do we need to revuild a .opi file
DEPENDENCIES_FOR_MOTOR_OPI = Makefile
DEPENDENCIES_FOR_MOTOR_OPI += genMotorShutter.sh
DEPENDENCIES_FOR_MOTOR_OPI += shiftopi.py
DEPENDENCIES_FOR_MOTOR_OPI += motorx.start
DEPENDENCIES_FOR_MOTOR_OPI += motorx.mid
DEPENDENCIES_FOR_MOTOR_OPI += motorx-egu-rbv.mid
DEPENDENCIES_FOR_MOTOR_OPI += plcHealthStatus.mid
DEPENDENCIES_FOR_MOTOR_OPI += plcIPADDR_PORT.mid
DEPENDENCIES_FOR_MOTOR_OPI += plcName.mid
DEPENDENCIES_FOR_MOTOR_OPI += ptp.mid
DEPENDENCIES_FOR_MOTOR_OPI += PTPdiffNTtime_MCU.mid
DEPENDENCIES_FOR_MOTOR_OPI += PTPdiffTimeIOC_MCU.mid
DEPENDENCIES_FOR_MOTOR_OPI += ptp-ts-ns.mid
DEPENDENCIES_FOR_MOTOR_OPI += openPTPErrBits.mid
DEPENDENCIES_FOR_MOTOR_OPI += motorx.end
DEPENDENCIES_FOR_SHUTTER_OPI = Makefile
DEPENDENCIES_FOR_SHUTTER_OPI += genMotorShutter.sh
DEPENDENCIES_FOR_SHUTTER_OPI += shutterx.start
DEPENDENCIES_FOR_SHUTTER_OPI += shutterx.mid
DEPENDENCIES_FOR_SHUTTER_OPI += plcIPADDR_PORT.mid
DEPENDENCIES_FOR_SHUTTER_OPI += plcName.mid
DEPENDENCIES_FOR_SHUTTER_OPI += ptp.mid
DEPENDENCIES_FOR_SHUTTER_OPI += openPTPErrBits.mid
DEPENDENCIES_FOR_SHUTTER_OPI += shutterx.end


DEPENDENCIES_FOR_STATUSWORD_OPI = Makefile
DEPENDENCIES_FOR_STATUSWORD_OPI += genExpertWithAuxBits.sh
DEPENDENCIES_FOR_STATUSWORD_OPI += genExpertWithAuxBits.py
DEPENDENCIES_FOR_STATUSWORD_OPI += ethercatmcStatusWord1802.start
DEPENDENCIES_FOR_STATUSWORD_OPI += ethercatmcStatusWord1802.mid
DEPENDENCIES_FOR_STATUSWORD_OPI += ethercatmcaxisAuxBit.mid
DEPENDENCIES_FOR_STATUSWORD_OPI += ethercatmcStatusWord1802.end
DEPENDENCIES_FOR_STATUSWORD_OPI += openPTPErrBits.mid


DEPENDENCIES_FOR_PTPERRORSTATUS_OPI = Makefile
DEPENDENCIES_FOR_PTPERRORSTATUS_OPI += genExpertWithAuxBits.sh
DEPENDENCIES_FOR_PTPERRORSTATUS_OPI += genExpertWithAuxBits.py
DEPENDENCIES_FOR_PTPERRORSTATUS_OPI += ethercatmcPTPErrBits.start
DEPENDENCIES_FOR_PTPERRORSTATUS_OPI += ethercatmcPTPErrBits.mid
DEPENDENCIES_FOR_PTPERRORSTATUS_OPI += ethercatmcaxisAuxBit.mid
DEPENDENCIES_FOR_PTPERRORSTATUS_OPI += ethercatmcPTPErrBits.end
DEPENDENCIES_FOR_PTPERRORSTATUS_OPI += ptp-ts-ns-pos-neg.mid
DEPENDENCIES_FOR_PTPERRORSTATUS_OPI += ptp-ts-ns.mid
DEPENDENCIES_FOR_PTPERRORSTATUS_OPI += PTPdiffTimeIOC_MCU-HIGH-LOW.mid



DEPENDENCIES_FOR_MOTOR_EXPERT_OPI = Makefile
DEPENDENCIES_FOR_MOTOR_EXPERT_OPI += ecmc.mid
DEPENDENCIES_FOR_MOTOR_EXPERT_OPI += genExpertWithAuxBits.sh
DEPENDENCIES_FOR_MOTOR_EXPERT_OPI += genExpertWithAuxBits.py
DEPENDENCIES_FOR_MOTOR_EXPERT_OPI += ethercatmcaxisExpert.start
DEPENDENCIES_FOR_MOTOR_EXPERT_OPI += ethercatmcaxisExpert.mid
DEPENDENCIES_FOR_MOTOR_EXPERT_OPI += ethercatmcaxisAuxBit.mid
DEPENDENCIES_FOR_MOTOR_EXPERT_OPI += ethercatmcaxisExpert.end
DEPENDENCIES_FOR_MOTOR_EXPERT_OPI += ptp.mid
DEPENDENCIES_FOR_MOTOR_EXPERT_OPI += tc.mid

DEPENDENCIES_FOR_SHUTTER_EXPERT_OPI = Makefile
DEPENDENCIES_FOR_SHUTTER_EXPERT_OPI += genExpertWithAuxBits.sh
DEPENDENCIES_FOR_SHUTTER_EXPERT_OPI += genExpertWithAuxBits.py
DEPENDENCIES_FOR_SHUTTER_EXPERT_OPI += ethercatmcShutter.start
DEPENDENCIES_FOR_SHUTTER_EXPERT_OPI += plcHealthStatus.mid
DEPENDENCIES_FOR_SHUTTER_EXPERT_OPI += plcName.mid
DEPENDENCIES_FOR_SHUTTER_EXPERT_OPI += ethercatmcShutter.mid
DEPENDENCIES_FOR_SHUTTER_EXPERT_OPI += ethercatmcaxisAuxBit.mid
DEPENDENCIES_FOR_SHUTTER_EXPERT_OPI += ptp.mid
DEPENDENCIES_FOR_SHUTTER_EXPERT_OPI += ethercatmcShutter.end


AUXBITS_EXPERT="20 19 18 17 16 15 14 13 12 11 10 9 8 7 6 5 4 3 2 1 0"
AUXBITS_SHUTTER="23 22 21 20 19 18 17 16 15 14 13 5 4 3 2 1 0"
AUXBITS_STATUSWORD="0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23"
AUXBITS_PTPERRORSTATUS="15 14 13 12 11 10 9 8 7 6 5 4 3 2 1 0"

../motor-1.opi: $(DEPENDENCIES_FOR_MOTOR_OPI)
	./genMotorShutter.sh $@ 1 x 1

../motor-0-ptp.opi: $(DEPENDENCIES_FOR_MOTOR_OPI)
	./genMotorShutter.sh $@ ptp

../motor-1-ptp.opi: $(DEPENDENCIES_FOR_MOTOR_OPI)
	./genMotorShutter.sh $@ ptp 1 x 1

../motor-2.opi: $(DEPENDENCIES_FOR_MOTOR_OPI)
	./genMotorShutter.sh $@ 2 x 1

../motor-3.opi: $(DEPENDENCIES_FOR_MOTOR_OPI)
	./genMotorShutter.sh $@ 3 x 1

../motor-3-ptp.opi: $(DEPENDENCIES_FOR_MOTOR_OPI)
	./genMotorShutter.sh $@ ptp 3 x 1

../motor-4.opi: $(DEPENDENCIES_FOR_MOTOR_OPI)
	./genMotorShutter.sh $@ 4 x 1

../motor-4-ecmc.opi: $(DEPENDENCIES_FOR_MOTOR_OPI)
	./genMotorShutter.sh $@ ecmc 4 x 1

../motor-4-ptp.opi: $(DEPENDENCIES_FOR_MOTOR_OPI)
	./genMotorShutter.sh $@ ptp 4 x 1

../motor-5.opi: $(DEPENDENCIES_FOR_MOTOR_OPI)
	./genMotorShutter.sh $@ 5 x 1

../motor-6.opi: $(DEPENDENCIES_FOR_MOTOR_OPI)
	./genMotorShutter.sh $@ 6 x 1

../motor-4x3.opi: $(DEPENDENCIES_FOR_MOTOR_OPI)
	./genMotorShutter.sh $@ 4 x 3

../motor-4x4.opi: $(DEPENDENCIES_FOR_MOTOR_OPI)
	./genMotorShutter.sh $@ 4 x 4

../motor-4x4-temp.opi: $(DEPENDENCIES_FOR_MOTOR_OPI) motorx-temp-rbv.mid
	./genMotorShutter.sh $@ temp m m m m n m m m m n m m m m n m m m m

../motor-estiaSel1-temp.opi: $(DEPENDENCIES_FOR_MOTOR_OPI) motorx-temp-rbv.mid tempsensor.mid
	./genMotorShutter.sh $@ temp m m m m m n m m m m m n m m m n t t t t t n t t t t t n t t

../motor-estiaSel2-temp.opi: $(DEPENDENCIES_FOR_MOTOR_OPI) motorx-temp-rbv.mid tempsensor.mid
	./genMotorShutter.sh $@ temp m m m m m n m m m m m n m m m m n t t t t t n t t t t t n t t

../motor-ymir-mcs1-temp.opi: $(DEPENDENCIES_FOR_MOTOR_OPI) motorx-temp-rbv.mid tempsensor.mid
	./genMotorShutter.sh $@ temp m m m m m m n shutter s s s

../ethercatmcaxisConfig.opi: $(DEPENDENCIES_FOR_CONFIG_OPI) Makefile
	cat $(DEPENDENCIES_FOR_CONFIG_OPI)  >$@

#../ethercatmcaxisConfig-ecmc.opi \


# Generate ethercatmcaxisExpert*.opi out of
#   ethercatmcaxisExpert.mid and ethercatmcaxisExpert.end
../ethercatmcaxisExpert-tc.opi: $(DEPENDENCIES_FOR_MOTOR_EXPERT_OPI)
	./genExpertWithAuxBits.sh $@ ethercatmcaxisExpert tc $(AUXBITS_EXPERT)

../ethercatmcaxisExpert-ecmc.opi: $(DEPENDENCIES_FOR_MOTOR_EXPERT_OPI)
	./genExpertWithAuxBits.sh $@ ethercatmcaxisExpert ecmc

../ethercatmcaxisExpert-tc-ptp.opi: $(DEPENDENCIES_FOR_MOTOR_EXPERT_OPI)
	./genExpertWithAuxBits.sh $@ ethercatmcaxisExpert tc ptp $(AUXBITS_EXPERT)

# Generate ethercatmcShutter*.opi out of
#   ethercatmcShutter.mid and ethercatmcShutter.end
../ethercatmcShutter.opi: $(DEPENDENCIES_FOR_SHUTTER_EXPERT_OPI)
	./genExpertWithAuxBits.sh $@ ethercatmcShutter $(AUXBITS_SHUTTER)

# Generate ethercatmcStatusWord1802*.opi out of
#   ethercatmcStatusWord1802.mid and ethercatmcStatusWord1802.end
../ethercatmcPTPErrBits.opi: $(DEPENDENCIES_FOR_PTPERRORSTATUS_OPI)
	/bin/sh -e -x ./genExpertWithAuxBits.sh $@ ethercatmcPTPErrBits ptp ptpdifftimeioc_mcu PTPdiffNTtime_mcu PTPdiffTcNTPExttime_mcu ptpTCdifftime_mcu ptptsnsposneg $(AUXBITS_PTPERRORSTATUS)

../ethercatmcStatusWord1802.opi: $(DEPENDENCIES_FOR_STATUSWORD_OPI)
	./genExpertWithAuxBits.sh $@ ethercatmcStatusWord1802 $(AUXBITS_STATUSWORD)

../shutter-1.opi: $(DEPENDENCIES_FOR_SHUTTER_OPI)
	./genMotorShutter.sh $@ shutter 1 x 1

../IOC.opi: Makefile genIOC.sh IOC*.*
	./genIOC.sh ../IOC.opi IOC


checkwhitespace:
	../../../../checkws.sh


