
GNERATEDOPIS = \
  ../motor-1.opi \
  ../motor-1-ptp.opi \
  ../motor-2.opi \
  ../motor-3.opi \
  ../motor-3-ptp.opi \
  ../motor-4.opi \
  ../motor-4-ptp.opi \
  ../motor-5.opi \
  ../motor-6.opi  \
  ../motor-4x3.opi \
  ../motor-4x4.opi \
  ../ethercatmcaxisExpert.opi \
  ../ethercatmcaxisExpert-ptp.opi \
  ../ethercatmcPTPErrBits.opi \
  ../ethercatmcShutter.opi \
  ../ethercatmcStatusWord1802.opi \
  ../shutter-1.opi \
  ../IOC.opi \

#Main target:
all: checkwhitespace $(GNERATEDOPIS)

clean:
	rm -f $(GNERATEDOPIS)

# Depencies, when do we need to revuild a .opi file
DEPENDENCIES_FOR_MOTOR_OPI = Makefile
DEPENDENCIES_FOR_MOTOR_OPI += genMotorShutter.sh
DEPENDENCIES_FOR_MOTOR_OPI += motorx.start
DEPENDENCIES_FOR_MOTOR_OPI += motorx.mid
DEPENDENCIES_FOR_MOTOR_OPI += plcHealthStatus.mid
DEPENDENCIES_FOR_MOTOR_OPI += plcIPADDR_PORT.mid
DEPENDENCIES_FOR_MOTOR_OPI += plcName.mid
DEPENDENCIES_FOR_MOTOR_OPI += ptp.mid
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


DEPENDENCIES_FOR_MOTOR_EXPERT_OPI = Makefile
DEPENDENCIES_FOR_MOTOR_EXPERT_OPI += genExpertWithAuxBits.sh
DEPENDENCIES_FOR_MOTOR_EXPERT_OPI += genExpertWithAuxBits.py
DEPENDENCIES_FOR_MOTOR_EXPERT_OPI += ethercatmcaxisExpert.start
DEPENDENCIES_FOR_MOTOR_EXPERT_OPI += ethercatmcaxisExpert.mid
DEPENDENCIES_FOR_MOTOR_EXPERT_OPI += ethercatmcaxisAuxBit.mid
DEPENDENCIES_FOR_MOTOR_EXPERT_OPI += ptp.mid
DEPENDENCIES_FOR_MOTOR_EXPERT_OPI += ethercatmcaxisExpert.end

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
	./genMotorShutter.sh $@ 1

../motor-1-ptp.opi: $(DEPENDENCIES_FOR_MOTOR_OPI)
	./genMotorShutter.sh $@ ptp 1

../motor-2.opi: $(DEPENDENCIES_FOR_MOTOR_OPI)
	./genMotorShutter.sh $@ 2

../motor-3.opi: $(DEPENDENCIES_FOR_MOTOR_OPI)
	./genMotorShutter.sh $@ 3

../motor-3-ptp.opi: $(DEPENDENCIES_FOR_MOTOR_OPI)
	./genMotorShutter.sh $@ ptp 3

../motor-4.opi: $(DEPENDENCIES_FOR_MOTOR_OPI)
	./genMotorShutter.sh $@ 4

../motor-4-ptp.opi: $(DEPENDENCIES_FOR_MOTOR_OPI)
	./genMotorShutter.sh $@ ptp 4

../motor-5.opi: $(DEPENDENCIES_FOR_MOTOR_OPI)
	./genMotorShutter.sh $@ 5


../motor-6.opi: $(DEPENDENCIES_FOR_MOTOR_OPI)
	./genMotorShutter.sh $@ 6


../motor-4x3.opi: $(DEPENDENCIES_FOR_MOTOR_OPI)
	./genMotorShutter.sh $@ 4 x 3

../motor-4x4.opi: $(DEPENDENCIES_FOR_MOTOR_OPI)
	./genMotorShutter.sh $@ 4 x 4

# Generate ethercatmcaxisExpert*.opi out of
#   ethercatmcaxisExpert.mid and ethercatmcaxisExpert.end
../ethercatmcaxisExpert.opi: $(DEPENDENCIES_FOR_MOTOR_EXPERT_OPI)
	./genExpertWithAuxBits.sh $@ ethercatmcaxisExpert $(AUXBITS_EXPERT)

../ethercatmcaxisExpert-ptp.opi: $(DEPENDENCIES_FOR_MOTOR_EXPERT_OPI)
	./genExpertWithAuxBits.sh $@ ethercatmcaxisExpert ptp $(AUXBITS_EXPERT)

# Generate ethercatmcShutter*.opi out of
#   ethercatmcShutter.mid and ethercatmcShutter.end
../ethercatmcShutter.opi: $(DEPENDENCIES_FOR_SHUTTER_EXPERT_OPI)
	./genExpertWithAuxBits.sh $@ ethercatmcShutter $(AUXBITS_SHUTTER)

# Generate ethercatmcStatusWord1802*.opi out of
#   ethercatmcStatusWord1802.mid and ethercatmcStatusWord1802.end
../ethercatmcPTPErrBits.opi: $(DEPENDENCIES_FOR_PTPERRORSTATUS_OPI)
	./genExpertWithAuxBits.sh $@ ethercatmcPTPErrBits ptp ptprwoffset $(AUXBITS_PTPERRORSTATUS)

../ethercatmcStatusWord1802.opi: $(DEPENDENCIES_FOR_STATUSWORD_OPI)
	./genExpertWithAuxBits.sh $@ ethercatmcStatusWord1802 $(AUXBITS_STATUSWORD)

../shutter-1.opi: $(DEPENDENCIES_FOR_SHUTTER_OPI)
	./genMotorShutter.sh $@ s 1

../IOC.opi: Makefile genIOC.sh IOC*.*
	./genIOC.sh ../IOC.opi IOC


checkwhitespace:
	../../../../checkws.sh


