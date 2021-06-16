
# Height of the title line
TITLEH=16

GNERATEDOPIS = \
  ../motor-1.opi \
  ../motor-1-ptp.opi \
  ../motor-2.opi \
  ../motor-3.opi \
  ../motor-3-ptp.opi \
  ../motor-4.opi \
  ../motor-5.opi \
  ../motor-6.opi  \
  ../motor-4x3.opi \
  ../motor-4x4.opi \
  ../ethercatmcaxisExpert.opi \
  ../ethercatmcaxisExpert-ptp.opi \

#Main target:
all: checkwhitespace $(GNERATEDOPIS)

clean:
	rm -f $(GNERATEDOPIS)

# Depencies, when do we need to revuild a .opi file
DEPENDENCIES_FOR_MOTOR_OPI = Makefile
DEPENDENCIES_FOR_MOTOR_OPI += genXY.sh
DEPENDENCIES_FOR_MOTOR_OPI += motorx.start
DEPENDENCIES_FOR_MOTOR_OPI += motorx.mid
DEPENDENCIES_FOR_MOTOR_OPI += plcName.mid
DEPENDENCIES_FOR_MOTOR_OPI += ptp.mid
DEPENDENCIES_FOR_MOTOR_OPI += motorx.end 


DEPENDENCIES_FOR_EXPERT_OPI = Makefile
DEPENDENCIES_FOR_EXPERT_OPI += genY.sh
DEPENDENCIES_FOR_EXPERT_OPI += genExpertWithAuxBits.sh
DEPENDENCIES_FOR_EXPERT_OPI += genExpertWithAuxBits.py
DEPENDENCIES_FOR_EXPERT_OPI += ethercatmcaxisExpert.start
DEPENDENCIES_FOR_EXPERT_OPI += ethercatmcaxisExpert.mid
DEPENDENCIES_FOR_EXPERT_OPI += ethercatmcaxisAuxBit.mid
DEPENDENCIES_FOR_EXPERT_OPI += ptp.mid
DEPENDENCIES_FOR_EXPERT_OPI += ethercatmcaxisExpert.end


AUXBITS="17 18 19 20 0 1 2 3 4"

../motor-1.opi: $(DEPENDENCIES_FOR_MOTOR_OPI)
	TITLEH=$(TITLEH) ./genXY.sh $@ 1

../motor-1-ptp.opi: $(DEPENDENCIES_FOR_MOTOR_OPI)
	TITLEH=$(TITLEH) ./genXY.sh $@ ptp 1

../motor-2.opi: $(DEPENDENCIES_FOR_MOTOR_OPI) 
	TITLEH=$(TITLEH) ./genXY.sh $@ 2

../motor-3.opi: $(DEPENDENCIES_FOR_MOTOR_OPI)
	TITLEH=$(TITLEH) ./genXY.sh $@ 3

../motor-3-ptp.opi: $(DEPENDENCIES_FOR_MOTOR_OPI)
	TITLEH=$(TITLEH) ./genXY.sh $@ ptp 3

../motor-4.opi: $(DEPENDENCIES_FOR_MOTOR_OPI)
	TITLEH=$(TITLEH) ./genXY.sh $@ 4


../motor-5.opi: $(DEPENDENCIES_FOR_MOTOR_OPI)
	TITLEH=$(TITLEH) ./genXY.sh $@ 5


../motor-6.opi: $(DEPENDENCIES_FOR_MOTOR_OPI)
	TITLEH=$(TITLEH) ./genXY.sh $@ 6


../motor-4x3.opi: $(DEPENDENCIES_FOR_MOTOR_OPI)
	TITLEH=$(TITLEH) ./genXY.sh $@ 4 x 3

../motor-4x4.opi: $(DEPENDENCIES_FOR_MOTOR_OPI)
	TITLEH=$(TITLEH) ./genXY.sh $@ 4 x 4

# Generate ethercatmcaxisExpert*.opi out of
#   ethercatmcaxisExpert.mid and ethercatmcaxisExpert.end
../ethercatmcaxisExpert.opi: $(DEPENDENCIES_FOR_EXPERT_OPI)
	TITLEH=$(TITLEH) ./genExpertWithAuxBits.sh $@ ethercatmcaxisExpert $(AUXBITS)

../ethercatmcaxisExpert-ptp.opi: $(DEPENDENCIES_FOR_EXPERT_OPI)
	TITLEH=$(TITLEH) ./genExpertWithAuxBits.sh $@ ethercatmcaxisExpert ptp $(AUXBITS)

checkwhitespace:
	../../../../checkws.sh


