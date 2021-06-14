
# Height of the title line
TITLEH=16

GNERATEDOPIS = \
  ../motor-1.opi \
  ../motor-2.opi \
  ../motor-3.opi \
  ../motor-4.opi \
  ../motor-5.opi \
  ../motor-6.opi  \
  ../motor-4x3.opi \
  ../motor-4x4.opi \
  ../ethercatmcaxisExpert.opi \

#Main target:
all: checkwhitespace $(GNERATEDOPIS)

clean:
	rm -f $(GNERATEDOPIS)

# Depencies, when do we need to revuild a .opi file
DEPENDENCIES_FOR_MOTOR_OPI = Makefile
DEPENDENCIES_FOR_MOTOR_OPI += genXY.sh
DEPENDENCIES_FOR_MOTOR_OPI += motorx.start
DEPENDENCIES_FOR_MOTOR_OPI += motorx.mid
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



../motor-1.opi: $(DEPENDENCIES_FOR_MOTOR_OPI)
	TITLEH=$(TITLEH) ./genXY.sh 1


../motor-2.opi: $(DEPENDENCIES_FOR_MOTOR_OPI) 
	TITLEH=$(TITLEH) ./genXY.sh 2


../motor-3.opi: $(DEPENDENCIES_FOR_MOTOR_OPI)
	TITLEH=$(TITLEH) ./genXY.sh 3

../motor-4.opi: $(DEPENDENCIES_FOR_MOTOR_OPI)
	TITLEH=$(TITLEH) ./genXY.sh 4


../motor-5.opi: $(DEPENDENCIES_FOR_MOTOR_OPI)
	TITLEH=$(TITLEH) ./genXY.sh 5


../motor-6.opi: $(DEPENDENCIES_FOR_MOTOR_OPI)
	TITLEH=$(TITLEH) ./genXY.sh 6


../motor-4x3.opi: $(DEPENDENCIES_FOR_MOTOR_OPI)
	TITLEH=$(TITLEH) ./genXY.sh 4 x 3

../motor-4x4.opi: $(DEPENDENCIES_FOR_MOTOR_OPI)
	TITLEH=$(TITLEH) ./genXY.sh 4 x 4

../ethercatmcaxisExpert.opi: $(DEPENDENCIES_FOR_EXPERT_OPI)
	TITLEH=$(TITLEH) ./genExpertWithAuxBits.sh ethercatmcaxisExpert 0 1 2 3 4 5 6 7


checkwhitespace:
	../../../../checkws.sh


