
GNERATEDOPIS: \
  ../motor-1.opi \
  ../motor-2.opi \
  ../motor-2-2.opi \
  ../motor-3.opi \
  ../motor-4.opi \
  ../motor-5.opi \
  ../motor-6.opi  \
  ../motor-4x3.opi \
  ../motor-4-2.opi \

# Depencies, when do we need to revuild a .opi file
DEPENDENCIES_FOR_OPI = Makefile
DEPENDENCIES_FOR_OPI += genXY.sh
DEPENDENCIES_FOR_OPI += motorx.start
DEPENDENCIES_FOR_OPI += motorx.mid
DEPENDENCIES_FOR_OPI += ptp.mid
DEPENDENCIES_FOR_OPI += motorx.end 

# Height of the title line
TITLEH=16

 ../motor-2-2.opi: $(DEPENDENCIES_FOR_OPI)
	TITLEH=$(TITLEH) ./genXY.sh 2 2

../motor-1.opi: $(DEPENDENCIES_FOR_OPI)
	TITLEH=$(TITLEH) ./genXY.sh 1

../motor-2.opi: $(DEPENDENCIES_FOR_OPI) 
	TITLEH=$(TITLEH) ./genXY.sh 2

../motor-3.opi: $(DEPENDENCIES_FOR_OPI)
	TITLEH=$(TITLEH) ./genXY.sh 3


../motor-4.opi: $(DEPENDENCIES_FOR_OPI)
	TITLEH=$(TITLEH) ./genXY.sh 4


../motor-5.opi: $(DEPENDENCIES_FOR_OPI)
	TITLEH=$(TITLEH) ./genXY.sh 5


../motor-6.opi: $(DEPENDENCIES_FOR_OPI)
	TITLEH=$(TITLEH) ./genXY.sh 6


../motor-4x3.opi: $(DEPENDENCIES_FOR_OPI)
	TITLEH=$(TITLEH) ./genXY.sh 4 x 3


../motor-4-2.opi: $(DEPENDENCIES_FOR_OPI)
	TITLEH=$(TITLEH) ./genXY.sh 4 2


all: $(GNERATEDOPIS)

