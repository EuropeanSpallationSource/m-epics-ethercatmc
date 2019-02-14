
ALLSRCFILES=Makefile shiftopi.py genXY.sh motorx.end motorx.mid motorx.start

GENERATEDOPIS: \
  ../motor-2.adl \
  ../motor-4.adl \
  ../motor-5.adl \
  ../motor-6.adl  \
  ../motor-4x3.adl \
  ../motor-4x2.adl \
  ../motor-4-2.adl \

#GENERATEDOPIS: \
#  ../motor-6-6-4.adl \



#

../motor-2.adl: $(ALLSRCFILES)
	./genXY.sh 2


../motor-4.adl: $(ALLSRCFILES)
	./genXY.sh 4


../motor-5.adl: $(ALLSRCFILES)
	./genXY.sh 5


../motor-6.adl: $(ALLSRCFILES)
	./genXY.sh 6


../motor-4x3.adl: $(ALLSRCFILES)
	./genXY.sh 4 x 3

../motor-4x2.adl: $(ALLSRCFILES)
	./genXY.sh 4 x 2

../motor-4-2.adl: $(ALLSRCFILES)
	./genXY.sh 4  2

#../motor-6-6-4.adl: $(ALLSRCFILES)
#	./genXY.sh 6 6 4



all: $(GENERATEDOPIS)

