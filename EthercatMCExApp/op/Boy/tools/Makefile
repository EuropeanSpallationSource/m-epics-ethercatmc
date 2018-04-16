
GNERATEDOPIS: \
  ../motor2x.opi \
  ../motor4x.opi \
  ../motor5x.opi \
  ../motor6x.opi  \
  ../motor4x3.opi \


#GNERATEDOPIS: \
#  ../motor-6-6-4.opi \



#
 ../motor2x.opi: Makefile genXY.sh motorx.end motorx.mid motorx.start
	./genXY.sh 2


../motor4x.opi: Makefile genXY.sh motorx.end motorx.mid motorx.start
	./genXY.sh 4


../motor6x.opi: Makefile genXY.sh motorx.end motorx.mid motorx.start
	./genXY.sh 6


../motor4x3.opi: Makefile genXY.sh motorx.end motorx.mid motorx.start
	./genXY.sh 4 x 3


../motor-6-6-4.opi: Makefile genXY.sh motorx.end motorx.mid motorx.start
	./genXY.sh 6 6 4



all: $(GNERATEDOPIS)

