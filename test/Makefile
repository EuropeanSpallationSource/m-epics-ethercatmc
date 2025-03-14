###########################################################
# The default target: test

all: all-pva monitor-ca-pva

all-pva:
	make -k -C pytests36 record-pva
	make -k -C pytests36 ethercatmc-pva
	make -k -C pytests36 simulator-one-pva
	make -k -C pytests36 simulator-two-pva

monitor-ca-pva:
	make -k -C pytests36 monitor-ca-pva

clean:
	make -C .. clean
	make -C pytests36  clean

###########################################################
