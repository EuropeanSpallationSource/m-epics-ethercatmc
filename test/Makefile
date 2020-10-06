###########################################################
# The default target: all

all-ca:
	make -k -C pytests36 record-ca
	make -k -C pytests36 ethercatmc-ca
	make -k -C pytests36 simulator-one-ca
	make -k -C pytests36 simulator-two-ca

clean:
	make -C .. clean
	make -C pytests36  clean

###########################################################
