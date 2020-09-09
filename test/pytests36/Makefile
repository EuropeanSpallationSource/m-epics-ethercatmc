#
# Makefile to run the different test case
# test cases are grouped into 3 categories:
# - (motor)Record : Should work with all motors
# - ethercatmc    : Works only with the ethercatmc driver
#                     additional ai/ao records are used which have a certain functionality
# - simulator     : Work only with ethercatmc & simulator
#                     Only here we can simulate e.g. a motor that has "stuck" in a certain position
#                     and triggers the retry-login in the motorRecord


SIMULATOR-ONE-CA = \
  900_Simulator-Softlimits.log-ca\
  910_Simulator_JOGF_BDST.log-ca\
  920_Simulator_RTRY_FRAC_RMOD.log-ca\
  930_Simulator_Vel-RB_Acc-RB.log-ca\
  940_Simulator-powerOnFailed.log-ca\

SIMULATOR-TWO-CA = \
  950_Simulator-JOGF-JOGR-JVEL-MRES.log-ca\


RECORD-CA = \
  100_Record-HOMF.log-ca\
  121_Record-JOGF_DIR_DLY_HLM.log-ca\
  131_Record-JOGR_DIR_DLY_LLM.log-ca\
  140_Record-VAL_VMAX.log-ca\
  220_Record_auto_power.log-ca\
  500_Record-DLY-STOP.log-ca\


ETHERCATMC-CA = \
  010_Basic_Communication.log-ca\
  090_Ethercat-SmokeTest-TWF_CNEN.log-ca\
  122_Ethercat-JOGF_HLS.log-ca\
  123_Ethercat-MoveVel_MCU_Hsoftlimit.log-ca\
  132_Ethercat-JOGR_LLS.log-ca\
  133_Ethercat_MoveVel_MCU_Lsoftlimit.log-ca\
  160-Ethercat-MoveAbs-STOP.log-ca\
  200_Ethercat-powerOff-JOG-CNEN-Error.log-ca\
  240_Ethercat-JOGF_stopped.log-ca\
  400_Ethercat-JAR-ACCL-VAL-JOGF-HOMF.log-ca\
  410_Ethercat-ACCL_ACCS_VELO_TWF.log-ca\


all-ca: \
  $(RECORD-CA) \
  $(ETHERCATMC-CA) \
  $(SIMULATOR-ONE-CA) \
  $(SIMULATOR-TWO-CA) \


record-ca:  $(RECORD-CA) 


ethercatmc-ca: $(ETHERCATMC-CA)


simulator-one-ca: $(SIMULATOR-ONE-CA)


simulator-two-ca: $(SIMULATOR-TWO-CA)


%.log-ca : %.py
	./test-ioc-with-sim-indexer-one-TC-wrapper.sh ca://IOC:m1 $< $@

clean:
	rm -f *.log* *.log*.failed
