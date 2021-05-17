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
  900_Simulator-Softlimits.log-ca-m1\
  910_Simulator_JOGF_BDST.log-ca-m1\
  920_Simulator_RTRY_FRAC_RMOD.log-ca-m1\
  930_Simulator_Vel-RB_Acc-RB.log-ca-m1\
  940_Simulator-powerOnFailed.log-ca-m1\

SIMULATOR-TWO-CA = \
  950_Simulator-JOGF-JOGR-JVEL-MRES.log-ca-m1\
  960_Simulator-motorFlags.log-ca-m1\


RECORD-CA = \
  100_Record-HOMF.log-ca-m1\
  121_Record-JOGF_DIR_DLY_HLM.log-ca-m1\
  131_Record-JOGR_DIR_DLY_LLM.log-ca-m1\
  140_Record-VAL_VMAX.log-ca-m1\
  141_Record_NTM.log-ca-m1\
  220_Record_auto_power.log-ca-m1\
  500_Record-DLY-STOP.log-ca-m1\


ETHERCATMC-CA = \
  010_Basic_Communication.log-ca-m1\
  090_Ethercat-SmokeTest-TWF_CNEN.log-ca-m1\
  110_Ethercat_HOMF_-ProcHom.log-ca-m1\
  122_Ethercat-JOGF_HLS.log-ca-m1\
  123_Ethercat-MoveVel_MCU_Hsoftlimit.log-ca-m1\
  132_Ethercat-JOGR_LLS.log-ca-m1\
  133_Ethercat_MoveVel_MCU_Lsoftlimit.log-ca-m1\
  160-Ethercat-MoveAbs-STOP.log-ca-m1\
  201_Ethercat-JOG-VAL-HOM-CNEN-No-Error.log-ca-m1\
  203_Ethercat-HOM-ParamInterface.log-ca-m1\
  240_Ethercat-JOGF_stopped.log-ca-m1\
  400_Ethercat-JAR-ACCL-VAL-JOGF-HOMF.log-ca-m1\
  410_Ethercat-ACCL_ACCS_VELO_TWF.log-ca-m1\


all-ca: \
  $(RECORD-CA) \
  $(ETHERCATMC-CA) \
  $(SIMULATOR-ONE-CA) \
  $(SIMULATOR-TWO-CA) \


record-ca:  $(RECORD-CA) 


ethercatmc-ca: $(ETHERCATMC-CA)


simulator-one-ca: $(SIMULATOR-ONE-CA)


simulator-two-ca: $(SIMULATOR-TWO-CA)


%.log-ca-m1 : %.py
	./test-ioc-with-sim-indexer-one-TC-wrapper.sh ca://IOC:m1 $< $@

clean:
	rm -f *.log* *.log*.failed [1-9]*.txt
