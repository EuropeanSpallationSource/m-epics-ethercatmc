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
  901-Simulator-SPMG.log-ca-m1\
  910_Simulator_JOGF_BDST.log-ca-m1\
  920_Simulator_RTRY_FRAC_RMOD.log-ca-m1\
  930_Simulator_Vel-RB_Acc-RB.log-ca-m1\
  931_Simulator-ACCL_ACCS_VELO_TWF.log-ca-m1\
  940_Simulator-powerOnFailed.log-ca-m1\

SIMULATOR-TWO-CA = \
  950_Simulator-JOGF-JOGR-JVEL-MRES.log-ca-m1\
  960_Simulator-motorFlags.log-ca-m1\
  980_Simulator-networkproblems.log-ca-m1\


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
  240_Ethercat-JOGF_stopped.log-ca-m1\
  400_Ethercat-JAR-ACCL-VAL-JOGF-HOMF.log-ca-m1\

##################################
SIMULATOR-ONE-PVA = \
  900_Simulator-Softlimits.log-pva-m1\
  901-Simulator-SPMG.log-pva-m1\
  910_Simulator_JOGF_BDST.log-pva-m1\
  920_Simulator_RTRY_FRAC_RMOD.log-pva-m1\
  930_Simulator_Vel-RB_Acc-RB.log-pva-m1\
  931_Simulator-ACCL_ACCS_VELO_TWF.log-pva-m1\
  940_Simulator-powerOnFailed.log-pva-m1\

SIMULATOR-TWO-PVA = \
  950_Simulator-JOGF-JOGR-JVEL-MRES.log-pva-m1\
  960_Simulator-motorFlags.log-pva-m1\
  980_Simulator-networkproblems.log-pva-m1\


RECORD-PVA = \
  100_Record-HOMF.log-pva-m1\
  121_Record-JOGF_DIR_DLY_HLM.log-pva-m1\
  131_Record-JOGR_DIR_DLY_LLM.log-pva-m1\
  140_Record-VAL_VMAX.log-pva-m1\
  141_Record_NTM.log-pva-m1\
  220_Record_auto_power.log-pva-m1\
  500_Record-DLY-STOP.log-pva-m1\


ETHERCATMC-PVA = \
  010_Basic_Communication.log-pva-m1\
  090_Ethercat-SmokeTest-TWF_CNEN.log-pva-m1\
  110_Ethercat_HOMF_-ProcHom.log-pva-m1\
  122_Ethercat-JOGF_HLS.log-pva-m1\
  123_Ethercat-MoveVel_MCU_Hsoftlimit.log-pva-m1\
  132_Ethercat-JOGR_LLS.log-pva-m1\
  133_Ethercat_MoveVel_MCU_Lsoftlimit.log-pva-m1\
  160-Ethercat-MoveAbs-STOP.log-pva-m1\
  201_Ethercat-JOG-VAL-HOM-CNEN-No-Error.log-pva-m1\
  240_Ethercat-JOGF_stopped.log-pva-m1\
  400_Ethercat-JAR-ACCL-VAL-JOGF-HOMF.log-pva-m1\


all-pva: \
  $(RECORD-PVA) \
  $(ETHERCATMC-PVA) \
  $(SIMULATOR-ONE-PVA) \
  $(SIMULATOR-TWO-PVA) \


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



###################################################
record-pva:  $(RECORD-PVA) 


ethercatmc-pva: $(ETHERCATMC-PVA)


simulator-one-pva: $(SIMULATOR-ONE-PVA)


simulator-two-pva: $(SIMULATOR-TWO-PVA)


%.log-pva-m1 : %.py
	./test-ioc-with-sim-indexer-one-TC-wrapper.sh pva://IOC:m1 $< $@





clean:
	rm -f *.log* *.log*.failed [1-9]*.txt
