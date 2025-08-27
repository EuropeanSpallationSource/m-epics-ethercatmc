#
# Makefile to run the different test case
# test cases are grouped into 3 categories:
# - (motor)Record : Should work with all motors
# - ethercatmc    : Works only with the ethercatmc driver
#                     additional ai/ao records are used which have a certain functionality
# - simulator     : Work only with ethercatmc & simulator
#                     Only here we can simulate e.g. a motor that has "stuck" in a certain position
#                     and triggers the retry-login in the motorRecord

##################################
SIMULATOR-ONE-PVA = \
  900_Simulator-MRES_HLM_LLM_DHLM_DLLM.log-pva-m1\
  901-Simulator-SPMG.log-pva-m1\
  910_Simulator_JOG_BDST_RTRY_FRAC.log-pva-m1\
  920_Simulator_RTRY_FRAC_RMOD.log-pva-m1\
  921_Simulator_HLS_LLS_do-not_move-further.log-pva-m1\
  922_Simulator_DLY_HLS_LLS_stall.log-pva-m1\
  930_Simulator_Vel-RB_Acc-RB.log-pva-m1\
  931_Simulator-ACCL_ACCS_VELO_TWF.log-pva-m1\

SIMULATOR-TWO-PVA = \
  940_Simulator-InputRecordAlarm.log-pva-m1\
  941_Simulator-MsgTxt-ErrTxt.log-pva-m1\
  942_Simulator_Cabinet.log-pva-m1\
  943_Simulator_Pneumatics.log-pva-pneu \
  950_Simulator-JOGF-JOGR-JVEL-MRES.log-pva-m1\
  960_Simulator-motorFlags.log-pva-m1\
  980_Simulator-networkproblems.log-pva-m1\


RECORD-PVA = \
  100_Record-HOMF.log-pva-m1\
  101_Record-VAL-HOMF-VAL.log-pva-m1\
  121_Record-JOGF_DIR_DLY_HLM.log-pva-m1\
  131_Record-JOGR_DIR_DLY_LLM.log-pva-m1\
  140_Record-VAL_VMAX.log-pva-m1\
  141_Record_NTM.log-pva-m1\
  220_Ethercat-auto-power.log-pva-m1\
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

#################################
# The main target
all: \
  $(RECORD-PVA) \
  $(ETHERCATMC-PVA) \
  $(SIMULATOR-ONE-PVA) \
  $(SIMULATOR-TWO-PVA) \

# clean
clean:
	rm -f *.log* *.log*.failed [1-9]*.txt

# targets called from ../Makefile
record-pva:  $(RECORD-PVA)

ethercatmc-pva: $(ETHERCATMC-PVA)

simulator-one-pva: $(SIMULATOR-ONE-PVA)

simulator-two-pva: $(SIMULATOR-TWO-PVA)

monitor-ca-pva: \
  100_Record-HOMF.log-ca-m1\
  100_Record-HOMF.log-pva-m1\

# rules for ca and pva
%.log-ca-m1 : %.py
	./test-ioc-with-sim-indexer-one-TC-wrapper.sh ca://IOC:m1 $< $@

%.log-pva-m1 : %.py
	./test-ioc-with-sim-indexer-one-TC-wrapper.sh pva://IOC:m1 $< $@

%.log-pva-pneu : %.py
	./test-ioc-with-sim-indexer-one-TC-wrapper.sh pva://IOC:Pneu $< $@
