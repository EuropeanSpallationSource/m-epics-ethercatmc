
GNERATEDOPIS = \
  ./e-cnen.mid \
  ./e-foff.mid \
  ./e-homf-homr.mid \
  ./motorx-hxpd.mid \
  ./motorx-pils.mid \
  ../motor-0-ptp.opi \
  ../motor-1.opi \
  ../motor-1-ptp.opi \
  ../motor-1-pils-ptp-Max-Red-Current-para195-197.opi \
  ../motor-2.opi \
  ../motor-3-pils.opi \
  ../motor-3-ptp.opi \
  ../motor-4-pils.opi \
  ../motor-4-ptp.opi \
  ../motor-4-ecmc.opi \
  ../motor-5-pils.opi \
  ../motor-6-hxpd.opi \
  ../motor-6-pils.opi  \
  ../motor-4x2-pils.opi \
  ../motor-4x4.opi \
  ../motor-4x4-temp.opi \
  ../motor-estiaSel1-temp.opi \
  ../motor-estiaSel2-temp.opi \
  ../motor-ymir-mcs1-pils.opi \
  ../motor-ymir-mcs2-pils.opi \
  ../ethercatmcaxisConfig.opi \
  ../ethercatmcaxisConfig-pils.opi \
  ../ethercatmcaxisConfig-pils-ptp-Max-Red-Current-para195-197.opi \
  ../ethercatmcaxisExpert.opi \
  ../ethercatmcaxisExpert-ecmc.opi \
  ../ethercatmcaxisExpert-pils.opi \
  ../ethercatmcaxisExpert-pils-ptp.opi \
  ../ethercatmcaxisExpert-pils-ptp-Max-Red-Current-para195-197.opi \
  ../ethercatmcPTPErrBits.opi \
  ../ethercatmcShutter.opi \
  ../ethercatmcStatusWord1802.opi \
  ../shutter-1.opi \
  ../IOC.opi \


#  ../ethercatmcaxisConfig-ecmc.opi \

#Main target:
all: checkwhitespace $(GNERATEDOPIS)

clean:
	rm -f $(GNERATEDOPIS)

DEPENDENCIES_FOR_MOTORX_HXPD_OPI = \
	m-motor.mid \
	m-alarm-sevr.mid \
        m-slip-problem-not-homed-poweroff.mid \
	m-homf.mid \


DEPENDENCIES_FOR_MOTORX_PILS_OPI = \
	m-motor.mid \
	m-alarm-sevr.mid \
        m-slip-problem-not-homed-poweroff.mid \
	m-errrst.mid \
	m-inhibit.mid \
	m-msgtxt.mid \


DEPENDENCIES_FOR_CONFIG_OPI = \
        ethercatmcaxisConfig.start \
        ethercatmcaxisConfig.mid \
        ethercatmcaxisConfig.end

DEPENDENCIES_FOR_CONFIG_PILS_OPI = \
        ethercatmcaxisConfig.start \
        ethercatmcaxisConfig-pils.mid \
        ethercatmcaxisConfigCustomPara.mid \
        ethercatmcaxisConfigParaDesc.mid \
        ethercatmcaxisConfig.end


# Depencies, when do we need to revuild a .opi file
DEPENDENCIES_FOR_MOTOR_OPI = Makefile
DEPENDENCIES_FOR_MOTOR_OPI += genMotorShutter.sh
DEPENDENCIES_FOR_MOTOR_OPI += shiftopi.py
DEPENDENCIES_FOR_MOTOR_OPI += motorx.start
DEPENDENCIES_FOR_MOTOR_OPI += motorx-hxpd.mid
DEPENDENCIES_FOR_MOTOR_OPI += motorx-pils.mid
DEPENDENCIES_FOR_MOTOR_OPI += m-egu-rbv.mid
DEPENDENCIES_FOR_MOTOR_OPI += m-temp-rbv.mid
DEPENDENCIES_FOR_MOTOR_OPI += Cabinet.mid
DEPENDENCIES_FOR_MOTOR_OPI += plcIPADDR_PORT.mid
DEPENDENCIES_FOR_MOTOR_OPI += plcName.mid
DEPENDENCIES_FOR_MOTOR_OPI += ptp.mid
DEPENDENCIES_FOR_MOTOR_OPI += PTPdiffNTtime_MCU.mid
DEPENDENCIES_FOR_MOTOR_OPI += PTPdiffTimeIOC_MCU.mid
DEPENDENCIES_FOR_MOTOR_OPI += ptp-ts-ns.mid
DEPENDENCIES_FOR_MOTOR_OPI += openPTPErrBits.mid
DEPENDENCIES_FOR_MOTOR_OPI += motorx.end
DEPENDENCIES_FOR_SHUTTER_OPI = Makefile
DEPENDENCIES_FOR_SHUTTER_OPI += genMotorShutter.sh
DEPENDENCIES_FOR_SHUTTER_OPI += shutterx.start
DEPENDENCIES_FOR_SHUTTER_OPI += shutterx.mid
DEPENDENCIES_FOR_SHUTTER_OPI += plcIPADDR_PORT.mid
DEPENDENCIES_FOR_SHUTTER_OPI += plcName.mid
DEPENDENCIES_FOR_SHUTTER_OPI += ptp.mid
DEPENDENCIES_FOR_SHUTTER_OPI += openPTPErrBits.mid
DEPENDENCIES_FOR_SHUTTER_OPI += shutterx.end


DEPENDENCIES_FOR_STATUSWORD_OPI = Makefile
DEPENDENCIES_FOR_STATUSWORD_OPI += genExpertWithAuxBits.sh
DEPENDENCIES_FOR_STATUSWORD_OPI += genExpertWithAuxBits.py
DEPENDENCIES_FOR_STATUSWORD_OPI += ethercatmcStatusWord1802.start
DEPENDENCIES_FOR_STATUSWORD_OPI += ethercatmcStatusWord1802.mid
DEPENDENCIES_FOR_STATUSWORD_OPI += ethercatmcaxisAuxBit.mid
DEPENDENCIES_FOR_STATUSWORD_OPI += ethercatmcStatusWord1802.end
DEPENDENCIES_FOR_STATUSWORD_OPI += openPTPErrBits.mid


DEPENDENCIES_FOR_PTPERRORSTATUS_OPI = Makefile
DEPENDENCIES_FOR_PTPERRORSTATUS_OPI += genExpertWithAuxBits.sh
DEPENDENCIES_FOR_PTPERRORSTATUS_OPI += genExpertWithAuxBits.py
DEPENDENCIES_FOR_PTPERRORSTATUS_OPI += ethercatmcPTPErrBits.start
DEPENDENCIES_FOR_PTPERRORSTATUS_OPI += ethercatmcPTPErrBits.mid
DEPENDENCIES_FOR_PTPERRORSTATUS_OPI += ethercatmcaxisAuxBit.mid
DEPENDENCIES_FOR_PTPERRORSTATUS_OPI += ethercatmcPTPErrBits.end
DEPENDENCIES_FOR_PTPERRORSTATUS_OPI += ptp-ts-ns-pos-neg.mid
DEPENDENCIES_FOR_PTPERRORSTATUS_OPI += ptp-ts-ns.mid
DEPENDENCIES_FOR_PTPERRORSTATUS_OPI += PTPdiffTimeIOC_MCU-HIGH-LOW.mid



# expert screens. e- stands for expert
DEPENDENCIES_FOR_MOTOR_EXPERT_OPI = Makefile
DEPENDENCIES_FOR_MOTOR_EXPERT_OPI += e-cnen.mid
DEPENDENCIES_FOR_MOTOR_EXPERT_OPI += e-cnen-vis.mid
DEPENDENCIES_FOR_MOTOR_EXPERT_OPI += e-ecmc.mid
DEPENDENCIES_FOR_MOTOR_EXPERT_OPI += e-errrst.mid
DEPENDENCIES_FOR_MOTOR_EXPERT_OPI += e-foff.mid
DEPENDENCIES_FOR_MOTOR_EXPERT_OPI += e-foff-vis.mid
DEPENDENCIES_FOR_MOTOR_EXPERT_OPI += e-homf-homr.mid
DEPENDENCIES_FOR_MOTOR_EXPERT_OPI += e-homf-homr-vis.mid
DEPENDENCIES_FOR_MOTOR_EXPERT_OPI += e-homproc-rb.mid
DEPENDENCIES_FOR_MOTOR_EXPERT_OPI += e-inhibitf-inhibitr.mid
DEPENDENCIES_FOR_MOTOR_EXPERT_OPI += e-retry.mid
DEPENDENCIES_FOR_MOTOR_EXPERT_OPI += e-urip.mid
DEPENDENCIES_FOR_MOTOR_EXPERT_OPI += e-pils-cfgdxlm-en.mid
DEPENDENCIES_FOR_MOTOR_EXPERT_OPI += e-pils-errtxt.mid
DEPENDENCIES_FOR_MOTOR_EXPERT_OPI += e-pils-status-bit24-25.mid
DEPENDENCIES_FOR_MOTOR_EXPERT_OPI += e-pils-statuscode.mid
DEPENDENCIES_FOR_MOTOR_EXPERT_OPI += e-vel-acc-RB-act.mid
DEPENDENCIES_FOR_MOTOR_EXPERT_OPI += ethercatmcaxisExpert.start
DEPENDENCIES_FOR_MOTOR_EXPERT_OPI += ethercatmcaxisExpert.mid
DEPENDENCIES_FOR_MOTOR_EXPERT_OPI += ethercatmcaxisAuxBit.mid
DEPENDENCIES_FOR_MOTOR_EXPERT_OPI += ethercatmcaxisExpert.end
DEPENDENCIES_FOR_MOTOR_EXPERT_OPI += genExpertWithAuxBits.sh
DEPENDENCIES_FOR_MOTOR_EXPERT_OPI += genExpertWithAuxBits.py
DEPENDENCIES_FOR_MOTOR_EXPERT_OPI += ptp.mid

DEPENDENCIES_FOR_SHUTTER_EXPERT_OPI = Makefile
DEPENDENCIES_FOR_SHUTTER_EXPERT_OPI += genExpertWithAuxBits.sh
DEPENDENCIES_FOR_SHUTTER_EXPERT_OPI += genExpertWithAuxBits.py
DEPENDENCIES_FOR_SHUTTER_EXPERT_OPI += ethercatmcShutter.start
DEPENDENCIES_FOR_SHUTTER_EXPERT_OPI += Cabinet.mid
DEPENDENCIES_FOR_SHUTTER_EXPERT_OPI += plcName.mid
DEPENDENCIES_FOR_SHUTTER_EXPERT_OPI += ethercatmcShutter.mid
DEPENDENCIES_FOR_SHUTTER_EXPERT_OPI += ethercatmcaxisAuxBit.mid
DEPENDENCIES_FOR_SHUTTER_EXPERT_OPI += ptp.mid
DEPENDENCIES_FOR_SHUTTER_EXPERT_OPI += ethercatmcShutter.end


AUXBITS_EXPERT="20 19 18 17 16 15 14 13 12 11 10 9 8 7 6 5 4 3 2 1 0"
AUXBITS_SHUTTER="23 22 21 20 19 18 17 16 15 14 13 5 4 3 2 1 0"
AUXBITS_STATUSWORD="0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23"
AUXBITS_PTPERRORSTATUS="15 14 13 12 11 10 9 8 7 6 5 4 3 2 1 0"


./e-cnen.mid: e-cnen-vis.mid Makefile
	sed -e '/<rules>/,/<\/rules>/d'<e-cnen-vis.mid >$@

./e-foff.mid: e-foff-vis.mid Makefile
	sed -e '/<rules>/,/<\/rules>/d'<e-foff-vis.mid >$@

./e-homf-homr.mid: e-homf-homr-vis.mid Makefile
	sed -e '/<rules>/,/<\/rules>/d'<e-homf-homr-vis.mid >$@

./motorx-hxpd.mid: $(DEPENDENCIES_FOR_MOTORX_HXPD_OPI) Makefile
	echo $(DEPENDENCIES_FOR_MOTORX_HXPD_OPI) '>$@'
	cat $(DEPENDENCIES_FOR_MOTORX_HXPD_OPI) >$@

./motorx-pils.mid: $(DEPENDENCIES_FOR_MOTORX_PILS_OPI) Makefile
	echo $(DEPENDENCIES_FOR_MOTORX_PILS_OPI) '>$@'
	cat $(DEPENDENCIES_FOR_MOTORX_PILS_OPI) >$@

../motor-0-ptp.opi: $(DEPENDENCIES_FOR_MOTOR_OPI)
	./genMotorShutter.sh $@ pils ptp

../motor-1.opi: $(DEPENDENCIES_FOR_MOTOR_OPI)
	./genMotorShutter.sh $@ 1 x 1

../motor-1-pils.opi: $(DEPENDENCIES_FOR_MOTOR_OPI)
	./genMotorShutter.sh $@ pils 1 x 1

../motor-1-ptp.opi: $(DEPENDENCIES_FOR_MOTOR_OPI)
	./genMotorShutter.sh $@ pils ptp 1 x 1

../motor-1-pils-ptp-Max-Red-Current-para195-197.opi: ../motor-1-ptp.opi
	sed -e 's/ethercatmcaxisExpert-pils-ptp/ethercatmcaxisExpert-pils-ptp-Max-Red-Current-para195-197/' < ../motor-1-ptp.opi >$@

../motor-2.opi: $(DEPENDENCIES_FOR_MOTOR_OPI)
	./genMotorShutter.sh $@ 2 x 1

../motor-2-pils.opi: $(DEPENDENCIES_FOR_MOTOR_OPI)
	./genMotorShutter.sh $@ pils 2 x 1

../motor-3-pils.opi: $(DEPENDENCIES_FOR_MOTOR_OPI)
	./genMotorShutter.sh $@ pils 3 x 1

../motor-3-ptp.opi: $(DEPENDENCIES_FOR_MOTOR_OPI)
	./genMotorShutter.sh $@ pils ptp 3 x 1

../motor-4-pils.opi: $(DEPENDENCIES_FOR_MOTOR_OPI)
	./genMotorShutter.sh $@ pils 4 x 1

../motor-4-ecmc.opi: $(DEPENDENCIES_FOR_MOTOR_OPI)
	./genMotorShutter.sh $@ ecmc 4 x 1

../motor-4-ptp.opi: $(DEPENDENCIES_FOR_MOTOR_OPI)
	./genMotorShutter.sh $@ pils ptp 4 x 1

../motor-5-pils.opi: $(DEPENDENCIES_FOR_MOTOR_OPI)
	./genMotorShutter.sh $@ pils 5 x 1

../motor-6-hxpd.opi: $(DEPENDENCIES_FOR_MOTOR_OPI)
	./genMotorShutter.sh $@ hxpd 6 x 1

../motor-6-pils.opi: $(DEPENDENCIES_FOR_MOTOR_OPI)
	./genMotorShutter.sh $@ pils 6 x 1

../motor-4x2-pils.opi: $(DEPENDENCIES_FOR_MOTOR_OPI)
	./genMotorShutter.sh $@ pils 4 x 2

../motor-4x4.opi: $(DEPENDENCIES_FOR_MOTOR_OPI)
	./genMotorShutter.sh $@ pils 4 x 4

../motor-4x4-temp.opi: $(DEPENDENCIES_FOR_MOTOR_OPI)
	./genMotorShutter.sh $@ pils temp m m m m n m m m m n m m m m n m m m m

../motor-estiaSel1-temp.opi: $(DEPENDENCIES_FOR_MOTOR_OPI) tempsensor.mid
	./genMotorShutter.sh $@ pils temp m m m m m n m m m m m n m m m n t t t t t n t t t t t n t t

../motor-estiaSel2-temp.opi: $(DEPENDENCIES_FOR_MOTOR_OPI) tempsensor.mid
	./genMotorShutter.sh $@ pils temp m m m m m n m m m m m n m m m m n t t t t t n t t t t t n t t

../motor-ymir-mcs1-pils.opi: $(DEPENDENCIES_FOR_MOTOR_OPI)
	./genMotorShutter.sh $@ pils m m m m m m n shutter s s s

../motor-ymir-mcs2-pils.opi: $(DEPENDENCIES_FOR_MOTOR_OPI)
	./genMotorShutter.sh $@ pils  m m m m m n m m m m m n m m m m m n m m m shutter s s

../ethercatmcaxisConfig.opi: $(DEPENDENCIES_FOR_CONFIG_OPI) Makefile
	cat $(DEPENDENCIES_FOR_CONFIG_OPI)  >$@

../ethercatmcaxisConfig-pils.opi: $(DEPENDENCIES_FOR_CONFIG_PILS_OPI) Makefile genAxisConfigCutomPara.sh
	./genAxisConfigCutomPara.sh $@

../ethercatmcaxisConfig-pils-ptp-Max-Red-Current-para195-197.opi: $(DEPENDENCIES_FOR_CONFIG_PILS_OPI) Makefile genAxisConfigCutomPara.sh
	./genAxisConfigCutomPara.sh $@ CfgMoveCurrent CfgIdleCurrent para195 para196 para197

# Generate ethercatmcaxisExpert*.opi out of
#   ethercatmcaxisExpert.mid and ethercatmcaxisExpert.end
../ethercatmcaxisExpert.opi: $(DEPENDENCIES_FOR_MOTOR_EXPERT_OPI)
	./genExpertWithAuxBits.sh $@ ethercatmcaxisExpert

../ethercatmcaxisExpert-pils.opi: $(DEPENDENCIES_FOR_MOTOR_EXPERT_OPI)
	./genExpertWithAuxBits.sh $@ ethercatmcaxisExpert pils $(AUXBITS_EXPERT)

../ethercatmcaxisExpert-ecmc.opi: $(DEPENDENCIES_FOR_MOTOR_EXPERT_OPI)
	./genExpertWithAuxBits.sh $@ ethercatmcaxisExpert ecmc

../ethercatmcaxisExpert-pils-ptp.opi: $(DEPENDENCIES_FOR_MOTOR_EXPERT_OPI)
	./genExpertWithAuxBits.sh $@ ethercatmcaxisExpert pils ptp $(AUXBITS_EXPERT)

../ethercatmcaxisExpert-pils-ptp-Max-Red-Current-para195-197.opi: ../ethercatmcaxisExpert-pils-ptp.opi
	sed -e 's/ethercatmcaxisConfig-pils.opi/ethercatmcaxisConfig-pils-ptp-Max-Red-Current-para195-197.opi/' < ../ethercatmcaxisExpert-pils-ptp.opi >$@

# Generate ethercatmcShutter*.opi out of
#   ethercatmcShutter.mid and ethercatmcShutter.end
../ethercatmcShutter.opi: $(DEPENDENCIES_FOR_SHUTTER_EXPERT_OPI)
	./genExpertWithAuxBits.sh $@ ethercatmcShutter $(AUXBITS_SHUTTER)

# Generate ethercatmcStatusWord1802*.opi out of
#   ethercatmcStatusWord1802.mid and ethercatmcStatusWord1802.end
../ethercatmcPTPErrBits.opi: $(DEPENDENCIES_FOR_PTPERRORSTATUS_OPI)
	/bin/sh -e -x ./genExpertWithAuxBits.sh $@ ethercatmcPTPErrBits ptp ptpdifftimeioc_mcu PTPdiffNTtime_mcu PTPdiffTcNTPExttime_mcu ptpTCdifftime_mcu ptptsnsposneg $(AUXBITS_PTPERRORSTATUS)

../ethercatmcStatusWord1802.opi: $(DEPENDENCIES_FOR_STATUSWORD_OPI)
	./genExpertWithAuxBits.sh $@ ethercatmcStatusWord1802 $(AUXBITS_STATUSWORD)

../shutter-1.opi: $(DEPENDENCIES_FOR_SHUTTER_OPI)
	./genMotorShutter.sh $@ pils shutter s

../IOC.opi: Makefile genIOC.sh IOC*.*
	./genIOC.sh ../IOC.opi IOC


checkwhitespace:
	../../../../checkws.sh


