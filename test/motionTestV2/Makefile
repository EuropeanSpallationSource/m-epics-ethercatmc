
AXISFILES =\
  AxisComm.py\
  AxisEPICS.py\
  AxisTestUtil.py\


TESTS-PART1-CA = \
  300-Basic-Communication.log-ca-m1\
  320-SmokeTest-LS.log-ca-m1\


TESTS-PART1-PVA = \
  300-Basic-Communication.log-pva-m1\
  320-SmokeTest-LS.log-pva-m1\

# make (the default target)
all: \
  $(TESTS-PART1-CA) \
  $(TESTS-PART1-PVA) \


all-ca: \
  $(TESTS-PART1-CA) \


all-pva: \
  $(TESTS-PART1-PVA) \


# make clean
clean:
	rm -f *.log* *.log*.failed [1-9]*.txt


# Dependencies to make sure that a change in "any" py file
# triggers at least one action
300-Basic-Communication.log-ca-m1: \
  300-Basic-Communication.py \
  $(AXISFILES) \
  Makefile \
  ConnCA.py

## Rules

300-Basic-Communication.log-pva-m1: \
  300-Basic-Communication.py \
  $(AXISFILES) \
  Makefile \
  ConnP4P.py

## Rules
%.log-ca-m1 : %.py
	./test-ioc-with-sim-indexer-one-TC-wrapper.sh ca://IOC:m1 $< $@

%.log-pva-m1 : %.py
	./test-ioc-with-sim-indexer-one-TC-wrapper.sh pva://IOC:m1 $< $@

