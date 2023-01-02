

TESTS-PART1-CA = \
  010-Basic-Communication.log-ca-m1\
  090-Hardware-Ethercat-SmokeTest-LS.log-ca-m1\


TESTS-PART1-PVA = \
  010-Basic-Communication.log-pva-m1\
  090-Hardware-Ethercat-SmokeTest-LS.log-pva-m1\

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


## Rules
%.log-ca-m1 : %.py
	./test-ioc-with-sim-indexer-one-TC-wrapper.sh ca://IOC:m1 $< $@

%.log-pva-m1 : %.py
	./test-ioc-with-sim-indexer-one-TC-wrapper.sh pva://IOC:m1 $< $@

