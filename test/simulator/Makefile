#
#
uname_S   := $(shell sh -c 'uname -s 2>/dev/null || echo unknown')
uname_M   := $(shell sh -c 'uname -m 2>/dev/null || echo unknown')
uname_R   := $(shell sh -c 'uname -r 2>/dev/null | sed -e "s/[()/]/-/g"|| echo unknown')

# Default for the binaries:
BIN            = $(uname_S)_$(uname_M)_$(uname_R)

CFLAGS         = -g -O0
CC             = Unkown_OS
CFLAGS         += -I.

ifeq ($(uname_S),Darwin)
CC             = gcc
CFLAGS         += -Wall -Werror
CFLAGS         += -O0 -g

endif

ifeq ($(uname_S),Linux)
CC             = gcc
CFLAGS         += -Wall -Werror
endif

ifneq (,$(findstring CYGWIN,$(uname_S)))
  ifneq (,$(findstring x86_64,$(uname_M)))
    CFLAGS        += -DSTART_WINSOCK2
    WINOBJS       += $(BIN)/startWinSock.o
  endif
  CC             = gcc
  CFLAGS         += -Wall -Werror
  #CFLAGS         += -DUSE_WINSOCK2
  EXE            = .exe
  LINKWINSOCK    = -lws2_32
endif

ifneq (,$(findstring MINGW,$(uname_S)))
  ifneq (,$(findstring MINGW32_NT-6.1-WOW,$(uname_S)))
    CC             = gcc
    CFLAGS         += -Wall -Werror
    CFLAGS         += -DUSE_WINSOCK2
    CFLAGS         += -DSTART_WINSOCK2
    #CFLAGS         += -O0 -g3 -Wall -fmessage-length=0
    EXE            = .exe
    WINOBJS       += $(BIN)/startWinSock.o
    LINKWINSOCK    = -lws2_32
  else
    CC             = gcc -mno-cygwin
    CFLAGS         += -Wall -Werror
    CFLAGS         += -DUSE_WINSOCK2
    CFLAGS         += -DSTART_WINSOCK2
    #CFLAGS         += -O0 -g3 -Wall -fmessage-length=0
    EXE            = .exe
    WINOBJS       += $(BIN)/startWinSock.o
    LINKWINSOCK    = -lws2_32
  endif

endif


#First target, done when we run "make" (and CC is unknown)
ifeq ($(CC),Unkown_OS)
unkown: 
	echo "Unsupported OS" $(uname_S)
	exit 1
endif

MOTOROBJS=\
 $(BIN)/cmd_Sim.o \
 $(BIN)/cmd_EAT.o \
 $(BIN)/cmd_IcePAP.o \
 $(BIN)/cmd_TCPsim.o \
 $(BIN)/hw_motor.o \
 $(BIN)/indexer.o

TELOBJS=\
 $(BIN)/ads.o \
 $(BIN)/main.o \
 $(BIN)/sock-ads.o \
 $(BIN)/sock-util.o \
 $(BIN)/cmd.o \
 $(BIN)/cmd_buf.o


#First target, done when we run "make" (and CC is known)
install: checkwhitespace mdbin $(BIN)/simMotor$(EXE)

checkwhitespace:
	./checkws.sh

mdbin:
	mkdir -p $(BIN)

clean:
	rm -f $(BIN)/*.o $(BIN)/*.exe

ALLOBJS=$(MOTOROBJS) $(TELOBJS) $(WINOBJS)

$(BIN)/simMotor$(EXE): $(ALLOBJS)
	$(CC) $(ALLOBJS) $(LINKWINSOCK) -o $@

$(BIN)/ads.o: \
 Makefile \
 ads.h \
 logerr_info.h \
 cmd_EAT.h \
 ads.c
	$(CC) -c $(CFLAGS) ads.c -o $@

$(BIN)/main.o: \
 Makefile \
 logerr_info.h \
 sock-util.h \
 main.c
	$(CC) -c $(CFLAGS) main.c -o $@

$(BIN)/sock-ads.o: \
 Makefile \
 ads.h \
 logerr_info.h \
 sock-util.h \
 sock-ads.h \
 sock-util.h \
 sock-ads.c
	$(CC) -c $(CFLAGS) sock-ads.c -o $@

$(BIN)/sock-util.o: \
 Makefile \
 cmd.h \
 logerr_info.h \
 sock-util.h \
 sock-ads.h \
 sock-util.c
	$(CC) -c $(CFLAGS) sock-util.c -o $@


$(BIN)/cmd.o: \
 Makefile \
 sock-util.h \
 hw_motor.h \
 cmd_IcePAP.h \
 cmd_TCPsim.h \
 cmd.c
	$(CC) -c $(CFLAGS) cmd.c -o $@

$(BIN)/cmd_buf.o: \
 Makefile \
 cmd_buf.h \
 max_axes.h \
 cmd_buf.c
	$(CC) -c $(CFLAGS) cmd_buf.c -o $@

$(BIN)/cmd_Sim.o: \
 Makefile \
 cmd_Sim.c \
 hw_motor.h \
 cmd_Sim.h
	$(CC) -c $(CFLAGS) cmd_Sim.c -o $@

$(BIN)/cmd_EAT.o: \
 Makefile \
 ads_defines.h \
 cmd_EAT.c \
 hw_motor.h \
 indexer.h \
 cmd_EAT.h
	$(CC) -c $(CFLAGS) cmd_EAT.c -o $@

$(BIN)/cmd_IcePAP.o: \
 Makefile \
 cmd_IcePAP.c \
 hw_motor.h \
 cmd_IcePAP.h \
 cmd_IcePAP-internal.h
	$(CC) -c $(CFLAGS) cmd_IcePAP.c -o $@

$(BIN)/cmd_TCPsim.o: \
 Makefile \
 cmd_TCPsim.c \
 hw_motor.h \
 cmd_TCPsim.h
	$(CC) -c $(CFLAGS) cmd_TCPsim.c -o $@

$(BIN)/hw_motor.o: \
 Makefile \
 hw_motor.h \
 max_axes.h \
 hw_motor.c
	$(CC) -c $(CFLAGS) hw_motor.c -o $@


$(BIN)/indexer.o: \
 Makefile \
 indexer.c \
 hw_motor.h \
 indexer.h \
 cmd.h \
 sock-util.h
	$(CC) -c $(CFLAGS) indexer.c -o $@

$(BIN)/startWinSock.o: \
 Makefile \
 startWinSock.h \
 startWinSock.c
	$(CC) -c $(CFLAGS)  startWinSock.c -o $@


