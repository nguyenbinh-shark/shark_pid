# Bộ test shark_pid — chạy trên máy tính, không cần vi điều khiển.
#
#   make test     biên dịch và chạy
#   make asan     chạy lại dưới AddressSanitizer + UBSan
#   make clean

CC      ?= cc
CSTD    ?= -std=c99
OPT     ?= -O2
WERROR  ?=

WARN = -Wall -Wextra -Wpedantic -Wshadow -Wstrict-prototypes \
       -Wmissing-prototypes -Wdouble-promotion -Wundef -Wcast-align

INC    = -I../src
SRC    = ../src/shark_pid.c test_shark_pid.c
DEPS   = ../src/shark_pid.h Makefile
BIN    = shark_pid_test
LDLIBS = -lm

.PHONY: all test asan clean

all: $(BIN)

$(BIN): $(SRC) $(DEPS)
	$(CC) $(CSTD) $(OPT) $(WARN) $(WERROR) $(INC) $(SRC) -o $@ $(LDLIBS)

test: $(BIN)
	./$(BIN)

asan: $(SRC) $(DEPS)
	$(CC) $(CSTD) -O1 -g $(WARN) $(WERROR) \
	    -fsanitize=address,undefined -fno-omit-frame-pointer \
	    $(INC) $(SRC) -o $(BIN)_asan $(LDLIBS)
	./$(BIN)_asan

clean:
	rm -f $(BIN) $(BIN)_asan
	rm -rf $(BIN).dSYM $(BIN)_asan.dSYM
