# ============================================================================
# 2D autonomous racing simulator -- build
# ============================================================================
#
# Two binaries out of one source tree:
#
#   build/racer  the simulator      (src/*.c, including main.c)
#   build/run_tests  the unit tests     (src/*.c except main.c, plus tests/*.c)
#
# Nothing is linked but libc and libm.

CC      ?= gcc

# -std=c11        the language this is written in, no compiler extensions
# -Wall -Wextra   the warnings that catch real bugs; treat them as errors you
#                 have not fixed yet, not as noise
# -g              debug symbols, so a sanitizer report names lines not addresses
CFLAGS  := -std=c11 -Wall -Wextra -g -Isrc
LDFLAGS :=
LDLIBS  := -lm

# ---------------------------------------------------------------------------
# Sanitizers
# ---------------------------------------------------------------------------
# AddressSanitizer catches out-of-bounds accesses, use-after-free and leaks.
# UndefinedBehaviorSanitizer catches signed overflow, bad shifts, misaligned
# loads and dereferences of NULL. Both are compile-time instrumentation: the
# program runs perhaps twice as slow and aborts with a stack trace the moment
# it does something wrong, instead of quietly corrupting memory and failing
# somewhere unrelated ten minutes later. For code that indexes flat arrays by
# hand -- which is all of this project -- they are the single most valuable
# flag you can pass.
#
# They must be given to BOTH the compiler and the linker, because they need
# their runtime library linked in as well as the instrumentation compiled in.
#
# Build with `make SANITIZE=0` to turn them off. You will want that if your
# toolchain does not ship the sanitizer runtimes -- notably mingw-w64 on
# Windows, where libasan and libubsan are not built. See README.md.
SANITIZE ?= 1
ifeq ($(SANITIZE),1)
    SAN     := -fsanitize=address,undefined
    CFLAGS  += $(SAN)
    LDFLAGS += $(SAN)
endif

# ---------------------------------------------------------------------------
# Sources
# ---------------------------------------------------------------------------
SRC_DIR   := src
TEST_DIR  := tests
BUILD_DIR := build

# Every .c under src/. main.c is pulled out separately because the test binary
# has its own main() and linking both would be a duplicate-symbol error.
ALL_SRCS  := $(wildcard $(SRC_DIR)/*.c)
LIB_SRCS  := $(filter-out $(SRC_DIR)/main.c,$(ALL_SRCS))
TEST_SRCS := $(wildcard $(TEST_DIR)/*.c)

LIB_OBJS  := $(patsubst %.c,$(BUILD_DIR)/%.o,$(LIB_SRCS))
MAIN_OBJ  := $(BUILD_DIR)/$(SRC_DIR)/main.o
TEST_OBJS := $(patsubst %.c,$(BUILD_DIR)/%.o,$(TEST_SRCS))

# Windows toolchains append .exe to whatever the linker is told to produce, so
# `make` would build build/racer.exe and then the run recipes below would look
# for build/racer and fail. Ask the environment rather than guessing.
EXE := $(if $(filter Windows_NT,$(OS)),.exe,)

RACER := $(BUILD_DIR)/racer$(EXE)
TESTS := $(BUILD_DIR)/run_tests$(EXE)

# Default track for `make run`. Override on the command line:
#   make run TRACK=Silverstone
TRACK ?= IMS
MAP   := maps/$(TRACK)/$(TRACK)_map.yaml
LINE  := maps/$(TRACK)/$(TRACK)_centerline.csv

# ---------------------------------------------------------------------------
# Rules
# ---------------------------------------------------------------------------
.PHONY: all test run render clean help

all: $(RACER) $(TESTS)

$(RACER): $(LIB_OBJS) $(MAIN_OBJ)
	@mkdir -p $(dir $@)
	$(CC) $(LDFLAGS) -o $@ $^ $(LDLIBS)

$(TESTS): $(LIB_OBJS) $(TEST_OBJS)
	@mkdir -p $(dir $@)
	$(CC) $(LDFLAGS) -o $@ $^ $(LDLIBS)

# One pattern rule covers src/ and tests/ alike, mirroring the source tree
# under build/ so object files never sit next to the code.
$(BUILD_DIR)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c -o $@ $<

# Header changes must trigger a rebuild. Listing every header against every
# object is coarse -- it rebuilds more than strictly necessary -- but this is a
# fifteen-file project that compiles in under a second, and the alternative
# (generated dependency files) is machinery to maintain for no gain here.
$(LIB_OBJS) $(MAIN_OBJ) $(TEST_OBJS): $(wildcard $(SRC_DIR)/*.h)

test: $(TESTS)
	./$(TESTS)

run: $(RACER)
	./$(RACER) $(MAP) $(LINE)

render: $(RACER)
	./$(RACER) $(MAP) $(LINE) --render

clean:
	rm -rf $(BUILD_DIR)

help:
	@echo "targets:"
	@echo "  make            build build/racer and build/run_tests"
	@echo "  make test       build and run the unit tests"
	@echo "  make run        drive a lap of \$$TRACK (default IMS)"
	@echo "  make render     the same, with the ASCII view"
	@echo "  make clean      remove build/"
	@echo ""
	@echo "variables:"
	@echo "  TRACK=Silverstone   which map under maps/ to use"
	@echo "  SANITIZE=0          build without ASan/UBSan"
	@echo "  CC=clang            use a different compiler"
