# Compiler
CXX = g++
CXXFLAGS = -std=c++11 -Wall -Wextra

# Directories
SRCDIR = src
TESTDIR = tests
OUTDIR = out

IGNORE_FILES = $(SRCDIR)/IMU/* $(SRCDIR)/MySerial/* $(SRCDIR)/NMEA/* $(SRCDIR)/SD/*

LIB_TARGETS = $(filter-out $(IGNORE_FILES), $(wildcard $(SRCDIR)/*.cpp))

all : compile 

# Test executables
compile : 
	@mkdir -p $(OUTDIR)/tests

	echo $(IGNORE_FILES)

	@$(CXX) $(CXXFLAGS) tests/SERCOMM_Test.cpp src/SERCOMM/SERCOMM.cpp -o out/tests/TEST_SERCOMM
	@$(CXX) $(CXXFLAGS) tests/RingList_Test.cpp src/RingList/RingList.h -o out/tests/TEST_RingList
	@$(CXX) $(CXXFLAGS) tests/ErrorRegisters_Test.cpp src/ErrorRegister/ErrorRegister.cpp -o out/tests/TEST_ErrorRegister

test : compile
	@./out/tests/TEST_SERCOMM
	@./out/tests/TEST_RingList
	@./out/tests/TEST_ErrorRegister

# Clean up build files
clean:
	@rm -rf $(OUTDIR)

# Phony targets
.PHONY: all clean
