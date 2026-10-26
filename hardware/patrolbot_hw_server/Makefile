CXX ?= g++
CXXFLAGS ?= -Wall -Wextra -Wpedantic -O2 -std=c++11
ARIA_CXXFLAGS = -I/usr/local/Aria/include -I/usr/local/Aria/ArNetworking/include
ARIA_LDFLAGS = -L/usr/local/Aria/lib
ARIA_LIBS = -lAria -lArNetworking -lpthread

TARGET = patrolbot_server
SOURCES = patrolbot_server.cpp protocol.cpp undock.cpp

# Commissioning tool. Built here but never installed beside the server: the base
# connection is single-owner, so only one of them may run at a time.
PROBE = patrolbot_commission_probe
PROBE_SOURCES = patrolbot_commission_probe.cpp

HEADERS = protocol.hpp undock.hpp scan_timing.hpp lidar_power.hpp encoder_capture.hpp manual_turn_scan.hpp

# Neither test binary links ARIA, so both build and run on a workstation.
TESTS = tests/protocol_test tests/undock_test tests/scan_timing_test tests/lidar_power_test tests/encoder_capture_test

.PHONY: all clean test probe

all: $(TARGET)

probe: $(PROBE)

$(TARGET): $(SOURCES) $(HEADERS)
	$(CXX) $(CXXFLAGS) $(ARIA_CXXFLAGS) $(SOURCES) -o $@ $(ARIA_LDFLAGS) $(ARIA_LIBS)

$(PROBE): $(PROBE_SOURCES)
	$(CXX) $(CXXFLAGS) $(ARIA_CXXFLAGS) $(PROBE_SOURCES) -o $@ $(ARIA_LDFLAGS) $(ARIA_LIBS)

tests/protocol_test: tests/protocol_test.cpp protocol.cpp undock.cpp $(HEADERS)
	$(CXX) $(CXXFLAGS) tests/protocol_test.cpp protocol.cpp undock.cpp -o $@

tests/undock_test: tests/undock_test.cpp undock.cpp $(HEADERS)
	$(CXX) $(CXXFLAGS) tests/undock_test.cpp undock.cpp -o $@

tests/scan_timing_test: tests/scan_timing_test.cpp scan_timing.hpp
	$(CXX) $(CXXFLAGS) tests/scan_timing_test.cpp -o $@

tests/lidar_power_test: tests/lidar_power_test.cpp lidar_power.hpp scan_timing.hpp
	$(CXX) $(CXXFLAGS) tests/lidar_power_test.cpp -o $@

tests/encoder_capture_test: tests/encoder_capture_test.cpp encoder_capture.hpp
	$(CXX) $(CXXFLAGS) tests/encoder_capture_test.cpp -o $@

test: $(TESTS)
	./tests/protocol_test
	./tests/undock_test
	./tests/scan_timing_test
	./tests/lidar_power_test
	./tests/encoder_capture_test

clean:
	rm -f $(TARGET) $(PROBE) $(TESTS)
