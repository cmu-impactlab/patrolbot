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

HEADERS = protocol.hpp undock.hpp

# Neither test binary links ARIA, so both build and run on a workstation.
TESTS = tests/protocol_test tests/undock_test

.PHONY: all clean test probe

all: $(TARGET)

probe: $(PROBE)

$(TARGET): $(SOURCES) $(HEADERS)
	$(CXX) $(CXXFLAGS) $(ARIA_CXXFLAGS) $(SOURCES) -o $@ $(ARIA_LDFLAGS) $(ARIA_LIBS)

$(PROBE): $(PROBE_SOURCES)
	$(CXX) $(CXXFLAGS) $(ARIA_CXXFLAGS) $(PROBE_SOURCES) -o $@ $(ARIA_LDFLAGS) $(ARIA_LIBS)

tests/protocol_test: tests/protocol_test.cpp protocol.cpp $(HEADERS)
	$(CXX) $(CXXFLAGS) tests/protocol_test.cpp protocol.cpp -o $@

tests/undock_test: tests/undock_test.cpp undock.cpp $(HEADERS)
	$(CXX) $(CXXFLAGS) tests/undock_test.cpp undock.cpp -o $@

test: $(TESTS)
	./tests/protocol_test
	./tests/undock_test

clean:
	rm -f $(TARGET) $(PROBE) $(TESTS)
