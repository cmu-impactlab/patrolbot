CXX ?= g++
CXXFLAGS ?= -Wall -Wextra -Wpedantic -O2 -std=c++11
ARIA_CXXFLAGS = -I/usr/local/Aria/include -I/usr/local/Aria/ArNetworking/include
ARIA_LDFLAGS = -L/usr/local/Aria/lib
ARIA_LIBS = -lAria -lArNetworking -lpthread

TARGET = patrolbot_server
SOURCES = patrolbot_server.cpp protocol.cpp

.PHONY: all clean test

all: $(TARGET)

$(TARGET): $(SOURCES) protocol.hpp
	$(CXX) $(CXXFLAGS) $(ARIA_CXXFLAGS) $(SOURCES) -o $@ $(ARIA_LDFLAGS) $(ARIA_LIBS)

tests/protocol_test: tests/protocol_test.cpp protocol.cpp protocol.hpp
	$(CXX) $(CXXFLAGS) tests/protocol_test.cpp protocol.cpp -o $@

test: tests/protocol_test
	./tests/protocol_test

clean:
	rm -f $(TARGET) tests/protocol_test
