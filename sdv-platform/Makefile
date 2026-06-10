CXX      ?= g++
CXXFLAGS ?= -I. -std=c++17 -Wall -Wextra -Werror -O2 -pthread
BUILD    := build

SRCS_COMMON := drivers/can/can_interface.cpp services/ota/ota_manager.cpp

.PHONY: all demo tests run-demo run-tests clean
all: demo tests

$(BUILD):
	mkdir -p $(BUILD)

demo: $(BUILD)
	$(CXX) $(CXXFLAGS) apps/demo/main.cpp $(SRCS_COMMON) -o $(BUILD)/sdv_demo

tests: $(BUILD)
	$(CXX) $(CXXFLAGS) tests/test_all.cpp $(SRCS_COMMON) -o $(BUILD)/sdv_tests

run-tests: tests
	./$(BUILD)/sdv_tests

run-demo: demo
	python3 tools/generate_update_package.py --out /tmp/sdv-ota-demo/incoming --version 1
	rm -rf /tmp/sdv-ota-demo/slot_a /tmp/sdv-ota-demo/slot_b /tmp/sdv-ota-demo/state
	./$(BUILD)/sdv_demo

clean:
	rm -rf $(BUILD)
