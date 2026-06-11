CXX      ?= g++
# C++20 required: std::jthread, std::span, concepts
CXXFLAGS ?= -I. -std=c++20 -Wall -Wextra -Wpedantic -Wno-unused-parameter \
            -Werror -O2 -pthread
BUILD    := build

SRCS_COMMON := drivers/can/can_interface.cpp services/ota/ota_manager.cpp

.PHONY: all demo demo-enhanced tests run-demo run-demo-enhanced run-tests clean lint

all: demo demo-enhanced tests

$(BUILD):
	mkdir -p $(BUILD)

demo: $(BUILD)
	$(CXX) $(CXXFLAGS) apps/demo/main.cpp $(SRCS_COMMON) -o $(BUILD)/sdv_demo

demo-enhanced: $(BUILD)
	$(CXX) $(CXXFLAGS) apps/demo/main_enhanced.cpp $(SRCS_COMMON) \
	    -o $(BUILD)/sdv_demo_enhanced

tests: $(BUILD)
	$(CXX) $(CXXFLAGS) tests/test_all.cpp $(SRCS_COMMON) -o $(BUILD)/sdv_tests

run-tests: tests
	./$(BUILD)/sdv_tests

run-demo: demo
	python3 tools/generate_update_package.py --out /tmp/sdv-ota-demo/incoming --version 1
	rm -rf /tmp/sdv-ota-demo/slot_a /tmp/sdv-ota-demo/slot_b /tmp/sdv-ota-demo/state
	./$(BUILD)/sdv_demo

run-demo-enhanced: demo-enhanced
	python3 tools/generate_update_package.py --out /tmp/sdv-ota-demo/incoming --version 1
	rm -rf /tmp/sdv-ota-demo/slot_a /tmp/sdv-ota-demo/slot_b /tmp/sdv-ota-demo/state
	./$(BUILD)/sdv_demo_enhanced

# Clang-tidy static analysis (requires clang-tidy installed)
lint:
	clang-tidy --extra-arg=-std=c++20 \
	    apps/demo/main_enhanced.cpp \
	    -- -I. -std=c++20 2>/dev/null || true

clean:
	rm -rf $(BUILD)
