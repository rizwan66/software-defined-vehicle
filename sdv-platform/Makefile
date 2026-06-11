CXX      ?= g++
# C++23: std::expected, std::flat_map (wave 2)
# C++20: std::jthread, std::span, concepts, __VA_OPT__ (wave 1)
CXXFLAGS ?= -I. -std=c++23 -Wall -Wextra -Wpedantic -Wno-unused-parameter \
            -Werror -O2 -pthread
BUILD    := build

SRCS_COMMON := drivers/can/can_interface.cpp services/ota/ota_manager.cpp

.PHONY: all demo demo-enhanced demo-v2 tests \
        run-demo run-demo-enhanced run-demo-v2 run-tests clean lint

all: demo demo-enhanced demo-v2 tests

$(BUILD):
	mkdir -p $(BUILD)

demo: $(BUILD)
	$(CXX) $(CXXFLAGS) apps/demo/main.cpp $(SRCS_COMMON) -o $(BUILD)/sdv_demo

demo-enhanced: $(BUILD)
	$(CXX) $(CXXFLAGS) apps/demo/main_enhanced.cpp $(SRCS_COMMON) \
	    -o $(BUILD)/sdv_demo_enhanced

demo-v2: $(BUILD)
	$(CXX) $(CXXFLAGS) apps/demo/main_v2.cpp $(SRCS_COMMON) \
	    -o $(BUILD)/sdv_demo_v2

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

run-demo-v2: demo-v2
	python3 tools/generate_update_package.py --out /tmp/sdv-ota-demo/incoming --version 1
	rm -rf /tmp/sdv-ota-demo/slot_a /tmp/sdv-ota-demo/slot_b /tmp/sdv-ota-demo/state
	./$(BUILD)/sdv_demo_v2

# Clang-tidy static analysis
lint:
	clang-tidy --extra-arg=-std=c++23 \
	    apps/demo/main_v2.cpp \
	    -- -I. -std=c++23 2>/dev/null || true

clean:
	rm -rf $(BUILD)
