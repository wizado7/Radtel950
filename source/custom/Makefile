PREFIX ?= arm-none-eabi-
OUT := build
CXXFLAGS := -std=c++17 -Wall -Wextra -Werror -Wshadow -fno-exceptions -fno-rtti
ARMFLAGS := -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -Os -ffreestanding -fno-builtin -ffunction-sections -fdata-sections
.PHONY: all test arm-check firmware clean
CSOURCES := ../src/startup.c ../src/system.c ../src/drivers/gpio.c ../src/drivers/lcd.c ../src/app/encoder.c ../diagnostic/nmea_status.c ../mvp/core.c board.c battery.c runtime.c
CPPSOURCES := ui_text.cpp ui_home.cpp display_power.cpp application.cpp controls.cpp main.cpp ui.cpp gps_view.cpp locale.cpp radio_settings.cpp settings_snapshot.cpp settings_flash.cpp fm_board.cpp voice_rx.cpp voice_factory_board.cpp
COBJECTS := $(addprefix $(OUT)/c/,$(notdir $(CSOURCES:.c=.o)))
CPPOBJECTS := $(addprefix $(OUT)/cpp/,$(CPPSOURCES:.cpp=.o))
-include $(COBJECTS:.o=.d) $(CPPOBJECTS:.o=.d)
.DEFAULT_GOAL := all
vpath %.c . ../src ../src/drivers ../src/app ../diagnostic ../mvp
all: test firmware
$(OUT):
	mkdir -p $(OUT)
test: | $(OUT)
	gcc -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined -c ../mvp/core.c -o $(OUT)/core-host.o
	g++ $(CXXFLAGS) -fsanitize=address,undefined -g application.cpp test_application.cpp $(OUT)/core-host.o -o $(OUT)/test-application
	ASAN_OPTIONS=detect_leaks=0 ./$(OUT)/test-application
	g++ $(CXXFLAGS) -fsanitize=address,undefined -g controls.cpp application.cpp test_controls.cpp $(OUT)/core-host.o -o $(OUT)/test-controls
	ASAN_OPTIONS=detect_leaks=0 ./$(OUT)/test-controls
arm-check: | $(OUT)
	$(PREFIX)gcc $(ARMFLAGS) -std=c11 -Wall -Wextra -Werror -c ../mvp/core.c -o $(OUT)/core-arm.o
	$(PREFIX)g++ $(ARMFLAGS) $(CXXFLAGS) -fno-threadsafe-statics -c application.cpp -o $(OUT)/application-arm.o
	$(PREFIX)size $(OUT)/application-arm.o $(OUT)/core-arm.o
$(OUT)/c/%.o: %.c
	mkdir -p $(dir $@)
	$(PREFIX)gcc $(ARMFLAGS) -std=c11 -Wall -Wextra -Werror -MMD -MP -DRT950_CUSTOM -I../include -c $< -o $@
$(filter-out $(OUT)/cpp/settings_flash.o,$(CPPOBJECTS)): CXXFLAGS += -Oz
$(OUT)/cpp/ui_home.o $(OUT)/cpp/ui_text.o: CXXFLAGS += -fno-if-conversion -fno-if-conversion2
$(OUT)/cpp/%.o: %.cpp
	mkdir -p $(dir $@)
	$(PREFIX)g++ $(ARMFLAGS) $(CXXFLAGS) -MMD -MP -fno-threadsafe-statics -I../include -c $< -o $@
$(OUT)/rt950-custom-ui.elf: $(COBJECTS) $(CPPOBJECTS) ../diagnostic/linker.ld linker.ld
	$(PREFIX)g++ $(ARMFLAGS) -nostdlib -T linker.ld -Wl,--gc-sections,-Map=$(OUT)/rt950-custom-ui.map $(COBJECTS) $(CPPOBJECTS) -o $@
firmware: $(OUT)/rt950-custom-ui.elf
	$(PREFIX)objcopy -O binary $< $(OUT)/rt950-custom-ui.bin
	python3 pad_image.py $< $(OUT)/rt950-custom-ui.bin $(PREFIX)nm
	python3 ../tools/encrypt_btf.py $(OUT)/rt950-custom-ui.bin $(OUT)/RT950_Custom_UI_v0.42.BTF --key 00000000000000000000000000000000
	$(PREFIX)size $<
clean:
	rm -rf $(OUT)
