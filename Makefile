ifeq ($(ERL_EI_INCLUDE_DIR),)
$(error ERL_EI_INCLUDE_DIR not set. Invoke via mix)
endif

# Set Erlang-specific compile and linker flags
ERL_CFLAGS ?= -I$(ERL_EI_INCLUDE_DIR)
ERL_LDFLAGS ?= -L$(ERL_EI_LIBDIR)

NIF_LDFLAGS += -fPIC -shared
NIF_CFLAGS ?= -fPIC -O2 -Wall

ifeq ($(CROSSCOMPILE),)
ifeq ($(shell uname),Darwin)
NIF_LDFLAGS += -undefined dynamic_lookup
endif
endif

NIF=priv/build_calendar.so
FIRMWARE_NIF=priv/firmware_nif.so
ARDUINO_FW=priv/arduino-firmware.hex
FARMDUINO_FW=priv/farmduino-firmware.hex
FARMDUINO_V14_FW=priv/farmduino_v14-firmware.hex
BLINK_FW=priv/blink.hex
CLEAR_EEPROM_FW=priv/clear_eeprom.hex

ARDUINO_INSTALL_DIR ?= $(HOME)/arduino-1.8.5
ARDUINO_BUILDER=$(ARDUINO_INSTALL_DIR)/arduino-builder

ARDUINO_HARDWARE_DIR = $(ARDUINO_INSTALL_DIR)/hardware
ARDUINO_HARDWARE_FLAGS = -hardware $(ARDUINO_HARDWARE_DIR)

ARDUINO_TOOLS_FLAGS = -tools $(ARDUINO_INSTALL_DIR)/tools-builder \
-tools $(ARDUINO_HARDWARE_DIR)/tools/avr

ARDUINO_LIBS_FLAGS = -built-in-libraries $(ARDUINO_INSTALL_DIR)/libraries

ARDUINO_PREFS_FLAGS = -prefs=build.warn_data_percentage=75 \
	-prefs=runtime.tools.avrdude.path=$(ARDUINO_INSTALL_DIR)/hardware/tools/avr \
	-prefs=runtime.tools.avr-gcc.path=$(ARDUINO_INSTALL_DIR)/hardware/tools/avr

ARDUINO_ARCH_FLAGS = -fqbn=arduino:avr:mega:cpu=atmega2560
ARDUINO_SRC_INO = c_src/farmbot-arduino-firmware/src/src.ino
ARDUINO_SRC_BLINK_INO = $(ARDUINO_INSTALL_DIR)/examples/01.Basics/Blink/Blink.ino
ARDUINO_SRC_CLEAR_EEPROM_INO = $(ARDUINO_HARDWARE_DIR)/arduino/avr/libraries/EEPROM/examples/eeprom_clear/eeprom_clear.ino

ARDUINO_BUILD_DIR = $(PWD)/_build/arduino
ARDUINO_CACHE_DIR = $(PWD)/_build/arduino-cache
ARDUINO_BUILD_DIR_FLAGS =	-build-path $(ARDUINO_BUILD_DIR) -build-cache	$(ARDUINO_CACHE_DIR)

ARDUINO_BUILD_COMMON = $(ARDUINO_BUILDER) \
	$(ARDUINO_HARDWARE_FLAGS) \
	$(ARDUINO_TOOLS_FLAGS) \
	$(ARDUINO_LIBS_FLAGS) \
	$(ARDUINO_ARCH_FLAGS) \
	$(ARDUINO_PREFS_FLAGS) \
	$(ARDUINO_BUILD_DIR_FLAGS)

ARDUINO_BUILD = $(ARDUINO_BUILD_COMMON) $(ARDUINO_SRC_INO)
BLINK_BUILD = $(ARDUINO_BUILD_COMMON) $(ARDUINO_SRC_BLINK_INO)
CLEAR_EEPROM_BUILD = $(ARDUINO_BUILD_COMMON) $(ARDUINO_SRC_CLEAR_EEPROM_INO)

all: priv $(NIF) $(FIRMWARE_NIF) farmbot_arduino_firmware

farmbot_arduino_firmware_build_dirs: $(ARDUINO_BUILD_DIR) $(ARDUINO_CACHE_DIR)

$(ARDUINO_BUILD_DIR):
	mkdir -p $(ARDUINO_BUILD_DIR)

$(ARDUINO_CACHE_DIR):
	mkdir -p $(ARDUINO_CACHE_DIR)

farmbot_arduino_firmware: arduino farmduino farmduino_v14 blink clear_eeprom

arduino: farmbot_arduino_firmware_build_dirs $(ARDUINO_FW)

farmduino: farmbot_arduino_firmware_build_dirs $(FARMDUINO_FW)

farmduino_v14: farmbot_arduino_firmware_build_dirs $(FARMDUINO_V14_FW)

blink: farmbot_arduino_firmware_build_dirs $(BLINK_FW)

clear_eeprom: farmbot_arduino_firmware_build_dirs $(CLEAR_EEPROM_FW)

priv:
	mkdir -p priv

$(NIF): c_src/build_calendar.c
	$(CC) $(ERL_CFLAGS) $(NIF_CFLAGS) $(ERL_LDFLAGS) $(NIF_LDFLAGS) -o $@ $<

$(FIRMWARE_NIF): c_src/firmware_nif.c
	$(CC) $(ERL_CFLAGS) $(NIF_CFLAGS) $(ERL_LDFLAGS) $(NIF_LDFLAGS) -o $@ $<

$(ARDUINO_FW):
	$(shell echo \#define RAMPS_V14 > c_src/farmbot-arduino-firmware/src/Board.h)
	rm -rf $(ARDUINO_BUILD_DIR)/*
	rm -rf $(ARDUINO_CACHE_DIR)/*
	$(ARDUINO_BUILD)
	cp $(ARDUINO_BUILD_DIR)/src.ino.hex $@

$(FARMDUINO_FW):
	$(shell echo \#define FARMDUINO_V10 > c_src/farmbot-arduino-firmware/src/Board.h)
	rm -rf $(ARDUINO_BUILD_DIR)/*
	rm -rf $(ARDUINO_CACHE_DIR)/*
	$(ARDUINO_BUILD)
	cp $(ARDUINO_BUILD_DIR)/src.ino.hex $@

$(FARMDUINO_V14_FW):
	$(shell echo \#define FARMDUINO_V14 > c_src/farmbot-arduino-firmware/src/Board.h)
	rm -rf $(ARDUINO_BUILD_DIR)/*
	rm -rf $(ARDUINO_CACHE_DIR)/*
	$(ARDUINO_BUILD)
	cp $(ARDUINO_BUILD_DIR)/src.ino.hex $@

$(BLINK_FW):
	$(BLINK_BUILD)
	cp $(ARDUINO_BUILD_DIR)/Blink.ino.hex $@

$(CLEAR_EEPROM_FW):
	$(CLEAR_EEPROM_BUILD)
	cp $(ARDUINO_BUILD_DIR)/eeprom_clear.ino.hex $@

clean:
	$(RM) $(NIF)
	rm -rf $(ARDUINO_BUILD_DIR) $(ARDUINO_CACHE_DIR)
	rm -rf priv/*.hex
