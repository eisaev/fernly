include mkenv.mk
include magic.mk

# isogashii:
BUILD = build
CROSS_COMPILE=arm-none-eabi-

CFLAGS = -march=armv5te -mfloat-abi=soft -Wall \
	 -Os -Iinclude -marm -fno-stack-protector
AFLAGS = 

LDFLAGS = --nostdlib -T fernvale.ld
LIBS = lib/libgcc-armv5.a

STAGE1_LDFLAGS = --nostdlib -T stage1.ld

SRC_C = \
	bionic.c \
	cmd-hex.c \
	cmd-irq.c \
	cmd-peekpoke.c \
	cmd-reboot.c \
	cmd-sleep.c \
	cmd-spi.c \
	cmd-led.c \
	cmd-load.c \
	cmd-bl.c \
	cmd-lcd.c \
	cmd-keypad.c \
	emi.c \
	irq.c \
	lcd.c \
	main.c \
	scriptic_core.c \
	scriptic_helper.c \
	serial.c \
	spi.c \
	utils.c \
	vectors.c \
	vsprintf.c

SRC_S = \
	scriptic/set-plls.S \
	scriptic/enable-psram.S \
	scriptic/spi.S \
	scriptic/spi-blockmode.S \
	scriptic/keypad.S \
	start.S

OBJ = $(addprefix $(BUILD)/, $(SRC_S:.S=.o) $(SRC_C:.c=.o))

STAGE1_SRC_C = \
	stage1.c \
	serial.c \
	vectors.c

STAGE1_SRC_S = \
	start.S

STAGE1_OBJ = $(addprefix $(BUILD)/, $(STAGE1_SRC_S:.S=.o) $(STAGE1_SRC_C:.c=.o))


FRSER_SRC_C = \
	frser.c \
	bionic.c \
	scriptic_core.c \
	utils.c \
	vectors.c

FRSER_SRC_S = \
	scriptic/spi.S \
	scriptic/spi-blockmode.S \
	scriptic/set-plls.S \
	rom_usb_uart.S \
	start.S

FRSER_OBJ = $(addprefix $(BUILD)/, $(FRSER_SRC_S:.S=.o) $(FRSER_SRC_C:.c=.o))

all: $(BUILD)/firmware.bin \
	$(BUILD)/stage1.bin \
	$(BUILD)/dump-rom-usb.bin \
	$(BUILD)/usb-loader.bin \
	$(BUILD)/mt6261-test.bin \
	$(BUILD)/fernly-usb-loader \
	$(BUILD)/echo.bin \
	$(BUILD)/frser.bin
clean:
	$(RM) -rf $(BUILD)

$(BUILD)/fernly-usb-loader: fernly-usb-loader.c sha1.c sha1.h
	$(CC_NATIVE) fernly-usb-loader.c sha1.c -o $@

$(BUILD)/%.bin: $(BUILD)/%.o
	$(OBJCOPY) -S -O binary $< $@

HEADER_BUILD = $(BUILD)/genhdr
$(BUILD)/firmware.bin: $(BUILD)/firmware.elf
	$(OBJCOPY) -S -O binary $(BUILD)/firmware.elf $@

$(BUILD)/firmware.elf: $(OBJ)
	$(LD) $(LDFLAGS) --entry=reset_handler -o $@ $(OBJ) $(LIBS)

$(BUILD)/stage1.bin: $(BUILD)/stage1.elf
	$(OBJCOPY) -S -O binary $(BUILD)/stage1.elf $@

$(BUILD)/stage1.elf: $(STAGE1_OBJ)
	$(LD) $(STAGE1_LDFLAGS) --entry=reset_handler -o $@ $(STAGE1_OBJ) $(LIBS)

$(BUILD)/frser.bin: $(BUILD)/frser.elf
	$(OBJCOPY) -S -O binary $(BUILD)/frser.elf $@

$(BUILD)/frser.elf: $(FRSER_OBJ)
	$(LD) $(STAGE1_LDFLAGS) --entry=reset_handler -o $@ $(FRSER_OBJ) $(LIBS)

$(OBJ): $(HEADER_BUILD)/generated.h | $(OBJ_DIRS)
$(HEADER_BUILD)/generated.h: | $(HEADER_BUILD)
	  touch $@

OBJ_DIRS = $(sort $(dir $(OBJ))) scriptic
$(OBJ_DIRS):
	$(MKDIR) -p $@ $@/scriptic
$(HEADER_BUILD):
	$(MKDIR) -p $@ build/scriptic
-include $(OBJ:.o=.P)

test: all
	novena-usb-hub -d u1 ; sleep 1; novena-usb-hub -e u1 ; sleep 2
	$(BUILD)/fernly-usb-loader /dev/fernvale $(BUILD)/usb-loader.bin $(BUILD)/firmware.bin

shell: all
	novena-usb-hub -d u1 ; sleep 1; novena-usb-hub -e u1 ; sleep 2
	$(BUILD)/fernly-usb-loader -s /dev/fernvale $(BUILD)/usb-loader.bin $(BUILD)/firmware.bin
