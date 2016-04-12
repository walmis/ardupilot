# board_skyfalcon.mk
#
# Build ArduPlane for Flymaple http://www.open-drone.org/flymaple


TOOLCHAIN = ARM

include $(MK_DIR)/find_tools.mk

FAMILY := cortex-m4
F_CPU := 72000000L
UPLOADER := dfu-util
USBID := 1EAF:0003
PRODUCT_ID := 0003
LD_MEM_DIR := sram_64k_flash_512k

#
# Tool options
#
DEFINES        =   $(EXTRAFLAGS)
DEFINES        +=   -DCONFIG_HAL_BOARD=$(HAL_BOARD) -DEKF_BUILD=1
WARNFLAGS       =   -Wformat -Wall -Wshadow -Wpointer-arith -Wcast-align -Wno-psabi
WARNFLAGS      +=   -Wwrite-strings -Wformat=2 -Wno-unused-parameter
WARNFLAGSCXX    =   -Wno-reorder
DEPFLAGS        =   -MD -MT $@

CXXOPTS         =   -std=gnu++11 -ffunction-sections -fdata-sections -fno-exceptions -fsigned-char 
COPTS           =   -ffunction-sections -fdata-sections -fsigned-char

ASOPTS          =   -x assembler-with-cpp 
LISTOPTS        =   -adhlns=$(@:.o=.lst)

NATIVE_CPUFLAGS     = -D_GNU_SOURCE
NATIVE_CPULDFLAGS   = -g
NATIVE_OPTFLAGS     = -O0 -g

ARM_CPUFLAGS        = -mcpu=$(FAMILY) -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16
ARM_CPULDFLAGS      = 
ARM_OPTFLAGS        = -Os 

CPUFLAGS= $($(TOOLCHAIN)_CPUFLAGS)
CPULDFLAGS= $($(TOOLCHAIN)_CPULDFLAGS)
OPTFLAGS= $($(TOOLCHAIN)_OPTFLAGS)

CXXFLAGS        =   $(CPUFLAGS) $(DEFINES) -Os -g3 -gdwarf-2 -nostdlib \
                   -ffunction-sections -fdata-sections \
                   -fno-rtti -fno-exceptions -Wl,--gc-sections $(OPTFLAGS)
CXXFLAGS       +=   $(WARNFLAGS) $(WARNFLAGSCXX) $(DEPFLAGS) $(CXXOPTS)
CFLAGS          =   $(CPUFLAGS) $(DEFINES) -Os -g3 -gdwarf-2 -nostdlib \
                   -ffunction-sections -fdata-sections \
                   -fno-exceptions -Wl,--gc-sections $(OPTFLAGS)
CFLAGS         +=   $(WARNFLAGS) $(DEPFLAGS) $(COPTS)
ASFLAGS         =   $(CPUFLAGS) $(DEFINES) -x assembler-with-cpp 
ASFLAGS        +=   $(ASOPTS)

LDFLAGS         =   $(CPUFLAGS) $(OPTFLAGS) $(WARNFLAGS) -mcpu=cortex-m3 -mthumb \
           -Xlinker --gc-sections \
           -Xassembler --march=armv7-m -Wall 
LDFLAGS        +=   -Wl,--gc-sections -Wl,-Map -Wl,$(SKETCHMAP) $(CPULDFLAGS)

LIBS = -lm

ifeq ($(VERBOSE),)
v = @
else
v =
endif

COREOBJS = 
COREINCLUDES = 

# Library object files
LIBOBJS			:= $(SKETCHLIBOBJS)	

################################################################################
# Built products
#

# The ELF file
SKETCHELF		=	$(BUILDROOT)/lib$(SKETCH).a
BUILDELF                =       $(notdir $(SKETCHELF))

# HEX file
SKETCHHEX		=	$(BUILDROOT)/$(SKETCH).hex

# BIN file
SKETCHBIN		=	$(BUILDROOT)/$(SKETCH).bin

# EEP file
SKETCHEEP		=	$(BUILDROOT)/$(SKETCH).eep

# Map file
SKETCHMAP		=	$(BUILDROOT)/$(SKETCH).map

# All of the objects that may be built
ALLOBJS			=	$(SKETCHOBJS) $(LIBOBJS)

# All of the dependency files that may be generated
ALLDEPS			=	$(ALLOBJS:%.o=%.d)

################################################################################
# Targets
#

all: $(SKETCHELF)

print-%:
	echo "$*=$($*)"

flymaple-upload: upload

.PHONY: upload
upload: $(SKETCHBIN)
	$(LIBMAPLE_PATH)/support/scripts/reset.py && sleep 1 &&  $(UPLOADER) -a1 -d $(USBID) -D $(SKETCHBIN) -R

debug:
	$(AVARICE) --mkII --capture --jtag usb :4242 & \
	gnome-terminal -x $(GDB) $(SKETCHELF) & \
	echo -e '\n\nat the gdb prompt type "target remote localhost:4242"'

# this allows you to flash your image via JTAG for when you
# have completely broken your USB
jtag-program:
	$(AVARICE) --mkII --jtag usb --erase --program --file $(SKETCHELF)

################################################################################
# Rules
#

# fetch dependency info from a previous build if any of it exists
-include $(ALLDEPS)

# Link the final object
$(SKETCHELF):	$(SKETCHOBJS) $(LIBOBJS)
	$(RULEHDR)
	$(v)$(AR) -r $@ $^
	$(v)cp $(SKETCHELF) .
	@echo "ArduCopter Static library built: $(BUILDELF)"


SKETCH_INCLUDES        =       $(SKETCHLIBINCLUDES) $(ARDUINOLIBINCLUDES) $(COREINCLUDES)
SLIB_INCLUDES  =       -I$(dir $<)/utility $(SKETCHLIBINCLUDES) $(ARDUINOLIBINCLUDES) $(COREINCLUDES)

include $(MK_DIR)/build_rules.mk
