TOOLCHAIN = ARM

include $(MK_DIR)/find_tools.mk

# Hardcoded libraries/AP_Common/missing/cmath defines in "make" to retain the current behavior
EXTRAFLAGS += -DHAVE_CMATH_ISFINITE -DNEED_CMATH_ISFINITE_STD_NAMESPACE

#EXTRAFLAGS += -DHAVE_ENDIAN_H -DHAVE_BYTESWAP_H
#
# Tool options
#
DEFINES         =   -D_POSIX_TIMERS
DEFINES        +=   -DSKETCH=\"$(SKETCH)\" -DSKETCHNAME="\"$(SKETCH)\"" -DSKETCHBOOK="\"$(SKETCHBOOK)\"" -DAPM_BUILD_DIRECTORY=APM_BUILD_$(SKETCH)
DEFINES        +=   $(EXTRAFLAGS)
DEFINES        +=   -DCONFIG_HAL_BOARD=$(HAL_BOARD) 
WARNFLAGS       =   -Wall -Wextra -Wformat -Wshadow -Wpointer-arith -Wcast-align \
                    -Wlogical-op -Wwrite-strings -Wformat=2 -Wno-unused-parameter -Wno-unknown-pragmas
WARNFLAGSCXX    = \
        -Wno-missing-field-initializers \
        -Wno-reorder \
        -Werror=format-security \
        -Werror=array-bounds \
        -Werror=unused-but-set-variable \
        -Werror=uninitialized \
        -Werror=init-self \
        -Wfatal-errors \
        -Wdouble-promotion \
        -Wundef

DEPFLAGS        =   -MD -MP -MT $@

CXXOPTS         =   -ffunction-sections -fdata-sections -fno-exceptions -fsigned-char -fno-rtti 
COPTS           =   -ffunction-sections -fdata-sections -fsigned-char 

ASOPTS          =   -x assembler-with-cpp 

ARM_CPUFLAGS        = -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16
ARM_CPULDFLAGS      = 
ARM_OPTFLAGS        = -Os 


# disable as this breaks distcc
#ifneq ($(SYSTYPE),Darwin)
#LISTOPTS        =   -adhlns=$(@:.o=.lst)
#endif

CPUFLAGS     = -D_GNU_SOURCE
CPULDFLAGS   = -g
OPTFLAGS     ?= -Os -g3 -fno-math-errno -fsingle-precision-constant

CXXFLAGS        =   -g $(CPUFLAGS) $(DEFINES) $(OPTFLAGS) $(ARM_CPUFLAGS)
CXXFLAGS       +=   -std=gnu++11 $(WARNFLAGS) $(WARNFLAGSCXX) $(DEPFLAGS) $(CXXOPTS) 
CFLAGS          =   -g $(CPUFLAGS) $(DEFINES) -Wa,$(LISTOPTS) $(OPTFLAGS) $(ARM_CPUFLAGS) 
CFLAGS         +=   $(WARNFLAGS) $(DEPFLAGS) $(COPTS)
ASFLAGS         =   -g $(CPUFLAGS) $(DEFINES) -Wa,$(LISTOPTS) $(DEPFLAGS) $(ARM_CPUFLAGS) 
ASFLAGS        +=   $(ASOPTS)
LDFLAGS         =   -g $(CPUFLAGS) $(OPTFLAGS) $(WARNFLAGS) $(ARM_CPUFLAGS) 

LIBS ?= -lm

ifeq ($(VERBOSE),)
v = @
else
v =
endif

# Library object files
LIBOBJS			:=	$(SKETCHLIBOBJS)


################################################################################
# Built products
#

# The ELF file
SKETCHELF		=	$(BUILDROOT)/lib$(SKETCH).a
BUILDELF                =       $(notdir $(SKETCHELF))

# HEX file
SKETCHHEX		=	$(BUILDROOT)/$(SKETCH).hex

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

################################################################################
# Rules
#

# fetch dependency info from a previous build if any of it exists
-include $(ALLDEPS)

# Link the final object
$(SKETCHELF): $(SKETCHOBJS) $(LIBOBJS)
	@echo "Building $(SKETCHELF)"
	$(RULEHDR)
	$(v)$(AR) -r $@ $^
	$(v)cp $(SKETCHELF) .
	@echo "Firmware is in $(BUILDELF)"


SKETCH_INCLUDES	=	$(SKETCHLIBINCLUDES)
SLIB_INCLUDES	=	-I$(dir $<)/utility $(SKETCHLIBINCLUDES)

include $(MK_DIR)/build_rules.mk
