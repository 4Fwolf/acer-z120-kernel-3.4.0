# For use with external or internal boots.

# RAM Version
TEXT_BASE = 0x01E00000
#TEXT_BASE = 0xffff0000
# System RAM Version
#TEXT_BASE = 0x40004000
PLATFORM_CPPFLAGS  += -DMACH_ARCH_ID=MACH_TYPE_$(MTK_PLATFORM)

