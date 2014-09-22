LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)

TARGET_PROVIDES_INIT_RC := true

# add your file here (as target) and place the file in this directory
PRODUCT_COPY_FILES += $(LOCAL_PATH)/mtk-kpd.kl:system/usr/keylayout/mtk-kpd.kl \
                      $(LOCAL_PATH)/mtk-kpd.kcm:system/usr/keychars/mtk-kpd.kcm \
                      $(LOCAL_PATH)/init.rc:root/init.rc \
                      $(LOCAL_PATH)/init.usb.rc:root/init.usb.rc \
                      $(LOCAL_PATH)/init.xlog.rc:root/init.xlog.rc \
                      $(LOCAL_PATH)/vold.fstab:system/etc/vold.fstab \
                      $(LOCAL_PATH)/vold.fstab.nand:system/etc/vold.fstab.nand \
                      $(LOCAL_PATH)/player.cfg:system/etc/player.cfg \
                      $(LOCAL_PATH)/mtk_omx_core.cfg:system/etc/mtk_omx_core.cfg \
                      $(LOCAL_PATH)/media_codecs.xml:system/etc/media_codecs.xml \
                      $(LOCAL_PATH)/advanced_meta_init.rc:root/advanced_meta_init.rc \
                      $(LOCAL_PATH)/meta_init.rc:root/meta_init.rc \
                      $(LOCAL_PATH)/mpeg4_venc_parameter.cfg:system/etc/mpeg4_venc_parameter.cfg \
                      $(LOCAL_PATH)/audio_policy.conf:system/etc/audio_policy.conf \
                      $(LOCAL_PATH)/ACCDET.kl:system/usr/keylayout/ACCDET.kl \
                      $(LOCAL_PATH)/fstab.mt6575:root/fstab.mt6575 \
                      $(LOCAL_PATH)/datacomp:root/sbin/datacomp
                      

_init_project_rc := $(MTK_ROOT_CONFIG_OUT)/init.project.rc
ifneq ($(wildcard $(_init_project_rc)),)
PRODUCT_COPY_FILES += $(_init_project_rc):root/init.project.rc
endif

_meta_init_project_rc := $(MTK_ROOT_CONFIG_OUT)/meta_init.project.rc
ifneq ($(wildcard $(_meta_init_project_rc)),)
PRODUCT_COPY_FILES += $(_meta_init_project_rc):root/meta_init.project.rc
endif

_advanced_meta_init_project_rc := $(MTK_ROOT_CONFIG_OUT)/advanced_meta_init.project.rc
ifneq ($(wildcard $(_advanced_meta_init_project_rc)),)
PRODUCT_COPY_FILES += $(_advanced_meta_init_project_rc):root/advanced_meta_init.project.rc
endif

#jiaxing.hu add
ifeq ($(filter fm50af,$(strip $(CUSTOM_KERNEL_LENS))),)
_jiaxing_immediate_rm := $(shell rm -f $(LOCAL_PATH)/android.hardware.camera.af.xml)
endif

PRODUCT_COPY_FILES += $(strip \
                        $(foreach file,$(call wildcard2, $(LOCAL_PATH)/*.xml), \
                          $(addprefix $(LOCAL_PATH)/$(notdir $(file)):system/etc/permissions/,$(notdir $(file))) \
                         ) \
                       )

ifeq ($(HAVE_AEE_FEATURE),yes)
ifeq ($(PARTIAL_BUILD),true)
PRODUCT_COPY_FILES += $(LOCAL_PATH)/init.aee.customer.rc:root/init.aee.customer.rc
else
PRODUCT_COPY_FILES += $(LOCAL_PATH)/init.aee.mtk.rc:root/init.aee.mtk.rc
endif
endif

ifeq ($(strip $(HAVE_SRSAUDIOEFFECT_FEATURE)),yes)
  PRODUCT_COPY_FILES += $(LOCAL_PATH)/srs_processing.cfg:system/data/srs_processing.cfg
endif

ifeq ($(MTK_SHARED_SDCARD),yes)
  PRODUCT_COPY_FILES += $(LOCAL_PATH)/init.sdcard.rc:root/init.sdcard.rc
endif

include $(CLEAR_VARS)
LOCAL_SRC_FILES := mtk-kpd.kcm
LOCAL_MODULE_TAGS := user
include $(BUILD_KEY_CHAR_MAP)

$(call config-custom-folder,modem:modem)
##### INSTALL MODEM FIRMWARE #####

include $(CLEAR_VARS)
LOCAL_MODULE := modem.img
LOCAL_MODULE_TAGS := user
LOCAL_MODULE_CLASS := ETC
LOCAL_MODULE_PATH := $(TARGET_OUT_ETC)/firmware
LOCAL_SRC_FILES := modem/$(LOCAL_MODULE)
include $(BUILD_PREBUILT)

##################################
##INSTALL MODEM DATABASE##
ifeq ($(strip $(MTK_INCLUDE_MODEM_DB_IN_IMAGE)), yes)
  ifeq ($(filter generic banyan_addon,$(PROJECT)),)
    MD_DATABASE_FILE := $(if $(wildcard mediatek/custom/common/modem/$(strip $(CUSTOM_MODEM))/BPLGUInfoCustomAppSrcP_*), $(wildcard mediatek/custom/common/modem/$(strip $(CUSTOM_MODEM))/BPLGUInfoCustomAppSrcP_*), $(wildcard mediatek/custom/common/modem/$(strip $(CUSTOM_MODEM))/BPLGUInfoCustomApp_*))
    ifneq ($(words $(MD_DATABASE_FILE)),1)
      $(error More than one modem database file: $(MD_DATABASE_FILE)!!)
    endif
    MD_DATABASE_FILENAME := $(notdir $(MD_DATABASE_FILE))
    PRODUCT_COPY_FILES += $(strip $(MD_DATABASE_FILE)):system/etc/mddb/$(strip $(MD_DATABASE_FILENAME))
  endif
endif
##########################
## INSTALL catcher_filter.bin ##

ifeq ($(MTK_MDLOGGER_SUPPORT),yes)
include $(CLEAR_VARS)
LOCAL_MODULE := catcher_filter.bin
LOCAL_MODULE_TAGS := user
LOCAL_MODULE_CLASS := ETC
LOCAL_MODULE_PATH := $(TARGET_OUT_ETC)/firmware
LOCAL_SRC_FILES := modem/catcher_filter.bin
include $(BUILD_PREBUILT)
endif

## Install ext catcher_filter.bin ##

ifeq ($(MTK_DT_SUPPORT),yes)
ifneq ($(EVDO_DT_SUPPORT),yes)
include $(CLEAR_VARS)
LOCAL_MODULE := catcher_filter_ext.bin
LOCAL_MODULE_TAGS := user
LOCAL_MODULE_CLASS := ETC 
LOCAL_MODULE_PATH := $(TARGET_OUT_ETC)/firmware
LOCAL_SRC_FILES := modem/ExtModem/catcher_filter.bin
include $(BUILD_PREBUILT)
endif
endif

#################################
####INSTALL EXT MODEM DATABASE###
# delete by houyan.qin for modem db cusotmization, begin
#ifeq ($(strip $(MTK_INCLUDE_EXT_MODEM_DB_IN_IMAGE)), yes)
#  ifeq ($(filter generic banyan_addon,$(PROJECT)),)
#    MD_DATABASE_FILE := $(if $(wildcard mediatek/custom/common/modem/$(strip $(CUSTOM_MODEM))/ExtModem/BPLGUInfoCustomAppSrcP_*), $(wildcard mediatek/custom/common/modem/$(strip $(CUSTOM_MODEM))/ExtModem/BPLGUInfoCustomAppSrcP_*), $(wildcard mediatek/custom/common/modem/$(strip $(CUSTOM_MODEM))/ExtModem/BPLGUInfoCustomApp_*))
#    ifneq ($(words $(MD_DATABASE_FILE)),1)
#      $(error More than one modem database file: $(MD_DATABASE_FILE)!!)
#    endif
#    MD_DATABASE_FILENAME := $(notdir $(MD_DATABASE_FILE))
#    PRODUCT_COPY_FILES += $(strip $(MD_DATABASE_FILE)):system/etc/extmddb/$(strip $(MD_DATABASE_FILENAME))
#  endif
#endif
# delete by houyan.qin for modem db cusotmization, end
#################################
##### INSTALL DSP FIRMWARE #####

include $(CLEAR_VARS)
LOCAL_MODULE := DSP_ROM
LOCAL_MODULE_TAGS := user
LOCAL_MODULE_CLASS := ETC
LOCAL_MODULE_PATH := $(TARGET_OUT_ETC)/firmware
LOCAL_SRC_FILES := modem/$(LOCAL_MODULE)
include $(BUILD_PREBUILT)

PRODUCT_COPY_FILES += $(LOCAL_PATH)/modem/DSP_BL:DSP_BL
##################################

##### INSTALL mtofn.idc ##########

ifeq ($(CUSTOM_KERNEL_OFN),ofn1090)
PRODUCT_COPY_FILES += $(LOCAL_PATH)/mtofn.idc:system/usr/idc/mtofn.idc
endif

##################################

##### INSTALL local.prop #########
ifeq ($(strip $(MTK_BSP_PACKAGE)),yes)
ifeq ($(TARGET_BUILD_VARIANT),eng)
PRODUCT_COPY_FILES += $(LOCAL_PATH)/local.prop:data/local.prop
endif
endif
##################################
