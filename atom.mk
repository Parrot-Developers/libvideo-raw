
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := libvideo-raw
LOCAL_CATEGORY_PATH := libs
LOCAL_DESCRIPTION := Raw video library
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/include
LOCAL_CFLAGS := -DVRAW_API_EXPORTS -fvisibility=hidden -std=gnu99
LOCAL_SRC_FILES := \
	src/vraw.c \
	src/vraw_image.c \
	src/vraw_psnr.c \
	src/vraw_reader.c \
	src/vraw_writer.c
LOCAL_LIBRARIES := \
	libulog \
	libvideo-defs
LOCAL_CONDITIONAL_LIBRARIES := \
	OPTIONAL:libpng \

include $(BUILD_LIBRARY)


include $(CLEAR_VARS)

LOCAL_MODULE := vraw-rewrite
LOCAL_CATEGORY_PATH := multimedia
LOCAL_DESCRIPTION := Raw video library rewriting program
LOCAL_SRC_FILES := \
	tools/vraw_rewrite.c
LOCAL_LIBRARIES := \
	libvideo-defs \
	libvideo-raw \
	libulog

include $(BUILD_EXECUTABLE)


include $(CLEAR_VARS)

LOCAL_MODULE := vraw-psnr
LOCAL_CATEGORY_PATH := multimedia
LOCAL_DESCRIPTION := Raw video library program computing PSNR between 2 YUV files
LOCAL_SRC_FILES := \
	tools/vraw_psnr.c
LOCAL_LIBRARIES := \
	libvideo-defs \
	libvideo-raw \
	libulog

include $(BUILD_EXECUTABLE)


include $(CLEAR_VARS)

LOCAL_MODULE := vraw-image
LOCAL_CATEGORY_PATH := multimedia
LOCAL_DESCRIPTION := Raw video library image program
LOCAL_SRC_FILES := \
	tools/vraw_image.c
LOCAL_LIBRARIES := \
	libvideo-defs \
	libvideo-raw \
	libulog

include $(BUILD_EXECUTABLE)


include $(CLEAR_VARS)

LOCAL_MODULE := vraw-conv-10bit
LOCAL_CATEGORY_PATH := multimedia
LOCAL_DESCRIPTION := 10bit raw video conversion program
LOCAL_SRC_FILES := \
	tools/vraw_conv_10bit.c
LOCAL_LIBRARIES := \
	libvideo-defs \
	libvideo-raw \
	libulog

include $(BUILD_EXECUTABLE)


ifdef TARGET_TEST

include $(CLEAR_VARS)

LOCAL_MODULE := tst-libvideo-raw
LOCAL_CFLAGS += -DTARGET_TEST -D_GNU_SOURCE
LOCAL_SRC_FILES := \
	tests/vraw_test_reader.c \
	tests/vraw_test_writer.c \
	tests/vraw_test.c
LOCAL_LIBRARIES := \
	libcunit \
	libvideo-defs \
	libvideo-raw

include $(BUILD_EXECUTABLE)

endif
