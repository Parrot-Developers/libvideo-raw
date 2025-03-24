/**
 * Copyright (c) 2018 Parrot Drones SAS
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the Parrot Drones SAS Company nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE PARROT DRONES SAS COMPANY BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <errno.h>
#include <stdio.h>

#include <video-defs/vdefs.h>
#include <video-raw/vraw.h>

#define ULOG_TAG vraw
#include <ulog.h>

#if BUILD_LIBPNG

#	include <png.h>

int vraw_image_read(const char *filename,
		    uint8_t *data,
		    size_t *len,
		    struct vraw_frame *frame)
{
	png_image image;
	int ret = 0;

	memset(&image, 0, sizeof(image));
	image.version = PNG_IMAGE_VERSION;
	if (png_image_begin_read_from_file(&image, filename) < 0 ||
	    image.height == 0 || image.width == 0) {
		ret = -EINVAL;
		ULOG_ERRNO("png_image_begin_read_from_file", -ret);
		goto end;
	}
	image.format = PNG_FORMAT_RGBA;
	*len = PNG_IMAGE_SIZE(image);
	if (data == NULL || frame == NULL)
		goto end;
	if (png_image_finish_read(&image, NULL, data, 0, NULL) < 0) {
		ret = -EINVAL;
		ULOG_ERRNO("png_image_finish_read", -ret);
		goto end;
	}
	memset(frame, 0, sizeof(*frame));
	frame->frame.format = vdef_abgr;
	frame->data[0] = data;
	frame->frame.plane_stride[0] = image.width * 4;
	frame->frame.info.resolution.height = image.height;
	frame->frame.info.resolution.width = image.width;
	frame->frame.info.sar.height = 1;
	frame->frame.info.sar.width = 1;
	frame->frame.format.data_size = image.colormap_entries * 4 / 8;
end:
	png_image_free(&image);
	return ret;
}


#else /* BUILD_LIBPNG */

int vraw_image_read(const char *file_name,
		    uint8_t *data,
		    size_t *len,
		    struct vraw_frame *frame)
{
	return -ENOSYS;
}

#endif /* BUILD_LIBPNG */
