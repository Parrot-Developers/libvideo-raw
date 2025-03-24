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

#ifndef ANDROID
#	ifndef _FILE_OFFSET_BITS
#		define _FILE_OFFSET_BITS 64
#	endif /* _FILE_OFFSET_BITS */
#endif /* ANDROID */

#include <errno.h>
#include <stdio.h>

#include <video-raw/vraw.h>

#define ULOG_TAG vraw
#include <ulog.h>

#include <pthread.h>
#define NB_SUPPORTED_FORMATS 32
static struct vdef_raw_format supported_formats[NB_SUPPORTED_FORMATS];
static pthread_once_t supported_formats_is_init = PTHREAD_ONCE_INIT;
static void initialize_supported_formats(void)
{
	supported_formats[0] = vdef_i420;
	supported_formats[1] = vdef_yv12;
	supported_formats[2] = vdef_nv12;
	supported_formats[3] = vdef_nv21;
	supported_formats[4] = vdef_i420_10_16le;
	supported_formats[5] = vdef_yv12_10_16le;
	supported_formats[6] = vdef_nv12_10_16le;
	supported_formats[7] = vdef_nv21_10_16le;
	supported_formats[8] = vdef_i420_10_16be;
	supported_formats[9] = vdef_yv12_10_16be;
	supported_formats[10] = vdef_nv12_10_16be;
	supported_formats[11] = vdef_nv21_10_16be;
	supported_formats[12] = vdef_i420_10_16le_high;
	supported_formats[13] = vdef_yv12_10_16le_high;
	supported_formats[14] = vdef_nv12_10_16le_high;
	supported_formats[15] = vdef_nv21_10_16le_high;
	supported_formats[16] = vdef_i420_10_16be_high;
	supported_formats[17] = vdef_yv12_10_16be_high;
	supported_formats[18] = vdef_nv12_10_16be_high;
	supported_formats[19] = vdef_nv21_10_16be_high;
	supported_formats[20] = vdef_nv21_10_packed;
	supported_formats[21] = vdef_nv21_hisi_tile;
	supported_formats[22] = vdef_nv21_hisi_tile_compressed;
	supported_formats[23] = vdef_nv21_hisi_tile_10_packed;
	supported_formats[24] = vdef_nv21_hisi_tile_compressed_10_packed;
	supported_formats[25] = vdef_gray;
	supported_formats[26] = vdef_gray16;
	supported_formats[27] = vdef_raw8;
	supported_formats[28] = vdef_raw16;
	supported_formats[29] = vdef_raw16_be;
	supported_formats[30] = vdef_raw32;
	supported_formats[31] = vdef_raw32_be;
}


struct vraw_writer {
	struct vraw_writer_config cfg;
	char *filename;
	FILE *file;
	size_t primary_line_width;
};


static int y4m_header_write(struct vraw_writer *self)
{
	const char *fmt = "";

	ULOG_ERRNO_RETURN_ERR_IF(
		!vdef_raw_format_cmp(&self->cfg.format, &vdef_i420) &&
			!vdef_raw_format_cmp(&self->cfg.format,
					     &vdef_i420_10_16le),
		EINVAL);

	if (vdef_raw_format_cmp(&self->cfg.format, &vdef_i420))
		fmt = " C420";
	else if (vdef_raw_format_cmp(&self->cfg.format, &vdef_i420_10_16le))
		fmt = " C420p10";

	fprintf(self->file,
		"YUV4MPEG2 W%d H%d F%d:%d Ip A%d:%d%s\n",
		self->cfg.info.resolution.width,
		self->cfg.info.resolution.height,
		self->cfg.info.framerate.num,
		self->cfg.info.framerate.den,
		self->cfg.info.sar.width,
		self->cfg.info.sar.height,
		fmt);

	return 0;
}


int vraw_writer_new(const char *filename,
		    const struct vraw_writer_config *config,
		    struct vraw_writer **ret_obj)
{
	int res = 0;
	struct vraw_writer *self = NULL;

	(void)pthread_once(&supported_formats_is_init,
			   initialize_supported_formats);

	ULOG_ERRNO_RETURN_ERR_IF(filename == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(config == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(
		!vdef_raw_format_intersect(&config->format,
					   supported_formats,
					   NB_SUPPORTED_FORMATS),
		EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(config->info.resolution.width == 0, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(config->info.resolution.height == 0, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(
		vdef_raw_format_cmp(&config->format, &vdef_nv21_10_packed) &&
			(config->info.resolution.width & 3),
		EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == NULL, EINVAL);

	self = calloc(1, sizeof(*self));
	if (self == NULL)
		return -ENOMEM;

	self->cfg = *config;

	/* Enforce the configuration */
	if (vdef_frac_is_null(&self->cfg.info.framerate)) {
		self->cfg.info.framerate.num = 30;
		self->cfg.info.framerate.den = 1;
	}
	if (vdef_dim_is_null(&self->cfg.info.sar)) {
		self->cfg.info.sar.width = 1;
		self->cfg.info.sar.height = 1;
	}

	self->primary_line_width = self->cfg.info.resolution.width *
				   self->cfg.format.data_size / 8;

	self->filename = strdup(filename);
	if (self->filename == NULL) {
		res = -ENOMEM;
		goto error;
	}

	self->file = fopen(self->filename, "wb");
	if (self->file == NULL) {
		res = -errno;
		goto error;
	}

	if (self->cfg.y4m) {
		/* Write YUV4MPEG2 file headers */
		res = y4m_header_write(self);
		if (res < 0)
			goto error;
	}

	*ret_obj = self;
	return 0;

error:
	(void)vraw_writer_destroy(self);
	*ret_obj = NULL;
	return res;
}


int vraw_writer_destroy(struct vraw_writer *self)
{
	if (self == NULL)
		return 0;

	if (self->file != NULL)
		fclose(self->file);

	free(self->filename);
	free(self);
	return 0;
}


int vraw_writer_frame_write(struct vraw_writer *self,
			    const struct vraw_frame *frame)
{
	int res = 0;
	size_t strd, res1;
	const uint8_t *ptr;
	unsigned int i;

	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(frame == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(
		!vdef_raw_format_cmp(&frame->frame.format, &self->cfg.format),
		EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF((frame->frame.info.resolution.width != 0) &&
					 (frame->frame.info.resolution.width !=
					  self->cfg.info.resolution.width),
				 EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF((frame->frame.info.resolution.height != 0) &&
					 (frame->frame.info.resolution.height !=
					  self->cfg.info.resolution.height),
				 EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(self->file == NULL, EPROTO);

	if (self->cfg.y4m) {
		/* Write YUV4MPEG2 frame header */
		fprintf(self->file, "FRAME\n");
	}

	/* Write raw data to file */
	switch (self->cfg.format.data_layout) {
	case VDEF_RAW_DATA_LAYOUT_PLANAR_Y_U_V:
		ULOG_ERRNO_RETURN_ERR_IF(frame->cdata[0] == NULL, EINVAL);
		ULOG_ERRNO_RETURN_ERR_IF(frame->cdata[1] == NULL, EINVAL);
		ULOG_ERRNO_RETURN_ERR_IF(frame->cdata[2] == NULL, EINVAL);
		ULOG_ERRNO_RETURN_ERR_IF(frame->frame.plane_stride[0] == 0,
					 EINVAL);
		ULOG_ERRNO_RETURN_ERR_IF(frame->frame.plane_stride[1] == 0,
					 EINVAL);
		ULOG_ERRNO_RETURN_ERR_IF(frame->frame.plane_stride[2] == 0,
					 EINVAL);
		/* Y */
		ptr = frame->cdata[0];
		strd = frame->frame.plane_stride[0];
		for (i = 0; i < self->cfg.info.resolution.height; i++) {
			res1 = fwrite(
				ptr, self->primary_line_width, 1, self->file);
			if (res1 != 1) {
				res = -errno;
				ULOG_ERRNO("fwrite", -res);
				return res;
			}
			ptr += strd;
		}

		/* U/V */
		ptr = frame->cdata[1];
		strd = frame->frame.plane_stride[1];
		for (i = 0; i < self->cfg.info.resolution.height / 2; i++) {
			res1 = fwrite(ptr,
				      self->primary_line_width / 2,
				      1,
				      self->file);
			if (res1 != 1) {
				res = -errno;
				ULOG_ERRNO("fwrite", -res);
				return res;
			}
			ptr += strd;
		}

		/* V/U */
		ptr = frame->cdata[2];
		strd = frame->frame.plane_stride[2];
		for (i = 0; i < self->cfg.info.resolution.height / 2; i++) {
			res1 = fwrite(ptr,
				      self->primary_line_width / 2,
				      1,
				      self->file);
			if (res1 != 1) {
				res = -errno;
				ULOG_ERRNO("fwrite", -res);
				return res;
			}
			ptr += strd;
		}
		break;
	case VDEF_RAW_DATA_LAYOUT_SEMI_PLANAR_Y_UV:
		ULOG_ERRNO_RETURN_ERR_IF(frame->cdata[0] == NULL, EINVAL);
		ULOG_ERRNO_RETURN_ERR_IF(frame->cdata[1] == NULL, EINVAL);
		ULOG_ERRNO_RETURN_ERR_IF(frame->frame.plane_stride[0] == 0,
					 EINVAL);
		ULOG_ERRNO_RETURN_ERR_IF(frame->frame.plane_stride[1] == 0,
					 EINVAL);
		/* Y */
		ptr = frame->cdata[0];
		strd = frame->frame.plane_stride[0];
		for (i = 0; i < self->cfg.info.resolution.height; i++) {
			res1 = fwrite(
				ptr, self->primary_line_width, 1, self->file);
			if (res1 != 1) {
				res = -errno;
				ULOG_ERRNO("fwrite", -res);
				return res;
			}
			ptr += strd;
		}

		/* UV/VU */
		ptr = frame->cdata[1];
		strd = frame->frame.plane_stride[1];
		for (i = 0; i < self->cfg.info.resolution.height / 2; i++) {
			res1 = fwrite(
				ptr, self->primary_line_width, 1, self->file);
			if (res1 != 1) {
				res = -errno;
				ULOG_ERRNO("fwrite", -res);
				return res;
			}
			ptr += strd;
		}
		break;
	case VDEF_RAW_DATA_LAYOUT_PACKED:
		ULOG_ERRNO_RETURN_ERR_IF(frame->cdata[0] == NULL, EINVAL);
		ULOG_ERRNO_RETURN_ERR_IF(frame->frame.plane_stride[0] == 0,
					 EINVAL);
		ptr = frame->cdata[0];
		strd = frame->frame.plane_stride[0];
		for (i = 0; i < self->cfg.info.resolution.height; i++) {
			res1 = fwrite(
				ptr, self->primary_line_width, 1, self->file);
			if (res1 != 1) {
				res = -errno;
				ULOG_ERRNO("fwrite", -res);
				return res;
			}
			ptr += strd;
		}
		break;
	default: {
		res = -ENOSYS;
		ULOG_ERRNO("unsupported format: " VDEF_RAW_FORMAT_TO_STR_FMT,
			   -res,
			   VDEF_RAW_FORMAT_TO_STR_ARG(&self->cfg.format));
		break;
	}
	}

	if (res < 0)
		return res;

	res = fflush(self->file);
	if (res < 0) {
		res = -errno;
		ULOG_ERRNO("fflush", -res);
		return res;
	}

	return res;
}
