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
#include <math.h>
#include <stdbool.h>
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


struct vraw_reader {
	struct vraw_reader_config cfg;
	char *filename;
	FILE *file;
	int reverse;
	size_t header_offset;
	size_t frame_header_size;
	size_t plane_stride[VDEF_RAW_MAX_PLANE_COUNT];
	size_t plane_size[VDEF_RAW_MAX_PLANE_COUNT];
	size_t frame_size;
	size_t file_frame_size;
	size_t file_size;
	size_t file_frame_count;
	uint64_t timestamp;
	unsigned int index;
	unsigned int count;
};


static int y4m_header_read(struct vraw_reader *self)
{
	int res;
	char str[100], *r, *p, *p2, *tmp;
	off_t off;

	r = fgets(str, sizeof(str), self->file);
	if (r == NULL) {
		res = -errno;
		ULOG_ERRNO("fgets", -res);
		return res;
	}

	off = ftello(self->file);
	if (off < 0) {
		res = -errno;
		ULOG_ERRNO("ftello", -res);
		return res;
	}
	self->header_offset = off;

	self->frame_header_size = strlen("FRAME\n");

	p = strtok_r(r, " ", &tmp);

	if ((p == NULL) || (strcmp(p, "YUV4MPEG2"))) {
		res = -EPROTO;
		ULOG_ERRNO("invalid YUV4MPEG2 file format", -res);
		return res;
	}

	self->cfg.format = vdef_i420;

	while (p) {
		if (strlen(p) < 2) {
			p = strtok_r(NULL, " ", &tmp);
			continue;
		}

		switch (p[0]) {
		case 'W':
			self->cfg.info.resolution.width = atoi(p + 1);
			break;
		case 'H':
			self->cfg.info.resolution.height = atoi(p + 1);
			break;
		case 'F':
			p2 = strchr(p, ':');
			if (p2) {
				self->cfg.info.framerate.num = atoi(p + 1);
				self->cfg.info.framerate.den = atoi(p2 + 1);
			}
			break;
		case 'A':
			p2 = strchr(p, ':');
			if (p2) {
				self->cfg.info.sar.width = atoi(p + 1);
				self->cfg.info.sar.height = atoi(p2 + 1);
			}
			break;
		case 'C':
			if (strcmp(p + 1, "420") == 0)
				self->cfg.format = vdef_i420;
			else if (strcmp(p + 1, "420p10") == 0)
				self->cfg.format = vdef_i420_10_16le;
		default:
			break;
		}

		p = strtok_r(NULL, " ", &tmp);
	}

	return 0;
}


static int y4m_frame_header_read(struct vraw_reader *self)
{
	int res;
	char str[10], *r;

	r = fgets(str, sizeof(str), self->file);
	if (r == NULL) {
		res = -errno;
		if (!feof(self->file))
			ULOG_ERRNO("fgets", -res);
		return res;
	}
	if (strcmp(str, "FRAME\n")) {
		res = -EPROTO;
		ULOG_ERRNO("failed to read y4m frame header", -res);
		return -res;
	}

	return 0;
}


static int seek_to_previous_frame(struct vraw_reader *self)
{
	int res;
	off_t cur_offset;
	off_t seek_to = (off_t)-2 * self->file_frame_size;

	/* Note: the current position is the beginning of the next frame;
	 * thus to seek to the beginning of the previous frame, the offset
	 * is -2 times the frame size */

	cur_offset = ftello(self->file);
	if (cur_offset < 0) {
		res = -errno;
		ULOG_ERRNO("ftello", -res);
		return res;
	}

	if (cur_offset + seek_to < 0)
		return -EOVERFLOW;

	res = fseeko(self->file, seek_to, SEEK_CUR);
	if (res < 0) {
		res = -errno;
		ULOG_ERRNO("fseeko", -res);
		return res;
	}

	if (self->cfg.y4m) {
		seek_to = -2 * self->frame_header_size;

		cur_offset = ftello(self->file);
		if (cur_offset < 0) {
			res = -errno;
			ULOG_ERRNO("ftello", -res);
			return res;
		}

		if (cur_offset + seek_to < 0)
			return -EOVERFLOW;

		res = fseeko(self->file, seek_to, SEEK_CUR);
		if (res < 0) {
			res = -errno;
			ULOG_ERRNO("fseeko", -res);
			return res;
		}
	}

	self->index -= 2;

	return 0;
}


int vraw_reader_new(const char *filename,
		    const struct vraw_reader_config *config,
		    struct vraw_reader **ret_obj)
{
	int res = 0;
	struct vraw_reader *self = NULL;
	unsigned int plane_count;
	bool align_constrained = false;
	float file_frame_count;
	off_t off = 0;

	(void)pthread_once(&supported_formats_is_init,
			   initialize_supported_formats);

	ULOG_ERRNO_RETURN_ERR_IF(filename == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(config == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(config->start_reversed && config->loop != -1,
				 EINVAL);
	if (!config->y4m) {
		/* Format, bit depth, width and height must be provided */
		ULOG_ERRNO_RETURN_ERR_IF(config->info.resolution.width == 0,
					 EINVAL);
		ULOG_ERRNO_RETURN_ERR_IF(config->info.resolution.height == 0,
					 EINVAL);
		ULOG_ERRNO_RETURN_ERR_IF(
			!vdef_raw_format_intersect(&config->format,
						   supported_formats,
						   NB_SUPPORTED_FORMATS),
			EINVAL);
	}

	self = calloc(1, sizeof(*self));
	if (self == NULL)
		return -ENOMEM;

	self->cfg = *config;

	self->filename = strdup(filename);
	if (self->filename == NULL) {
		res = -ENOMEM;
		goto error;
	}

	self->file = fopen(self->filename, "rb");
	if (self->file == NULL) {
		res = -errno;
		ULOG_ERRNO("fopen('%s')", -res, self->filename);
		goto error;
	}

	/* Seek to the end of file */
	off = fseeko(self->file, 0L, SEEK_END);
	if (off < 0) {
		res = -errno;
		ULOG_ERRNO("fseeko", -res);
		goto error;
	}

	off = ftello(self->file);
	if (off < 0) {
		res = -errno;
		ULOG_ERRNO("ftello", -res);
		goto error;
	}
	self->file_size = off;

	/* Seek back to the beginning of file */
	off = fseeko(self->file, 0L, SEEK_SET);
	if (off < 0) {
		res = -errno;
		ULOG_ERRNO("fseeko", -res);
		goto error;
	}

	if (self->cfg.y4m) {
		res = y4m_header_read(self);
		if (res < 0)
			goto error;
	}

	/* Enforce the configuration */
	if (vdef_frac_is_null(&self->cfg.info.framerate)) {
		self->cfg.info.framerate.num = 30;
		self->cfg.info.framerate.den = 1;
	}
	if (vdef_dim_is_null(&self->cfg.info.sar)) {
		self->cfg.info.sar.width = 1;
		self->cfg.info.sar.height = 1;
	}
	if (vdef_dim_is_null(&self->cfg.info.resolution)) {
		res = -EPROTO;
		ULOG_ERRNO("invalid video dimensions %dx%d",
			   -res,
			   self->cfg.info.resolution.width,
			   self->cfg.info.resolution.height);
		goto error;
	}

	plane_count = vdef_get_raw_frame_plane_count(&self->cfg.format);

	for (unsigned int p = 0; p < plane_count; ++p) {
		align_constrained = (self->cfg.plane_stride_align[p] ||
				     self->cfg.plane_scanline_align[p] ||
				     self->cfg.plane_size_align[p]);
		if (align_constrained)
			break;
	}

	memset(self->plane_stride,
	       0,
	       plane_count * sizeof(*self->plane_stride));
	memset(self->plane_size, 0, plane_count * sizeof(*self->plane_size));
	if (!align_constrained) {
		memset(self->cfg.plane_stride_align,
		       0,
		       plane_count * sizeof(*self->cfg.plane_stride_align));
		memset(self->cfg.plane_scanline_align,
		       0,
		       plane_count * sizeof(*self->cfg.plane_scanline_align));
		memset(self->cfg.plane_size_align,
		       0,
		       plane_count * sizeof(*self->cfg.plane_size_align));
	}

	/* Get non-aligned plane_stride and plane_size */
	vdef_calc_raw_frame_size(&self->cfg.format,
				 &self->cfg.info.resolution,
				 self->plane_stride,
				 NULL,
				 NULL,
				 NULL,
				 self->plane_size,
				 NULL);

	self->file_frame_size = 0;
	for (unsigned int p = 0; p < plane_count; ++p)
		self->file_frame_size += self->plane_size[p];

	file_frame_count = (self->file_size - self->header_offset) /
			   (self->file_frame_size + self->frame_header_size);
	if (rint(file_frame_count) != file_frame_count) {
		res = -EINVAL;
		ULOGE("invalid file size: %zu", self->file_size);
		goto error;
	}
	self->file_frame_count = (size_t)file_frame_count;

	/* Get aligned plane_stride and plane_size */
	vdef_calc_raw_frame_size(&self->cfg.format,
				 &self->cfg.info.resolution,
				 self->plane_stride,
				 self->cfg.plane_stride_align,
				 NULL,
				 self->cfg.plane_scanline_align,
				 self->plane_size,
				 self->cfg.plane_size_align);

	self->frame_size = 0;
	for (unsigned int p = 0; p < plane_count; ++p)
		self->frame_size += self->plane_size[p];

	if (self->cfg.start_index > 0) {
		if (self->cfg.start_reversed)
			self->reverse = 1;
		res = fseeko(self->file,
			     (off_t)self->cfg.start_index *
				     (self->file_frame_size +
				      self->frame_header_size),
			     SEEK_CUR);
		if (res < 0) {
			res = -errno;
			ULOG_ERRNO("fseeko", -res);
			goto error;
		}
		self->index = self->cfg.start_index;
	}

	*ret_obj = self;

	return 0;

error:
	(void)vraw_reader_destroy(self);
	*ret_obj = NULL;
	return res;
}


int vraw_reader_destroy(struct vraw_reader *self)
{
	if (self == NULL)
		return 0;

	if (self->file != NULL)
		fclose(self->file);

	free(self->filename);
	free(self);
	return 0;
}


int vraw_reader_get_config(struct vraw_reader *self,
			   struct vraw_reader_config *config)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(config == NULL, EINVAL);

	*config = self->cfg;

	return 0;
}


ssize_t vraw_reader_get_min_buf_size(struct vraw_reader *self)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);

	return self->frame_size;
}


ssize_t vraw_reader_get_file_frame_count(struct vraw_reader *self)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);

	return self->file_frame_count;
}


int vraw_reader_set_framerate(struct vraw_reader *self,
			      const struct vdef_frac *framerate)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(framerate == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(vdef_frac_is_null(framerate), EINVAL);

	self->cfg.info.framerate = *framerate;

	return 0;
}


static int vraw_reader_frame_read_planes(struct vraw_reader *self,
					 uint8_t *data)
{
	int res = 0;
	unsigned char *current_addr = data;
	unsigned int height = self->cfg.info.resolution.height;
	unsigned int row_bytes = self->cfg.info.resolution.width *
				 self->cfg.format.data_size / 8;

	errno = EINVAL;
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(data == NULL, EINVAL);

	/* Read Y */
	for (unsigned int h = 0; h < height; ++h) {
		res = fread(current_addr, row_bytes, 1, self->file);
		if (res == 1) {
			current_addr += self->plane_stride[0];
			continue;
		} else if (res < 0) {
			res = -errno;
			ULOG_ERRNO("fread Y plane", -res);
		}
		goto out;
	}
	current_addr = data + self->plane_size[0];

	if (self->cfg.format.data_layout == VDEF_RAW_DATA_LAYOUT_SEMI_PLANAR) {
		/* Read UV */
		for (unsigned int h = 0; h < height / 2; ++h) {
			res = fread(current_addr, row_bytes, 1, self->file);
			if (res == 1) {
				current_addr += self->plane_stride[1];
				continue;
			} else if (res < 0) {
				res = -errno;
				ULOG_ERRNO("fread UV plane", -res);
			}
			goto out;
		}
	} else if (self->cfg.format.data_layout ==
		   VDEF_RAW_DATA_LAYOUT_PLANAR) {
		/* Read U */
		for (unsigned int h = 0; h < height / 2; ++h) {
			res = fread(current_addr, row_bytes / 2, 1, self->file);
			if (res == 1) {
				current_addr += self->plane_stride[1];
				continue;
			} else if (res < 0) {
				res = -errno;
				ULOG_ERRNO("fread U plane", -res);
			}
			goto out;
		}

		/* Read V */
		current_addr = data + self->plane_size[0] + self->plane_size[1];
		for (unsigned int h = 0; h < height / 2; ++h) {
			res = fread(current_addr, row_bytes / 2, 1, self->file);
			if (res == 1) {
				current_addr += self->plane_stride[2];
				continue;
			} else if (res < 0) {
				res = -errno;
				ULOG_ERRNO("fread V plane", -res);
			}
			goto out;
		}
	} else if (self->cfg.format.data_layout ==
		   VDEF_RAW_DATA_LAYOUT_PACKED) {
		/* Do nothing */
	} else {
		errno = ENOSYS;
		return -ENOSYS;
	}

	self->index++;

out:
	return res;
}


int vraw_reader_frame_read(struct vraw_reader *self,
			   uint8_t *data,
			   size_t len,
			   struct vraw_frame *frame)
{
	int res;
	size_t res1;
	unsigned int plane_count;

	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(data == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(len == 0, ENOBUFS);
	ULOG_ERRNO_RETURN_ERR_IF(frame == NULL, EINVAL);

	ULOG_ERRNO_RETURN_ERR_IF(len < self->frame_size, ENOBUFS);
	ULOG_ERRNO_RETURN_ERR_IF(self->file == NULL, EPROTO);

	if (self->cfg.y4m) {
		/* Read the frame header */
		res = y4m_frame_header_read(self);
		if (feof(self->file) || (self->cfg.max_count > 0 &&
					 self->index > self->cfg.max_count)) {
			if (self->cfg.loop > 0) {
				self->index = 0;
				res = fseeko(self->file,
					     self->header_offset,
					     SEEK_SET);
				if (res < 0) {
					res = -errno;
					ULOG_ERRNO("fseeko", -res);
					return res;
				}

				res = y4m_frame_header_read(self);
				if (res < 0)
					return res;
			} else if (self->cfg.loop < 0) {
				self->reverse = 1;
				res = seek_to_previous_frame(self);
				if (res < 0)
					return res;

				res = y4m_frame_header_read(self);
				if (res < 0)
					return res;
			} else {
				return -ENOENT;
			}
		} else if (res != -1) {
			res = -errno;
			ULOG_ERRNO("y4m_frame_header_read", -res);
			return res;
		}
	}

	/* Read the YUV data */
	res1 = vraw_reader_frame_read_planes(self, data);
	if (feof(self->file) ||
	    (self->cfg.max_count > 0 && (self->index > self->cfg.max_count))) {
		if (self->cfg.loop > 0) {
			self->index = 0;
			res = fseeko(self->file, self->header_offset, SEEK_SET);
			if (res < 0) {
				res = -errno;
				ULOG_ERRNO("fseeko", -res);
				return res;
			}

			res = vraw_reader_frame_read(self, data, len, frame);
			if (res < 0)
				ULOG_ERRNO("vraw_reader_frame_read", -res);
			return res;
		} else if (self->cfg.loop < 0) {
			self->reverse = 1;
			res = seek_to_previous_frame(self);
			if (res < 0)
				return res;

			res = vraw_reader_frame_read(self, data, len, frame);
			if (res < 0)
				ULOG_ERRNO("vraw_reader_frame_read", -res);
			return res;
		} else {
			return -ENOENT;
		}
	} else if (res1 != 1) {
		res = -errno;
		ULOG_ERRNO("vraw_reader_frame_read_planes", -res);
		return res;
	}


	/* Fill the frame info */
	plane_count = vdef_get_raw_frame_plane_count(&self->cfg.format);
	for (unsigned int p = 0; p < VDEF_RAW_MAX_PLANE_COUNT; p++) {
		frame->data[p] = data;
		data = (p + 1) < plane_count ? data + self->plane_size[p]
					     : NULL;
	}
	memcpy(frame->frame.plane_stride,
	       self->plane_stride,
	       sizeof(self->plane_stride));
	frame->frame.format = self->cfg.format;
	vdef_format_to_frame_info(&self->cfg.info, &frame->frame.info);
	frame->frame.info.timestamp = self->timestamp;
	frame->frame.info.timescale = 1000000;
	frame->frame.info.index = self->count;

	self->timestamp += 1000000ULL * self->cfg.info.framerate.den /
			   self->cfg.info.framerate.num;

	self->count++;

	if (self->reverse) {
		res = seek_to_previous_frame(self);
		if (res < 0) {
			if (res != -EOVERFLOW)
				return res;
			self->reverse = 0;
		}
	}

	return 0;
}
