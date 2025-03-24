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
 *   * Neither the name of the copyright holders nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPretS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "vraw_test.h"

#define ASSET_CROWD_RUN "/tmp/crowd_run_144p50_i420.yuv"


static struct {
	enum vdef_resolution resolution;
	const struct vdef_raw_format *format;
	char absolute_path[200];
} s_assets_map[] = {
	{VDEF_RESOLUTION_144P, &vdef_gray, "/tmp/crowd_run_144p50_gray.yuv"},
	{VDEF_RESOLUTION_144P, &vdef_i420, "/tmp/crowd_run_144p50_i420.yuv"},
	{VDEF_RESOLUTION_144P, &vdef_nv12, "/tmp/crowd_run_144p50_nv12.yuv"},
	{VDEF_RESOLUTION_144P, &vdef_nv21, "/tmp/crowd_run_144p50_nv21.yuv"},
	{VDEF_RESOLUTION_192X144,
	 &vdef_i420,
	 "/tmp/crowd_run_192x144@50_i420.yuv"},
};


static const char *get_path(const struct vdef_raw_format *format)
{
	for (size_t i = 0; i < ARRAY_SIZE(s_assets_map); i++) {
		if (!vdef_raw_format_cmp(format, s_assets_map[i].format))
			continue;
		return s_assets_map[i].absolute_path;
	}
	return NULL;
}


static void fill_config(struct vraw_writer_config *config,
			enum vdef_resolution resolution,
			const struct vdef_raw_format *format)
{
	int ret;
	memset(config, 0, sizeof(*config));

	config->y4m = 0;
	/* TODO: test y4m */
	config->format = *format;
	config->info.color_primaries = VDEF_COLOR_PRIMARIES_BT709;
	config->info.dynamic_range = VDEF_DYNAMIC_RANGE_SDR;
	config->info.framerate.num = 30;
	config->info.framerate.den = 1;
	config->info.full_range = false;
	config->info.matrix_coefs = VDEF_MATRIX_COEFS_BT709;
	ret = vdef_resolution_to_dim(resolution, &config->info.resolution);
	CU_ASSERT_EQUAL(ret, 0);
	config->info.sar.height = 1;
	config->info.sar.width = 1;
	config->info.tone_mapping = VDEF_TONE_MAPPING_STANDARD;
	config->info.transfer_function = VDEF_TRANSFER_FUNCTION_BT709;
}


static void fill_frame(struct vraw_frame *frame,
		       enum vdef_resolution resolution,
		       const struct vdef_raw_format *format)
{
	int ret;
	memset(frame, 0, sizeof(*frame));

	frame->frame.format = *format;
	frame->frame.info.bit_depth = 8;
	frame->frame.info.color_primaries = VDEF_COLOR_PRIMARIES_BT709;
	frame->frame.info.dynamic_range = VDEF_DYNAMIC_RANGE_SDR;
	frame->frame.info.full_range = false;
	frame->frame.info.matrix_coefs = VDEF_MATRIX_COEFS_BT709;
	ret = vdef_resolution_to_dim(resolution, &frame->frame.info.resolution);
	CU_ASSERT_EQUAL(ret, 0);
	frame->frame.info.sar.width = 1;
	frame->frame.info.sar.height = 1;
	frame->frame.info.timescale = 1000000;
	frame->frame.info.tone_mapping = VDEF_TONE_MAPPING_STANDARD;
	frame->frame.info.transfer_function = VDEF_TRANSFER_FUNCTION_BT709;

	ret = vdef_calc_raw_frame_size(format,
				       &frame->frame.info.resolution,
				       frame->frame.plane_stride,
				       NULL,
				       NULL,
				       NULL,
				       NULL,
				       NULL);
	CU_ASSERT_EQUAL(ret, 0);
}


static void test_vraw_writer_new(void)
{
	for (size_t i = 0; i < ARRAY_SIZE(s_assets_map); i++) {
		int ret = 0;
		struct vraw_writer *writer = NULL;
		struct vraw_writer_config empty_config = {0};
		struct vraw_writer_config invalid_config = {0};
		struct vraw_writer_config config = {0};
		enum vdef_resolution resolution = s_assets_map[i].resolution;
		const struct vdef_raw_format *format = s_assets_map[i].format;

		const char *path = get_path(format);
		fill_config(&config, resolution, format);

		/* Ensure the file does not exist */
		unlink(path);

		/* Bad args */
		ret = vraw_writer_new(NULL, NULL, NULL);
		CU_ASSERT_EQUAL(ret, -EINVAL);

		ret = vraw_writer_new(path, NULL, NULL);
		CU_ASSERT_EQUAL(ret, -EINVAL);

		ret = vraw_writer_new(NULL, &config, NULL);
		CU_ASSERT_EQUAL(ret, -EINVAL);

		ret = vraw_writer_new(path, NULL, &writer);
		CU_ASSERT_EQUAL(ret, -EINVAL);

		ret = vraw_writer_new(path, &config, NULL);
		CU_ASSERT_EQUAL(ret, -EINVAL);

		ret = vraw_writer_new(path, NULL, &writer);
		CU_ASSERT_EQUAL(ret, -EINVAL);

		ret = vraw_writer_new(path, NULL, &writer);
		CU_ASSERT_EQUAL(ret, -EINVAL);

		/* empty config */
		ret = vraw_writer_new(path, &empty_config, &writer);
		CU_ASSERT_EQUAL(ret, -EINVAL);

		/* invalid config: resolution */
		fill_config(&invalid_config, resolution, format);
		invalid_config.info.resolution.width = 0;

		ret = vraw_writer_new(path, &invalid_config, &writer);
		CU_ASSERT_EQUAL(ret, -EINVAL);

		/* invalid config: resolution */
		fill_config(&invalid_config, resolution, format);
		invalid_config.info.resolution.height = 0;

		ret = vraw_writer_new(path, &invalid_config, &writer);
		CU_ASSERT_EQUAL(ret, -EINVAL);

		/* invalid config: format */
		fill_config(&invalid_config, resolution, format);
		invalid_config.format = vdef_abgr;

		ret = vraw_writer_new(path, &invalid_config, &writer);
		CU_ASSERT_EQUAL(ret, -EINVAL);

		/* valid */
		ret = vraw_writer_new(path, &config, &writer);
		CU_ASSERT_EQUAL(ret, 0);

		(void)vraw_writer_destroy(writer);
	}
}


static void test_vraw_writer_frame_write(void)
{
	for (size_t i = 0; i < ARRAY_SIZE(s_assets_map); i++) {
		int ret = 0;
		struct vraw_writer *writer = NULL;
		struct vraw_writer_config config = {0};
		struct vraw_frame frame = {0};
		uint8_t *frame_data = NULL;
		uint8_t *frame_data_in_file = NULL;
		size_t frame_size;
		enum vdef_resolution resolution = s_assets_map[i].resolution;
		const struct vdef_raw_format *format = s_assets_map[i].format;
		size_t plane_size[VDEF_RAW_MAX_PLANE_COUNT] = {0};

		const char *path = get_path(format);
		fill_config(&config, resolution, format);

		vdef_calc_raw_frame_size(format,
					 &config.info.resolution,
					 NULL,
					 NULL,
					 NULL,
					 NULL,
					 plane_size,
					 NULL);
		frame_size = 0;
		for (unsigned int p = 0; p < VDEF_RAW_MAX_PLANE_COUNT; ++p)
			frame_size += plane_size[p];

		frame_data = calloc(1, frame_size);
		frame_data_in_file = calloc(1, frame_size);

		(void)vraw_writer_new(path, &config, &writer);

		ret = vraw_writer_frame_write(NULL, NULL);
		CU_ASSERT_EQUAL(ret, -EINVAL);

		ret = vraw_writer_frame_write(writer, NULL);
		CU_ASSERT_EQUAL(ret, -EINVAL);

		ret = vraw_writer_frame_write(NULL, &frame);
		CU_ASSERT_EQUAL(ret, -EINVAL);

		ret = vraw_writer_frame_write(writer, &frame);
		CU_ASSERT_EQUAL(ret, -EINVAL);

		/* config OK, no data */
		fill_frame(&frame, resolution, format);
		ret = vraw_writer_frame_write(writer, &frame);
		CU_ASSERT_EQUAL(ret, -EINVAL);

		/* missing U, V planes (when not in GRAY format) */
		if (!vdef_raw_format_cmp(format, &vdef_gray)) {
			frame.cdata[0] = frame_data;
			frame.cdata[1] = NULL;
			frame.cdata[2] = NULL;
			ret = vraw_writer_frame_write(writer, &frame);
			CU_ASSERT_EQUAL(ret, -EINVAL);
		}

		/* missing V planes (I420 only) */
		if (vdef_raw_format_cmp(format, &vdef_i420)) {
			frame.cdata[0] = frame_data;
			frame.cdata[1] = frame_data;
			frame.cdata[2] = NULL;
			ret = vraw_writer_frame_write(writer, &frame);
			CU_ASSERT_EQUAL(ret, -EINVAL);
		}

		/* missing strides */
		frame.cdata[0] = frame_data;
		frame.cdata[1] = frame_data;
		frame.cdata[2] = frame_data;
		frame.frame.plane_stride[0] = 0;
		frame.frame.plane_stride[1] = 0;
		frame.frame.plane_stride[2] = 0;
		ret = vraw_writer_frame_write(writer, &frame);
		CU_ASSERT_EQUAL(ret, -EINVAL);

		fill_frame(&frame, resolution, format);
		frame.cdata[0] = frame_data;
		frame.cdata[1] = frame_data;
		frame.cdata[2] = frame_data;

		unsigned int i = 0;
		for (i = 0; i < 5; i++) {
			FILE *file = NULL;
			size_t file_len;
			size_t read;

			memset(frame_data, i, frame_size);

			ret = vraw_writer_frame_write(writer, &frame);
			CU_ASSERT_EQUAL(ret, 0);

			file = fopen(path, "rb");
			CU_ASSERT_PTR_NOT_NULL(file);

			(void)fseek(file, 0, SEEK_END);

			/* Check that write size is correct */
			file_len = ftello(file);
			CU_ASSERT_EQUAL(file_len, (i + 1) * frame_size);

			(void)fseek(file, (i)*frame_size, SEEK_SET);

			/* Check that last frame was properly written */
			read = fread(frame_data_in_file, frame_size, 1, file);
			CU_ASSERT_EQUAL(read, 1);

			ret = memcmp(
				frame_data, frame_data_in_file, frame_size);

			CU_ASSERT_EQUAL(ret, 0);

			(void)fclose(file);
		}

		(void)vraw_writer_destroy(writer);

		free(frame_data);
		free(frame_data_in_file);
	}
}


CU_TestInfo g_vraw_test_writer[] = {
	{FN("vraw-writer-new"), &test_vraw_writer_new},
	{FN("vraw-writer-frame-write"), &test_vraw_writer_frame_write},

	CU_TEST_INFO_NULL,
};
