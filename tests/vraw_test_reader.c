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


#define CROWD_RUN_FRAME_COUNT 500


static struct {
	enum vdef_resolution resolution;
	const struct vdef_raw_format *format;
	char relative_path[100];
	char absolute_path[200];
	bool test_loop;
	size_t frame_count;
} s_assets_map[] = {
	{VDEF_RESOLUTION_144P,
	 &vdef_gray,
	 "Raw/SVT/crowd_run_144p50_gray.yuv",
	 "",
	 false,
	 CROWD_RUN_FRAME_COUNT},
	{VDEF_RESOLUTION_144P,
	 &vdef_i420,
	 "Raw/SVT/crowd_run_144p50_i420.yuv",
	 "",
	 true,
	 CROWD_RUN_FRAME_COUNT},
	{VDEF_RESOLUTION_144P,
	 &vdef_nv12,
	 "Raw/SVT/crowd_run_144p50_nv12.yuv",
	 "",
	 false,
	 CROWD_RUN_FRAME_COUNT},
	{VDEF_RESOLUTION_144P,
	 &vdef_nv21,
	 "Raw/SVT/crowd_run_144p50_nv21.yuv",
	 "",
	 false,
	 CROWD_RUN_FRAME_COUNT},
	{VDEF_RESOLUTION_192X144,
	 &vdef_i420,
	 "Raw/SVT/crowd_run_192x144@50_i420.yuv",
	 "",
	 false,
	 CROWD_RUN_FRAME_COUNT},
};


static const char *get_path(size_t index)
{
	if (index >= ARRAY_SIZE(s_assets_map))
		return NULL;

	if (s_assets_map[index].absolute_path[0] == '\0') {
		snprintf(s_assets_map[index].absolute_path,
			 sizeof(s_assets_map[index].absolute_path),
			 "%s/%s",
			 (getenv("ASSETS_ROOT") != NULL) ? getenv("ASSETS_ROOT")
							 : ASSETS_ROOT,
			 s_assets_map[index].relative_path);
	}
	int file_read_access = access(s_assets_map[index].absolute_path, R_OK);
	CU_ASSERT_FATAL(file_read_access == 0);
	return s_assets_map[index].absolute_path;
}


static void fill_config(struct vraw_reader_config *config,
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


static void test_vraw_reader_new(void)
{
	for (size_t i = 0; i < ARRAY_SIZE(s_assets_map); i++) {
		int ret = 0;
		struct vraw_reader *reader = NULL;
		struct vraw_reader_config empty_config = {0};
		struct vraw_reader_config invalid_config = {0};
		struct vraw_reader_config config = {0};
		enum vdef_resolution resolution = s_assets_map[i].resolution;
		const struct vdef_raw_format *format = s_assets_map[i].format;

		const char *path = get_path(i);
		fill_config(&config, resolution, format);

		/* Bad args */
		ret = vraw_reader_new(NULL, NULL, NULL);
		CU_ASSERT_EQUAL(ret, -EINVAL);

		ret = vraw_reader_new(path, NULL, NULL);
		CU_ASSERT_EQUAL(ret, -EINVAL);

		ret = vraw_reader_new(NULL, &config, NULL);
		CU_ASSERT_EQUAL(ret, -EINVAL);

		ret = vraw_reader_new(path, NULL, &reader);
		CU_ASSERT_EQUAL(ret, -EINVAL);

		ret = vraw_reader_new(path, &config, NULL);
		CU_ASSERT_EQUAL(ret, -EINVAL);

		ret = vraw_reader_new(path, NULL, &reader);
		CU_ASSERT_EQUAL(ret, -EINVAL);

		ret = vraw_reader_new(path, NULL, &reader);
		CU_ASSERT_EQUAL(ret, -EINVAL);

		/* invalid file path */
		/* FIXME: should be KO */
		/*ret = vraw_reader_new(".", &config, &reader);
		CU_ASSERT_EQUAL(ret, -ENOENT);*/

		ret = vraw_reader_new("invalid_path.yuv", &config, &reader);
		CU_ASSERT_EQUAL(ret, -ENOENT);

		/* empty config */
		ret = vraw_reader_new(path, &empty_config, &reader);
		CU_ASSERT_EQUAL(ret, -EINVAL);

		/* invalid config: resolution */
		fill_config(&invalid_config, resolution, format);
		invalid_config.info.resolution.width = 0;

		ret = vraw_reader_new(path, &invalid_config, &reader);
		CU_ASSERT_EQUAL(ret, -EINVAL);

		/* invalid config: resolution */
		fill_config(&invalid_config, resolution, format);
		invalid_config.info.resolution.height = 0;

		ret = vraw_reader_new(path, &invalid_config, &reader);
		CU_ASSERT_EQUAL(ret, -EINVAL);

		/* invalid config: format */
		fill_config(&invalid_config, resolution, format);
		invalid_config.format = vdef_abgr;

		ret = vraw_reader_new(path, &invalid_config, &reader);
		CU_ASSERT_EQUAL(ret, -EINVAL);

		/* valid */
		ret = vraw_reader_new(path, &config, &reader);
		CU_ASSERT_EQUAL(ret, 0);

		(void)vraw_reader_destroy(reader);
	}
}


static void test_vraw_reader_get_config(void)
{
	for (size_t i = 0; i < ARRAY_SIZE(s_assets_map); i++) {
		int ret = 0;
		struct vraw_reader *reader = NULL;
		struct vraw_reader_config config = {0};
		struct vraw_reader_config recv_config = {0};
		enum vdef_resolution resolution = s_assets_map[i].resolution;
		const struct vdef_raw_format *format = s_assets_map[i].format;

		const char *path = get_path(i);
		fill_config(&config, resolution, format);

		(void)vraw_reader_new(path, &config, &reader);

		/* Bad args */
		ret = vraw_reader_get_config(NULL, NULL);
		CU_ASSERT_EQUAL(ret, -EINVAL);

		ret = vraw_reader_get_config(reader, NULL);
		CU_ASSERT_EQUAL(ret, -EINVAL);

		ret = vraw_reader_get_config(NULL, &recv_config);
		CU_ASSERT_EQUAL(ret, -EINVAL);

		ret = vraw_reader_get_config(reader, &recv_config);
		CU_ASSERT_EQUAL(ret, 0);
		CU_ASSERT_EQUAL(config.loop, recv_config.loop);
		CU_ASSERT_EQUAL(config.max_count, recv_config.max_count);
		CU_ASSERT_EQUAL(config.y4m, recv_config.y4m);
		CU_ASSERT_EQUAL(config.start_index, recv_config.start_index);
		CU_ASSERT_TRUE(vdef_raw_format_cmp(&config.format,
						   &recv_config.format));
		CU_ASSERT_TRUE(vdef_dim_cmp(&config.info.resolution,
					    &recv_config.info.resolution));
		CU_ASSERT_TRUE(
			vdef_dim_cmp(&config.info.sar, &recv_config.info.sar));

		(void)vraw_reader_destroy(reader);
	}
}


static void test_vraw_reader_get_min_buf_size(void)
{
	for (size_t i = 0; i < ARRAY_SIZE(s_assets_map); i++) {
		ssize_t size, expected_size;
		struct vraw_reader *reader = NULL;
		struct vraw_reader_config config = {0};
		enum vdef_resolution resolution = s_assets_map[i].resolution;
		const struct vdef_raw_format *format = s_assets_map[i].format;
		size_t plane_size[VDEF_RAW_MAX_PLANE_COUNT] = {0};

		const char *path = get_path(i);
		fill_config(&config, resolution, format);

		(void)vraw_reader_new(path, &config, &reader);

		/* Bad args */
		size = vraw_reader_get_min_buf_size(NULL);
		CU_ASSERT_EQUAL(size, -EINVAL);

		vdef_calc_raw_frame_size(format,
					 &config.info.resolution,
					 NULL,
					 NULL,
					 NULL,
					 NULL,
					 plane_size,
					 NULL);
		expected_size = 0;
		for (unsigned int p = 0; p < VDEF_RAW_MAX_PLANE_COUNT; ++p)
			expected_size += plane_size[p];

		size = vraw_reader_get_min_buf_size(reader);
		CU_ASSERT_EQUAL(size, expected_size);

		(void)vraw_reader_destroy(reader);
	}
}


static void test_vraw_reader_get_file_frame_count(void)
{
	for (size_t i = 0; i < ARRAY_SIZE(s_assets_map); i++) {
		ssize_t count, expected_count = s_assets_map[i].frame_count;
		struct vraw_reader *reader = NULL;
		struct vraw_reader_config config = {0};
		enum vdef_resolution resolution = s_assets_map[i].resolution;
		const struct vdef_raw_format *format = s_assets_map[i].format;

		const char *path = get_path(i);
		fill_config(&config, resolution, format);

		(void)vraw_reader_new(path, &config, &reader);

		/* Bad args */
		count = vraw_reader_get_file_frame_count(NULL);
		CU_ASSERT_EQUAL(count, -EINVAL);

		count = vraw_reader_get_file_frame_count(reader);
		CU_ASSERT_EQUAL(count, expected_count);

		(void)vraw_reader_destroy(reader);
	}
}


static void test_vraw_reader_set_framerate(void)
{
	int ret;
	struct vraw_reader *reader = NULL;
	struct vraw_reader_config config = {0};
	enum vdef_resolution resolution = s_assets_map[0].resolution;
	const struct vdef_raw_format *format = s_assets_map[0].format;
	uint8_t *data = NULL;
	ssize_t size = 0;
	struct vdef_frac framerate = {};
	uint64_t expected_timestamp = 0;

	const char *path = get_path(0);
	fill_config(&config, resolution, format);

	(void)vraw_reader_new(path, &config, &reader);

	size = vraw_reader_get_min_buf_size(reader);
	data = calloc(1, size);

	/* Bad args */
	ret = vraw_reader_set_framerate(NULL, NULL);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	ret = vraw_reader_set_framerate(reader, NULL);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	ret = vraw_reader_set_framerate(NULL, &framerate);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* Framerate is null */
	ret = vraw_reader_set_framerate(reader, &framerate);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	enum vdef_framerate framerate_list[] = {
		VDEF_FRAMERATE_24,
		VDEF_FRAMERATE_25,
		VDEF_FRAMERATE_30,
		VDEF_FRAMERATE_60_7,
	};

	for (size_t i = 0; i < ARRAY_SIZE(framerate_list); i++) {
		ret = vdef_framerate_to_frac(framerate_list[i], &framerate);
		CU_ASSERT_EQUAL(ret, 0);

		ret = vraw_reader_set_framerate(reader, &framerate);
		CU_ASSERT_EQUAL(ret, 0);

		unsigned int i = 0;
		for (i = 0; i < 5; i++) {
			struct vraw_frame frame = {0};
			memset(&frame, 0, sizeof(frame));
			ret = vraw_reader_frame_read(
				reader, data, size, &frame);
			CU_ASSERT_EQUAL(ret, 0);
			CU_ASSERT_EQUAL(frame.frame.info.timestamp,
					expected_timestamp);
			expected_timestamp +=
				(1000000ULL * framerate.den / framerate.num);
		}
	}

	(void)vraw_reader_destroy(reader);

	free(data);
}


static void test_vraw_reader_frame_read(void)
{
	for (size_t i = 0; i < ARRAY_SIZE(s_assets_map); i++) {
		int ret = 0;
		uint8_t *data = NULL;
		ssize_t size = 0;
		struct vraw_reader *reader = NULL;
		struct vraw_reader_config config = {0};
		struct vraw_frame frame = {0};
		enum vdef_resolution resolution = s_assets_map[i].resolution;
		const struct vdef_raw_format *format = s_assets_map[i].format;

		const char *path = get_path(i);
		fill_config(&config, resolution, format);

		(void)vraw_reader_new(path, &config, &reader);

		size = vraw_reader_get_min_buf_size(reader);
		data = calloc(1, size);

		/* Bad args */
		ret = vraw_reader_frame_read(NULL, NULL, 0, NULL);
		CU_ASSERT_EQUAL(ret, -EINVAL);

		ret = vraw_reader_frame_read(reader, NULL, 0, NULL);
		CU_ASSERT_EQUAL(ret, -EINVAL);

		ret = vraw_reader_frame_read(reader, data, 0, NULL);
		CU_ASSERT_EQUAL(ret, -ENOBUFS);

		ret = vraw_reader_frame_read(reader, data, size, NULL);
		CU_ASSERT_EQUAL(ret, -EINVAL);

		ret = vraw_reader_frame_read(reader, data, 0, &frame);
		CU_ASSERT_EQUAL(ret, -ENOBUFS);

		unsigned int i = 0;
		for (i = 0; i < 5; i++) {
			memset(&frame, 0, sizeof(frame));
			uint64_t expected_timestamp =
				(1000000ULL * config.info.framerate.den /
				 config.info.framerate.num) *
				i;

			ret = vraw_reader_frame_read(
				reader, data, size, &frame);
			CU_ASSERT_EQUAL(ret, 0);

			/* Check frame info */
			CU_ASSERT_EQUAL(frame.frame.info.index, i);
			CU_ASSERT_EQUAL(frame.frame.info.resolution.width,
					config.info.resolution.width);
			CU_ASSERT_EQUAL(frame.frame.info.resolution.height,
					config.info.resolution.height);
			CU_ASSERT_TRUE(vdef_raw_format_cmp(&frame.frame.format,
							   &config.format));
			CU_ASSERT_TRUE(
				vdef_dim_cmp(&frame.frame.info.resolution,
					     &config.info.resolution));
			CU_ASSERT_TRUE(vdef_dim_cmp(&frame.frame.info.sar,
						    &config.info.sar));
			CU_ASSERT_EQUAL(frame.frame.info.bit_depth,
					config.info.bit_depth);
			CU_ASSERT_EQUAL(frame.frame.info.color_primaries,
					config.info.color_primaries);
			CU_ASSERT_EQUAL(frame.frame.info.dynamic_range,
					config.info.dynamic_range);
			CU_ASSERT_EQUAL(frame.frame.info.tone_mapping,
					config.info.tone_mapping);
			CU_ASSERT_EQUAL(frame.frame.info.transfer_function,
					config.info.transfer_function);
			CU_ASSERT_EQUAL(frame.frame.info.matrix_coefs,
					config.info.matrix_coefs);

			CU_ASSERT_EQUAL(frame.frame.info.timescale, 1000000);
			CU_ASSERT_EQUAL(frame.frame.info.timestamp,
					expected_timestamp);
			CU_ASSERT_EQUAL(frame.frame.info.capture_timestamp, 0);
		}

		(void)vraw_reader_destroy(reader);

		free(data);
	}
}


static void test_vraw_reader_api(void)
{
	for (size_t i = 0; i < ARRAY_SIZE(s_assets_map); i++) {
		int ret = 0;
		uint8_t *data = NULL;
		ssize_t size = 0;
		struct vraw_reader *reader = NULL;
		struct vraw_reader_config config = {0};
		enum vdef_resolution resolution = s_assets_map[i].resolution;
		const struct vdef_raw_format *format = s_assets_map[i].format;

		const char *path = get_path(i);
		fill_config(&config, resolution, format);

		/* valid */
		ret = vraw_reader_new(path, &config, &reader);
		CU_ASSERT_EQUAL(ret, 0);

		size = vraw_reader_get_min_buf_size(reader);
		data = calloc(1, size);

		unsigned int i = 0;
		for (i = 0; i < 5; i++) {
			struct vraw_frame frame = {0};
			uint64_t expected_timestamp =
				(1000000ULL * config.info.framerate.den /
				 config.info.framerate.num) *
				i;

			ret = vraw_reader_frame_read(
				reader, data, size, &frame);
			CU_ASSERT_EQUAL(ret, 0);

			/* Check timestamps info */
			CU_ASSERT_EQUAL(frame.frame.info.timescale, 1000000);
			CU_ASSERT_EQUAL(frame.frame.info.timestamp,
					expected_timestamp);
			CU_ASSERT_EQUAL(frame.frame.info.capture_timestamp, 0);
		}

		ret = vraw_reader_destroy(reader);
		CU_ASSERT_EQUAL(ret, 0);

		free(data);
	}
}


static void test_vraw_reader_max_count(void)
{
	int COUNT_LIST[] = {1, 2, 3, 4, 5};
	for (size_t i = 0; i < ARRAY_SIZE(s_assets_map); i++) {
		for (size_t j = 0; j < ARRAY_SIZE(COUNT_LIST); j++) {
			int ret = 0;
			uint8_t *data = NULL;
			ssize_t size = 0;
			struct vraw_reader *reader = NULL;
			struct vraw_frame frame = {0};
			struct vraw_reader_config config = {0};
			enum vdef_resolution resolution =
				s_assets_map[i].resolution;
			const struct vdef_raw_format *format =
				s_assets_map[i].format;
			unsigned int max_count = COUNT_LIST[j];

			const char *path = get_path(i);
			fill_config(&config, resolution, format);
			config.max_count = max_count;

			/* valid */
			ret = vraw_reader_new(path, &config, &reader);
			CU_ASSERT_EQUAL(ret, 0);

			size = vraw_reader_get_min_buf_size(reader);
			data = calloc(1, size);

			for (size_t k = 0; k < max_count; k++) {
				ret = vraw_reader_frame_read(
					reader, data, size, &frame);
				CU_ASSERT_EQUAL(ret, 0);
			}

			ret = vraw_reader_frame_read(
				reader, data, size, &frame);
			CU_ASSERT_EQUAL(ret, -ENOENT);

			ret = vraw_reader_destroy(reader);
			CU_ASSERT_EQUAL(ret, 0);

			free(data);
		}
	}
}


static void test_vraw_reader_loop(void)
{
	for (size_t i = 0; i < ARRAY_SIZE(s_assets_map); i++) {
		int ret = 0;
		unsigned int width;
		uint8_t *data = NULL;
		ssize_t size = 0;
		uint8_t **first_rows = NULL;
		struct vraw_reader *reader = NULL;
		struct vraw_reader_config config = {0};
		struct vraw_frame frame = {0};
		enum vdef_resolution resolution = s_assets_map[i].resolution;
		const struct vdef_raw_format *format = s_assets_map[i].format;

		/* Note: as this test is heavy, it is not performed for all
		 * files */
		if (!s_assets_map[i].test_loop)
			continue;

		first_rows = calloc(500, sizeof(*first_rows));

		const char *path = get_path(i);

		/**
		 * Don't loop
		 */
		fill_config(&config, resolution, format);
		config.loop = 0;

		(void)vraw_reader_new(path, &config, &reader);

		size = vraw_reader_get_min_buf_size(reader);
		width = config.info.resolution.width;

		data = calloc(1, size);

		unsigned int i = 0;
		for (i = 0; i < 500; i++) {
			memset(&frame, 0, sizeof(frame));
			uint64_t expected_timestamp =
				(1000000ULL * config.info.framerate.den /
				 config.info.framerate.num) *
				i;

			ret = vraw_reader_frame_read(
				reader, data, size, &frame);
			CU_ASSERT_EQUAL(ret, 0);

			first_rows[i] = calloc(width, sizeof(*first_rows[0]));
			memcpy(first_rows[i], data, width);

			/* Check frame info */
			CU_ASSERT_EQUAL(frame.frame.info.index, i);
			CU_ASSERT_EQUAL(frame.frame.info.timestamp,
					expected_timestamp);
			CU_ASSERT_EQUAL(frame.frame.info.capture_timestamp, 0);
		}

		/* EOF reached */
		ret = vraw_reader_frame_read(reader, data, size, &frame);
		CU_ASSERT_EQUAL(ret, -ENOENT);

		(void)vraw_reader_destroy(reader);

		free(data);

		/**
		 * Loop forwards (loop = 1)
		 */
		fill_config(&config, resolution, format);
		config.loop = 1;

		(void)vraw_reader_new(path, &config, &reader);

		size = vraw_reader_get_min_buf_size(reader);
		data = calloc(1, size);

		for (i = 0; i < 1500; i++) {
			memset(&frame, 0, sizeof(frame));
			uint64_t expected_timestamp =
				(1000000ULL * config.info.framerate.den /
				 config.info.framerate.num) *
				i;

			ret = vraw_reader_frame_read(
				reader, data, size, &frame);
			CU_ASSERT_EQUAL(ret, 0);

			/* Check frame info */
			CU_ASSERT_EQUAL(frame.frame.info.index, i);
			CU_ASSERT_EQUAL(frame.frame.info.timestamp,
					expected_timestamp);
			CU_ASSERT_EQUAL(frame.frame.info.capture_timestamp, 0);

			if (i >= 500) {
				/* Check that [i % 500] frame equals i */
				ret = memcmp(data, first_rows[i % 500], width);
				CU_ASSERT_EQUAL(ret, 0);
			}
		}

		(void)vraw_reader_destroy(reader);

		free(data);

		/**
		 * Loop backwards (loop = -1)
		 */
		fill_config(&config, resolution, format);
		config.loop = -1;

		(void)vraw_reader_new(path, &config, &reader);

		size = vraw_reader_get_min_buf_size(reader);
		data = calloc(1, size);

		int index_in_file = 0;
		unsigned int reverse = 0;
		for (i = 0; i < 1500; i++) {
			memset(&frame, 0, sizeof(frame));
			uint64_t expected_timestamp =
				(1000000ULL * config.info.framerate.den /
				 config.info.framerate.num) *
				i;

			ret = vraw_reader_frame_read(
				reader, data, size, &frame);
			CU_ASSERT_EQUAL(ret, 0);

			/* Check frame info */
			CU_ASSERT_EQUAL(frame.frame.info.index, i);
			CU_ASSERT_EQUAL(frame.frame.info.timestamp,
					expected_timestamp);
			CU_ASSERT_EQUAL(frame.frame.info.capture_timestamp, 0);

			if (i >= 500 && i <= (500 + 499)) {
				/* Check that current frame matches with index
				 */
				ret = memcmp(
					data, first_rows[index_in_file], width);
				CU_ASSERT_EQUAL(ret, 0);
			}

			if (!reverse && index_in_file >= 499)
				reverse = 1;
			else if (reverse && index_in_file <= 0)
				reverse = 0;
			index_in_file =
				reverse ? index_in_file - 1 : index_in_file + 1;
		}

		(void)vraw_reader_destroy(reader);

		free(data);

		for (i = 0; i < 500; i++)
			free(first_rows[i]);
		free(first_rows);
	}
}


CU_TestInfo g_vraw_test_reader[] = {
	{FN("vraw-reader-new"), &test_vraw_reader_new},
	{FN("vraw-reader-get-config"), &test_vraw_reader_get_config},
	{FN("vraw-reader-get-min-buf-size"),
	 &test_vraw_reader_get_min_buf_size},
	{FN("vraw-reader-get-file-frame-count"),
	 &test_vraw_reader_get_file_frame_count},
	{FN("vraw-reader-set-framerate"), &test_vraw_reader_set_framerate},
	{FN("vraw-reader-frame-read"), &test_vraw_reader_frame_read},
	{FN("vraw-reader-api"), &test_vraw_reader_api},
	{FN("vraw-reader-max-count"), &test_vraw_reader_max_count},
	{FN("vraw-reader-loop"), &test_vraw_reader_loop},

	CU_TEST_INFO_NULL,
};
