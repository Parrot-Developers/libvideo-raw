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
#include <getopt.h>
#include <inttypes.h>
#include <stdio.h>

#include <video-raw/vraw.h>

#define ULOG_TAG vraw_rewrite
#include <ulog.h>
ULOG_DECLARE_TAG(vraw_rewrite);


static const char short_options[] = "hf:W:H:F:s:l:";


static const struct option long_options[] = {
	{"help", no_argument, NULL, 'h'},
	{"format", required_argument, NULL, 'f'},
	{"width", required_argument, NULL, 'W'},
	{"height", required_argument, NULL, 'H'},
	{"framerate", required_argument, NULL, 'F'},
	{"sar", required_argument, NULL, 's'},
	{"loop", required_argument, NULL, 'l'},
	{0, 0, 0, 0},
};


static void welcome(int argc, char **argv)
{
	printf("\n%s - "
	       "Raw video library rewriting program\n\n",
	       argv[0]);
}


static void usage(int argc, char *argv[])
{
	printf("Usage: %s [options] <input_file> <output_file>\n\n"
	       "Options:\n"
	       "  -h | --help                        "
	       "Print this message\n"
	       "  -f | --format <format>             "
	       "Data format (\"I420\", \"YV12\", \"NV12\", \"NV21\")\n"
	       "  -W | --width <width>               "
	       "Input width in pixel units "
	       "(useless if input is *.y4m)\n"
	       "  -H | --height <height>             "
	       "Input height in pixel units "
	       "(useless if input is *.y4m)\n"
	       "  -F | --framerate <framerate>       "
	       "Input framerate, format num/den "
	       "(useless if input is *.y4m)\n"
	       "  -s | --sar <sar>                   "
	       "Source aspect ratio, format w:h "
	       "(useless if input is *.y4m)\n"
	       "  -l | --loop <dir>                  "
	       "Loop forever, dir=1: loop from beginning, "
	       "dir=-1: loop with reverse\n"
	       "\n",
	       argv[0]);
}


int main(int argc, char **argv)
{
	int res, ret = EXIT_SUCCESS;
	char *input = NULL, *output = NULL;
	struct vdef_raw_format format = {0};
	struct vdef_dim resolution = {0};
	struct vdef_frac framerate = {0};
	struct vdef_dim sar = {0};
	int loop = 0;
	struct vraw_reader_config reader_config;
	struct vraw_reader *reader = NULL;
	struct vraw_writer_config writer_config;
	struct vraw_writer *writer = NULL;
	uint8_t *data = NULL;
	size_t len = 0;
	ssize_t res1;
	struct vraw_frame in_frame, out_frame;

	welcome(argc, argv);

	/* Command-line parameters */
	int idx, c;
	while ((c = getopt_long(
			argc, argv, short_options, long_options, &idx)) != -1) {
		switch (c) {
		case 0:
			break;

		case 'h':
			usage(argc, argv);
			exit(EXIT_SUCCESS);
			break;

		case 'f':
			res = vdef_raw_format_from_str(optarg, &format);
			if (res != 0) {
				ULOG_ERRNO("vdef_raw_format_from_str", -res);
				usage(argc, argv);
				exit(EXIT_FAILURE);
			}
			break;

		case 'W':
			sscanf(optarg, "%u", &resolution.width);
			break;

		case 'H':
			sscanf(optarg, "%u", &resolution.height);
			break;

		case 'F':
			sscanf(optarg, "%u/%u", &framerate.num, &framerate.den);
			break;

		case 's':
			sscanf(optarg, "%u:%u", &sar.width, &sar.height);
			break;

		case 'l':
			sscanf(optarg, "%d", &loop);
			break;

		default:
			usage(argc, argv);
			exit(EXIT_FAILURE);
			break;
		}
	}

	if (argc - optind < 2) {
		usage(argc, argv);
		exit(EXIT_FAILURE);
	}

	input = argv[optind];
	output = argv[optind + 1];

	memset(&reader_config, 0, sizeof(reader_config));
	memset(&writer_config, 0, sizeof(writer_config));

	reader_config.loop = loop;
	reader_config.format = format;
	reader_config.info.resolution = resolution;
	reader_config.info.framerate = framerate;
	reader_config.info.sar = sar;
	if ((strlen(input) > 4) &&
	    (strcmp(input + strlen(input) - 4, ".y4m") == 0))
		reader_config.y4m = 1;

	res = vraw_reader_new(input, &reader_config, &reader);
	if (res < 0) {
		ULOG_ERRNO("vraw_reader_new", -res);
		ret = EXIT_FAILURE;
		goto out;
	}

	res = vraw_reader_get_config(reader, &reader_config);
	if (res < 0) {
		ULOG_ERRNO("vraw_reader_get_config", -res);
		ret = EXIT_FAILURE;
		goto out;
	}

	if (vdef_dim_is_null(&reader_config.info.resolution)) {
		ULOGE("invalid video dimensions: %dx%d",
		      reader_config.info.resolution.width,
		      reader_config.info.resolution.height);
		ret = EXIT_FAILURE;
		goto out;
	}

	ULOGI("Format: " VDEF_RAW_FORMAT_TO_STR_FMT,
	      VDEF_RAW_FORMAT_TO_STR_ARG(&reader_config.format));
	ULOGI("Bit depth: %d bits", reader_config.format.data_size);
	ULOGI("Dimensions: %dx%d",
	      reader_config.info.resolution.width,
	      reader_config.info.resolution.height);
	ULOGI("Framerate: %d/%d",
	      reader_config.info.framerate.num,
	      reader_config.info.framerate.den);
	ULOGI("SAR: %d:%d",
	      reader_config.info.sar.width,
	      reader_config.info.sar.height);

	writer_config.format = reader_config.format;
	writer_config.info = reader_config.info;
	if ((strlen(output) > 4) &&
	    (strcmp(output + strlen(output) - 4, ".y4m") == 0))
		writer_config.y4m = 1;

	res = vraw_writer_new(output, &writer_config, &writer);
	if (res < 0) {
		ULOG_ERRNO("vraw_writer_new", -res);
		ret = EXIT_FAILURE;
		goto out;
	}

	res1 = vraw_reader_get_min_buf_size(reader);
	if (res1 < 0) {
		ULOG_ERRNO("vraw_reader_get_min_buf_size", (int)-res1);
		ret = EXIT_FAILURE;
		goto out;
	}
	len = res1;
	data = malloc(len);
	if (data == NULL) {
		ULOG_ERRNO("malloc", ENOMEM);
		ret = EXIT_FAILURE;
		goto out;
	}

	memset(&in_frame, 0, sizeof(in_frame));
	memset(&out_frame, 0, sizeof(out_frame));

	while (res != -ENOENT) {
		res = vraw_reader_frame_read(reader, data, len, &in_frame);
		if ((res < 0) && (res != -ENOENT)) {
			ULOG_ERRNO("vraw_reader_frame_read", -res);
			ret = EXIT_FAILURE;
			break;
		}
		if (res == -ENOENT)
			break;
		ULOGI("read frame #%d ts=%" PRIu64,
		      in_frame.frame.info.index,
		      in_frame.frame.info.timestamp);

		out_frame = in_frame;

		res = vraw_writer_frame_write(writer, &out_frame);
		if (res < 0) {
			ULOG_ERRNO("vraw_writer_frame_write", -res);
			ret = EXIT_FAILURE;
			break;
		}
	}

out:
	res = vraw_reader_destroy(reader);
	if (res < 0)
		ULOG_ERRNO("vraw_reader_destroy", -res);
	res = vraw_writer_destroy(writer);
	if (res < 0)
		ULOG_ERRNO("vraw_writer_destroy", -res);
	free(data);

	exit(ret);
}
