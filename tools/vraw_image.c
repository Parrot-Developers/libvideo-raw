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
#include <stdio.h>

#include <video-raw/vraw.h>

#define ULOG_TAG vraw_image
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);


static const char short_options[] = "h";


static const struct option long_options[] = {
	{"help", no_argument, NULL, 'h'},
	{0, 0, 0, 0},
};


static void welcome(int argc, char **argv)
{
	printf("\n%s - "
	       "Raw video library image program\n\n",
	       argv[0]);
}


static void usage(int argc, char *argv[])
{
	printf("Usage: %s <input_file> [<output_file>]\n\n"
	       "Options:\n"
	       "  -h | --help                        "
	       "Print this message\n"
	       "\n",
	       argv[0]);
}


int main(int argc, char **argv)
{
	int res, ret = EXIT_SUCCESS;
	char *input = NULL, *output = NULL;
	uint8_t *data = NULL;
	size_t len = 0;
	struct vraw_frame frame;
	FILE *f = NULL;

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

		default:
			usage(argc, argv);
			exit(EXIT_FAILURE);
			break;
		}
	}

	if (argc - optind < 1) {
		usage(argc, argv);
		exit(EXIT_FAILURE);
	}

	input = argv[optind];
	if (optind + 1 < argc)
		output = argv[optind + 1];

	len = 0;
	res = vraw_image_read(input, NULL, &len, NULL);
	if (res < 0) {
		ULOG_ERRNO("vraw_image_read", -res);
		ret = EXIT_FAILURE;
		goto out;
	}

	data = malloc(len);
	if (data == NULL) {
		ULOG_ERRNO("malloc", ENOMEM);
		ret = EXIT_FAILURE;
		goto out;
	}

	memset(&frame, 0, sizeof(frame));

	res = vraw_image_read(input, data, &len, &frame);
	if (res < 0) {
		ULOG_ERRNO("vraw_image_read", -res);
		ret = EXIT_FAILURE;
		goto out;
	}

	ULOGI("Format: " VDEF_RAW_FORMAT_TO_STR_FMT,
	      VDEF_RAW_FORMAT_TO_STR_ARG(&frame.frame.format));
	ULOGI("Dimensions: %dx%d",
	      frame.frame.info.resolution.width,
	      frame.frame.info.resolution.height);

	if (output != NULL) {
		f = fopen(output, "wb");
		if (f == NULL) {
			res = -errno;
			ULOG_ERRNO("fopen('%s')", -res, output);
			ret = EXIT_FAILURE;
			goto out;
		}
		uint8_t *ptr = data;
		size_t w = frame.frame.info.resolution.width * 3;
		if (vdef_raw_format_cmp(&frame.frame.format, &vdef_rgba) ||
		    vdef_raw_format_cmp(&frame.frame.format, &vdef_abgr))
			w = frame.frame.info.resolution.width * 4;
		for (unsigned int i = 0; i < frame.frame.info.resolution.height;
		     i++) {
			fwrite(ptr, w, 1, f);
			ptr += frame.frame.plane_stride[0];
		}
	}

out:
	free(data);
	if (f != NULL)
		fclose(f);

	exit(ret);
}
