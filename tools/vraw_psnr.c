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

#define ULOG_TAG vraw_psnr
#include <ulog.h>
ULOG_DECLARE_TAG(vraw_psnr);


static const char short_options[] = "hf:F:W:H:d:D:c:";


static const struct option long_options[] = {
	{"help", no_argument, NULL, 'h'},
	{"format", required_argument, NULL, 'f'},
	{"format2", required_argument, NULL, 'F'},
	{"width", required_argument, NULL, 'W'},
	{"height", required_argument, NULL, 'H'},
	{"decimation", required_argument, NULL, 'd'},
	{"decimation2", required_argument, NULL, 'D'},
	{"csv", required_argument, NULL, 'c'},
	{0, 0, 0, 0},
};


static void welcome()
{
	printf("\nvraw_psnr - "
	       "Raw video library program computing PSNR between 2 files\n\n");
}


static void usage(int argc, char *argv[])
{
	printf("Usage: %s -f <format> -W <width> -H <height> <file_1> "
	       "<file_2>\n\n"
	       "Options:\n"
	       "  -h | --help                        "
	       "Print this message\n"
	       "  -f | --format <format>             "
	       "  -F | --format2 <format2>             "
	       "Data format (\"i420\", \"yv12\", \"nv12\", \"nv21\", "
	       "\"i420_10_16le\", \"yv12_10_16le\", \"nv12_10_16le\",\n"
	       "  -W | --width <width>               "
	       "Input width in pixel units "
	       "(useless if input is *.y4m)\n"
	       "  -H | --height <height>             "
	       "Input height in pixel units "
	       "(useless if input is *.y4m)\n"
	       "  -d | --decimation <format>         "
	       "  -D | --decimation2 <format2>       "
	       "Decimation factor for files 1 and 2\n"
	       "  -c | --csv <file>                  "
	       "Output the results to a CSV file\n"
	       "  <file_1>                           "
	       "path of file 1 on which PSNR must be computed\n"
	       "  <file_2>                           "
	       "path of file 2 on which PSNR must be computed\n"
	       "\n",
	       argv[0]);
}


int main(int argc, char **argv)
{
	int res, ret = EXIT_SUCCESS;
	struct vdef_raw_format format = {0};
	struct vdef_raw_format format2 = {0};
	struct vdef_dim resolution = {0};
	struct vraw_reader_config reader_config_1;
	struct vraw_reader_config reader_config_2;
	struct vraw_reader *reader_1 = NULL;
	struct vraw_reader *reader_2 = NULL;
	unsigned int decimation = 1, decimation2 = 1;
	uint8_t *data_1 = NULL;
	uint8_t *data_2 = NULL;
	size_t len = 0;
	unsigned int index = 0;
	struct vraw_frame frame_1;
	struct vraw_frame frame_2;
	double psnr[4];
	double psnr_mean[4];
	char *file_1 = NULL;
	char *file_2 = NULL;
	char *csv_file = NULL;
	FILE *csv = NULL;

	memset(psnr, 0, sizeof(psnr));
	memset(psnr_mean, 0, sizeof(psnr_mean));

	welcome();

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
			if (res != 0)
				ULOG_ERRNO("vdef_raw_format_from_str", -res);
			break;
		case 'F':
			res = vdef_raw_format_from_str(optarg, &format2);
			if (res != 0)
				ULOG_ERRNO("vdef_raw_format_from_str", -res);
			break;
		case 'W':
			sscanf(optarg, "%u", &resolution.width);
			break;

		case 'H':
			sscanf(optarg, "%u", &resolution.height);
			break;

		case 'd':
			decimation = atoi(optarg);
			break;

		case 'D':
			decimation2 = atoi(optarg);
			break;

		case 'c':
			csv_file = optarg;
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

	file_1 = argv[optind];
	file_2 = argv[optind + 1];

	printf("File1 : %s\n", file_1);
	printf("File2 : %s\n", file_2);

	if (csv_file != NULL)
		printf("CSV file : %s\n", csv_file);

	/* set format2 if necessary */
	if (format2.data_size == 0)
		format2 = format;

	/* reader config for file 1 */
	memset(&reader_config_1, 0, sizeof(reader_config_1));
	reader_config_1.format = format;
	reader_config_1.info.resolution = resolution;
	if ((strlen(file_1) > 4) &&
	    (strcmp(file_1 + strlen(file_1) - 4, ".y4m") == 0))
		reader_config_1.y4m = 1;

	res = vraw_reader_new(file_1, &reader_config_1, &reader_1);
	if (res < 0) {
		ULOG_ERRNO("vraw_reader_new", -res);
		ret = EXIT_FAILURE;
		goto out;
	}

	res = vraw_reader_get_config(reader_1, &reader_config_1);
	if (res < 0) {
		ULOG_ERRNO("vraw_reader_get_config", -res);
		ret = EXIT_FAILURE;
		goto out;
	}


	/* reader config for file 2 */
	memset(&reader_config_2, 0, sizeof(reader_config_1));
	reader_config_2.format = format2;
	reader_config_2.info.resolution = resolution;
	if ((strlen(file_2) > 4) &&
	    (strcmp(file_2 + strlen(file_2) - 4, ".y4m") == 0))
		reader_config_2.y4m = 1;

	res = vraw_reader_new(file_2, &reader_config_2, &reader_2);
	if (res < 0) {
		ULOG_ERRNO("vraw_reader_new", -res);
		ret = EXIT_FAILURE;
		goto out;
	}

	res = vraw_reader_get_config(reader_2, &reader_config_2);
	if (res < 0) {
		ULOG_ERRNO("vraw_reader_get_config", -res);
		ret = EXIT_FAILURE;
		goto out;
	}

	/* common to 1 & 2 */
	if (vdef_dim_is_null(&reader_config_1.info.resolution)) {
		ULOGE("invalid video dimensions: %dx%d",
		      reader_config_1.info.resolution.width,
		      reader_config_1.info.resolution.height);
		ret = EXIT_FAILURE;
		goto out;
	}

	if (vdef_raw_format_cmp(&reader_config_1.format,
				&reader_config_2.format)) {
		ULOGI("Format : " VDEF_RAW_FORMAT_TO_STR_FMT,
		      VDEF_RAW_FORMAT_TO_STR_ARG(&reader_config_1.format));

	} else {
		ULOGI("Format1: " VDEF_RAW_FORMAT_TO_STR_FMT,
		      VDEF_RAW_FORMAT_TO_STR_ARG(&reader_config_1.format));
		ULOGI("Format2: " VDEF_RAW_FORMAT_TO_STR_FMT,
		      VDEF_RAW_FORMAT_TO_STR_ARG(&reader_config_2.format));
	}

	ULOGI("Dimensions: %dx%d",
	      reader_config_1.info.resolution.width,
	      reader_config_1.info.resolution.height);

	if (csv_file != NULL) {
		csv = fopen(csv_file, "wt");
		if (csv == NULL) {
			res = -errno;
			ULOG_ERRNO("fopen", -res);
			ret = EXIT_FAILURE;
			goto out;
		}
	}

	/* read frames 1 by 1 and compute their PSNR */
	len = reader_config_1.info.resolution.width *
	      reader_config_1.info.resolution.height * 3 / 2;
	len *= format.data_size / 8;
	data_1 = malloc(len);
	data_2 = malloc(len);

	memset(&frame_1, 0, sizeof(frame_1));
	memset(&frame_2, 0, sizeof(frame_2));

	while (res != -ENOENT) {
		do {
			res = vraw_reader_frame_read(
				reader_1, data_1, len, &frame_1);
			if ((res < 0) && (res != -ENOENT)) {
				ULOG_ERRNO("vraw_reader_frame_read", -res);
				ret = EXIT_FAILURE;
				break;
			}
			if (res == -ENOENT)
				break;
		} while ((frame_1.frame.info.index % decimation) ||
			 (frame_1.frame.info.index / decimation != index));
		if (res == -ENOENT)
			break;

		do {
			res = vraw_reader_frame_read(
				reader_2, data_2, len, &frame_2);
			if ((res < 0) && (res != -ENOENT)) {
				ULOG_ERRNO("vraw_reader_frame_read", -res);
				ret = EXIT_FAILURE;
				break;
			}
			if (res == -ENOENT)
				break;
		} while ((frame_2.frame.info.index % decimation2) ||
			 (frame_2.frame.info.index / decimation2 != index));
		if (res == -ENOENT)
			break;

		res = vraw_compute_psnr(&frame_1, &frame_2, psnr);
		if (res < 0) {
			ULOG_ERRNO("vraw_psnr_meta", -res);
			ret = EXIT_FAILURE;
			break;
		}

		ULOGI("frame #%d, PSNR Y=%.3f, U=%.3f, V=%.3f",
		      index,
		      psnr[0],
		      psnr[1],
		      psnr[2]);

		if (csv != NULL) {
			fprintf(csv,
				"%d %.3f %.3f %.3f\n",
				index,
				psnr[0],
				psnr[1],
				psnr[2]);
		}

		psnr_mean[0] += psnr[0];
		psnr_mean[1] += psnr[1];
		psnr_mean[2] += psnr[2];

		index++;
	}

	if (index > 0) {
		psnr_mean[0] /= index;
		psnr_mean[1] /= index;
		psnr_mean[2] /= index;
		if (csv != NULL) {
			fprintf(csv,
				"#mean %.3f %.3f %.3f\n",
				psnr_mean[0],
				psnr_mean[1],
				psnr_mean[2]);
		}
	} else {
		ULOGE("0 frame processed. Mean PSNR computation impossible.");
		goto out;
	}

	printf("Mean PSNR: Y = %.3f dB, U = %.3f dB, V = %.3f dB\n",
	       psnr_mean[0],
	       psnr_mean[1],
	       psnr_mean[2]);

out:
	res = vraw_reader_destroy(reader_1);
	if (res < 0)
		ULOG_ERRNO("vraw_reader_destroy", -res);

	res = vraw_reader_destroy(reader_2);
	if (res < 0)
		ULOG_ERRNO("vraw_reader_destroy", -res);
	if (csv != NULL)
		fclose(csv);

	free(data_1);
	free(data_2);

	exit(ret);
}
