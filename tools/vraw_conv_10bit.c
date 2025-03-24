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

#define ULOG_TAG vraw_conv_10bit
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);


static const char short_options[] = "hi:o:W:H:F:s:l:";


static const struct option long_options[] = {
	{"help", no_argument, NULL, 'h'},
	{"input-format", required_argument, NULL, 'i'},
	{"output-format", required_argument, NULL, 'o'},
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
	       "10bit raw video conversion program\n\n",
	       argv[0]);
}


static void usage(int argc, char *argv[])
{
	printf("Usage: %s [options] <input_file> <output_file>\n\n"
	       "Options:\n"
	       "  -h | --help                        "
	       "Print this message\n"
	       "  -i | --input-format <format>       "
	       "Input data format (\"i420\", \"yv12\", \"nv12\", \"nv21\", "
	       "\"i420_10_16le\", \"yv12_10_16le\", \"nv12_10_16le\",\n"
	       "                                     "
	       "\"nv21_10_16le\", \"i420_10_16be\", \"yv12_10_16be\", "
	       "\"nv12_10_16be\", \"nv21_10_16be\", \"nv21_10_packed\")\n"
	       "  -o | --output-format <format>      "
	       "Output data format (\"i420\", \"yv12\", \"nv12\", \"nv21\", "
	       "\"i420_10_16le\", \"yv12_10_16le\", \"nv12_10_16le\",\n"
	       "                                     "
	       "\"nv21_10_16le\", \"i420_10_16be\", \"yv12_10_16be\", "
	       "\"nv12_10_16be\", \"nv21_10_16be\", \"nv21_10_packed\")\n"
	       "  -W | --width <width>               "
	       "Width in pixel units (useless if input is *.y4m)\n"
	       "  -H | --height <height>             "
	       "Height in pixel units (useless if input is *.y4m)\n"
	       "  -F | --framerate <framerate>       "
	       "Framerate, format num/den (useless if input is *.y4m)\n"
	       "  -s | --sar <sar>                   "
	       "Source aspect ratio, format w:h (useless if input is *.y4m)\n"
	       "  -l | --loop <dir>                  "
	       "Loop forever, dir=1: loop from beginning, "
	       "dir=-1: loop with reverse\n"
	       "\n"
	       "Either --input-format or --output-format must be "
	       "\"nv21_10_packed\".\n"
	       "\n",
	       argv[0]);
}


static size_t get_out_size(size_t size_in,
			   struct vdef_raw_format *fmt_in,
			   struct vdef_raw_format *fmt_out)
{
	return size_in * fmt_out->data_size / fmt_in->data_size;
}


static size_t get_frame_size(struct vraw_frame *frame)
{
	size_t plane_sizes[VDEF_RAW_MAX_PLANE_COUNT] = {0};
	int ret = vdef_calc_raw_frame_size(&frame->frame.format,
					   &frame->frame.info.resolution,
					   NULL,
					   NULL,
					   NULL,
					   NULL,
					   plane_sizes,
					   NULL);
	if (ret < 0) {
		ULOG_ERRNO("vdef_calc_raw_frame_size", -ret);
		return 0;
	}

	size_t total_size = 0;
	for (unsigned int i = 0;
	     i < vdef_get_raw_frame_plane_count(&frame->frame.format);
	     i++) {
		total_size += plane_sizes[i];
	}
	return total_size;
}


static void conv_16le_to_10packed(uint8_t *in1,
				  uint8_t *in2,
				  uint8_t *out,
				  int step,
				  int nb_blk)
{
	for (int n = 0; n < nb_blk;
	     n++, in1 += 2 * step, in2 += 2 * step, out += 5) {

		uint64_t t = 0;

		t = in2[step + 1] & 3;
		t = (t << 8) + in2[step];

		t = (t << 2) + (in1[step + 1] & 3);
		t = (t << 8) + in1[step];

		t = (t << 2) + (in2[1] & 3);
		t = (t << 8) + in2[0];

		t = (t << 2) + (in1[1] & 3);
		t = (t << 8) + in1[0];

		for (int i = 0; i < 5; i++) {
			out[i] = (uint8_t)(t & 0xFF);
			t >>= 8;
		}
	}
}


static void conv_16be_to_10packed(uint8_t *in1,
				  uint8_t *in2,
				  uint8_t *out,
				  int step,
				  int nb_blk)
{
	for (int n = 0; n < nb_blk;
	     n++, in1 += 2 * step, in2 += 2 * step, out += 5) {

		uint64_t t = 0;

		t = in2[step] & 3;
		t = (t << 8) + in2[step + 1];

		t = (t << 2) + (in1[step] & 3);
		t = (t << 8) + in1[step + 1];

		t = (t << 2) + (in2[0] & 3);
		t = (t << 8) + in2[1];

		t = (t << 2) + (in1[0] & 3);
		t = (t << 8) + in1[1];

		for (int i = 0; i < 5; i++) {
			out[i] = (uint8_t)(t & 0xFF);
			t >>= 8;
		}
	}
}


static void conv_8_to_10packed(uint8_t *in1,
			       uint8_t *in2,
			       uint8_t *out,
			       int step,
			       int nb_blk)
{
	for (int n = 0; n < nb_blk;
	     n++, in1 += 2 * step, in2 += 2 * step, out += 5) {

		uint64_t t = 0;

		t = in2[step];
		t = (t << 10) + in1[step];
		t = (t << 10) + in2[0];
		t = (t << 10) + in1[0];
		t <<= 2;

		for (int i = 0; i < 5; i++) {
			out[i] = (uint8_t)(t & 0xFF);
			t >>= 8;
		}
	}
}


static void line_10packed_to_16(const uint8_t *in,
				uint8_t *out1_low,
				uint8_t *out1_high,
				uint8_t *out2_low,
				uint8_t *out2_high,
				size_t out_pixel_stride,
				size_t blocks)
{
	for (size_t n = 0; n < blocks; n++,
		    in += 5,
		    out1_low += 2 * out_pixel_stride,
		    out1_high += 2 * out_pixel_stride,
		    out2_low += 2 * out_pixel_stride,
		    out2_high += 2 * out_pixel_stride) {
		uint64_t t = 0, v;

		/* Read 40 bits (4 packed pixel values) */
		for (size_t i = 0; i < 5; i++) {
			t |= in[4 - i];
			t <<= 8;
		}

		/* Write 64 bits (4 pixel values with padding) */
		t >>= 8;
		v = t & 0x3FF;
		out1_low[0] = (v >> 0) & 0xFF;
		out1_high[0] = (v >> 8) & 0xFF;
		t >>= 10;
		v = t & 0x3FF;
		out2_low[0] = (v >> 0) & 0xFF;
		out2_high[0] = (v >> 8) & 0xFF;
		t >>= 10;
		v = t & 0x3FF;
		out1_low[out_pixel_stride] = (v >> 0) & 0xFF;
		out1_high[out_pixel_stride] = (v >> 8) & 0xFF;
		t >>= 10;
		v = t & 0x3FF;
		out2_low[out_pixel_stride] = (v >> 0) & 0xFF;
		out2_high[out_pixel_stride] = (v >> 8) & 0xFF;
	}
}


static void line_10packed_to_8(const uint8_t *in,
			       uint8_t *out1,
			       uint8_t *out2,
			       size_t out_pixel_stride,
			       size_t blocks)
{
	for (size_t n = 0; n < blocks; n++,
		    in += 5,
		    out1 += 2 * out_pixel_stride,
		    out2 += 2 * out_pixel_stride) {
		uint64_t t = 0, v;

		/* Read 40 bits (4 packed pixel values) */
		for (size_t i = 0; i < 5; i++) {
			t |= in[4 - i];
			t <<= 8;
		}

		/* Write 32 bits (4 pixel values with padding) */
		t >>= 8;
		v = ((t & 0x3FF) + 2) >> 2;
		out1[0] = v & 0xFF;
		t >>= 10;
		v = ((t & 0x3FF) + 2) >> 2;
		out2[0] = v & 0xFF;
		t >>= 10;
		v = ((t & 0x3FF) + 2) >> 2;
		out1[out_pixel_stride] = v & 0xFF;
		t >>= 10;
		v = ((t & 0x3FF) + 2) >> 2;
		out2[out_pixel_stride] = v & 0xFF;
	}
}


static void line_to_nv21_10packed(uint8_t *in1,
				  uint8_t *in2,
				  uint8_t *out,
				  int nb_blk,
				  int step,
				  int mode)
{
	if (mode == 0) /* 16LE */
		conv_16le_to_10packed(in1, in2, out, step, nb_blk);
	else if (mode == 1) /* 16BE */
		conv_16be_to_10packed(in1, in2, out, step, nb_blk);
	else /* 8 bits */
		conv_8_to_10packed(in1, in2, out, step, nb_blk);
}


static void frame_to_nv21_10packed(struct vraw_frame *in_frame,
				   struct vraw_frame *out_frame)
{
	unsigned int w = in_frame->frame.info.resolution.width;
	unsigned int h = in_frame->frame.info.resolution.height;
	int elem_size = in_frame->frame.format.data_size / 8;
	uint8_t *y_out = (uint8_t *)out_frame->cdata[0];
	uint8_t *vu_out = (uint8_t *)out_frame->cdata[1];

	uint8_t *y_in = (uint8_t *)in_frame->data[0];

	uint8_t *u_in = NULL;
	uint8_t *v_in = NULL;
	uint32_t jmp_uv = 0;
	int mode = 2;

	if (vdef_raw_format_cmp(&in_frame->frame.format, &vdef_i420) ||
	    vdef_raw_format_cmp(&in_frame->frame.format, &vdef_i420_10_16le) ||
	    vdef_raw_format_cmp(&in_frame->frame.format, &vdef_i420_10_16be)) {
		u_in = in_frame->data[1];
		v_in = in_frame->data[2];
		jmp_uv = elem_size;
	} else if (vdef_raw_format_cmp(&in_frame->frame.format, &vdef_yv12) ||
		   vdef_raw_format_cmp(&in_frame->frame.format,
				       &vdef_yv12_10_16le) ||
		   vdef_raw_format_cmp(&in_frame->frame.format,
				       &vdef_yv12_10_16be)) {
		u_in = in_frame->data[2];
		v_in = in_frame->data[1];
		jmp_uv = elem_size;
	} else if (vdef_raw_format_cmp(&in_frame->frame.format, &vdef_nv12) ||
		   vdef_raw_format_cmp(&in_frame->frame.format,
				       &vdef_nv12_10_16le) ||
		   vdef_raw_format_cmp(&in_frame->frame.format,
				       &vdef_nv12_10_16be)) {
		u_in = in_frame->data[1];
		v_in = in_frame->data[1] + elem_size;
		jmp_uv = elem_size * 2;
	} else if (vdef_raw_format_cmp(&in_frame->frame.format, &vdef_nv21) ||
		   vdef_raw_format_cmp(&in_frame->frame.format,
				       &vdef_nv21_10_16le) ||
		   vdef_raw_format_cmp(&in_frame->frame.format,
				       &vdef_nv21_10_16be)) {
		v_in = in_frame->data[1];
		u_in = in_frame->data[1] + elem_size;
		jmp_uv = elem_size * 2;
	} else {
		return;
	}

	if (elem_size == 2)
		mode = (in_frame->frame.format.data_little_endian) ? 0 : 1;

	for (unsigned int i = 0; i < h; i++) {
		line_to_nv21_10packed(y_in,
				      y_in + elem_size,
				      y_out,
				      w / 4,
				      2 * elem_size,
				      mode);
		y_in += in_frame->frame.plane_stride[0];
		y_out += out_frame->frame.plane_stride[0];
	}

	for (unsigned int i = 0; i < h / 2; i++) {
		line_to_nv21_10packed(v_in, u_in, vu_out, w / 4, jmp_uv, mode);
		v_in += in_frame->frame.plane_stride[1];
		u_in += in_frame->frame.plane_stride[1];
		vu_out += out_frame->frame.plane_stride[1];
	}
}


static void frame_from_nv21_10packed(struct vraw_frame *in_frame,
				     struct vraw_frame *out_frame)
{
	unsigned int w = in_frame->frame.info.resolution.width;
	unsigned int h = in_frame->frame.info.resolution.height;
	unsigned int data_size = out_frame->frame.format.data_size / 8;
	bool little_endian = out_frame->frame.format.data_little_endian;
	const uint8_t *y_in = in_frame->cdata[0];
	const uint8_t *vu_in = in_frame->cdata[1];

	uint8_t *y_out_low = out_frame->data[0];
	uint8_t *y_out_high = out_frame->data[0];
	uint8_t *u_out_low = NULL;
	uint8_t *u_out_high = NULL;
	uint8_t *v_out_low = NULL;
	uint8_t *v_out_high = NULL;
	size_t uv_pixel_stride = 0, u_line_stride = 0, v_line_stride = 0;

	if (vdef_raw_format_cmp(&out_frame->frame.format, &vdef_i420) ||
	    vdef_raw_format_cmp(&out_frame->frame.format, &vdef_i420_10_16le) ||
	    vdef_raw_format_cmp(&out_frame->frame.format, &vdef_i420_10_16be)) {
		u_out_low = u_out_high = out_frame->data[1];
		v_out_low = v_out_high = out_frame->data[2];
		u_line_stride = out_frame->frame.plane_stride[1];
		v_line_stride = out_frame->frame.plane_stride[2];
		uv_pixel_stride = data_size;
	} else if (vdef_raw_format_cmp(&out_frame->frame.format, &vdef_yv12) ||
		   vdef_raw_format_cmp(&out_frame->frame.format,
				       &vdef_yv12_10_16le) ||
		   vdef_raw_format_cmp(&out_frame->frame.format,
				       &vdef_yv12_10_16be)) {
		u_out_low = u_out_high = out_frame->data[2];
		v_out_low = v_out_high = out_frame->data[1];
		u_line_stride = out_frame->frame.plane_stride[2];
		v_line_stride = out_frame->frame.plane_stride[1];
		uv_pixel_stride = data_size;
	} else if (vdef_raw_format_cmp(&out_frame->frame.format, &vdef_nv12) ||
		   vdef_raw_format_cmp(&out_frame->frame.format,
				       &vdef_nv12_10_16le) ||
		   vdef_raw_format_cmp(&out_frame->frame.format,
				       &vdef_nv12_10_16be)) {
		u_out_low = u_out_high = out_frame->data[1];
		v_out_low = v_out_high = out_frame->data[1] + data_size;
		u_line_stride = out_frame->frame.plane_stride[1];
		v_line_stride = out_frame->frame.plane_stride[1];
		uv_pixel_stride = data_size * 2;
	} else if (vdef_raw_format_cmp(&out_frame->frame.format, &vdef_nv21) ||
		   vdef_raw_format_cmp(&out_frame->frame.format,
				       &vdef_nv21_10_16le) ||
		   vdef_raw_format_cmp(&out_frame->frame.format,
				       &vdef_nv21_10_16be)) {
		u_out_low = u_out_high = out_frame->data[1] + data_size;
		v_out_low = v_out_high = out_frame->data[1];
		u_line_stride = out_frame->frame.plane_stride[1];
		v_line_stride = out_frame->frame.plane_stride[1];
		uv_pixel_stride = data_size * 2;
	} else {
		return;
	}

	if (data_size == 2) {
		y_out_low += (little_endian) ? 0 : 1;
		y_out_high += (little_endian) ? 1 : 0;
		u_out_low += (little_endian) ? 0 : 1;
		u_out_high += (little_endian) ? 1 : 0;
		v_out_low += (little_endian) ? 0 : 1;
		v_out_high += (little_endian) ? 1 : 0;

		for (unsigned int i = 0; i < h; i++) {
			line_10packed_to_16(y_in,
					    y_out_low,
					    y_out_high,
					    y_out_low + 2,
					    y_out_high + 2,
					    data_size * 2,
					    w / 4);
			y_in += in_frame->frame.plane_stride[0];
			y_out_low += out_frame->frame.plane_stride[0];
			y_out_high += out_frame->frame.plane_stride[0];
		}

		for (unsigned int i = 0; i < h / 2; i++) {
			line_10packed_to_16(vu_in,
					    v_out_low,
					    v_out_high,
					    u_out_low,
					    u_out_high,
					    uv_pixel_stride,
					    w / 4);
			vu_in += in_frame->frame.plane_stride[1];
			u_out_low += u_line_stride;
			u_out_high += u_line_stride;
			v_out_low += v_line_stride;
			v_out_high += v_line_stride;
		}
	} else if (data_size == 1) {
		for (unsigned int i = 0; i < h; i++) {
			line_10packed_to_8(y_in,
					   y_out_low,
					   y_out_low + 1,
					   data_size * 2,
					   w / 4);
			y_in += in_frame->frame.plane_stride[0];
			y_out_low += out_frame->frame.plane_stride[0];
		}

		for (unsigned int i = 0; i < h / 2; i++) {
			line_10packed_to_8(vu_in,
					   v_out_low,
					   u_out_low,
					   uv_pixel_stride,
					   w / 4);
			vu_in += in_frame->frame.plane_stride[1];
			u_out_low += u_line_stride;
			v_out_low += v_line_stride;
		}
	}
}


int main(int argc, char **argv)
{
	int res, ret = EXIT_SUCCESS;
	char *input = NULL, *output = NULL;
	struct vdef_raw_format in_format = {0}, out_format = {0};
	struct vraw_frame in_frame = {0}, out_frame = {0};
	struct vdef_dim resolution = {0};
	struct vdef_frac framerate = {0};
	struct vdef_dim sar = {0};
	int loop = 0;
	struct vraw_reader_config reader_config;
	struct vraw_reader *reader = NULL;
	struct vraw_writer_config writer_config;
	struct vraw_writer *writer = NULL;
	uint8_t *data = NULL;
	uint8_t *data_out = NULL;
	size_t len = 0;
	size_t len_out = 0;
	ssize_t res1;

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

		case 'i':
			res = vdef_raw_format_from_str(optarg, &in_format);
			if (res != 0)
				ULOG_ERRNO("vdef_raw_format_from_str", -res);
			break;

		case 'o':
			res = vdef_raw_format_from_str(optarg, &out_format);
			if (res != 0)
				ULOG_ERRNO("vdef_raw_format_from_str", -res);
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

	if (!vdef_raw_format_cmp(&in_format, &vdef_nv21_10_packed) &&
	    !vdef_raw_format_cmp(&out_format, &vdef_nv21_10_packed)) {
		printf("Either --input-format or --output-format must be "
		       "\"nv21_10_packed\".\n");
		exit(EXIT_FAILURE);
	}

	input = argv[optind];
	output = argv[optind + 1];

	memset(&reader_config, 0, sizeof(reader_config));
	memset(&writer_config, 0, sizeof(writer_config));

	reader_config.loop = loop;
	reader_config.format = in_format;
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
	ULOGI("Dimensions: %dx%d",
	      reader_config.info.resolution.width,
	      reader_config.info.resolution.height);
	ULOGI("Framerate: %d/%d",
	      reader_config.info.framerate.num,
	      reader_config.info.framerate.den);
	ULOGI("SAR: %d:%d",
	      reader_config.info.sar.width,
	      reader_config.info.sar.height);

	writer_config.format = out_format;
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

	len_out =
		get_out_size(len, &reader_config.format, &writer_config.format);
	data_out = malloc(len_out);
	memset(data_out, 0x80, len_out);

	if (data == NULL || data_out == NULL) {
		ULOG_ERRNO("malloc", ENOMEM);
		ret = EXIT_FAILURE;
		goto out;
	}

	vdef_format_to_frame_info(&writer_config.info, &out_frame.frame.info);
	out_frame.frame.format = writer_config.format;

	out_frame.frame.plane_stride[0] = writer_config.info.resolution.width *
					  writer_config.format.data_size / 8;
	out_frame.cdata[0] = data_out;
	out_frame.cdata[1] = out_frame.cdata[0] +
			     out_frame.frame.plane_stride[0] *
				     writer_config.info.resolution.height;

	switch (writer_config.format.data_layout) {
	case VDEF_RAW_DATA_LAYOUT_PLANAR:
		out_frame.frame.plane_stride[1] =
			out_frame.frame.plane_stride[0] / 2;
		out_frame.frame.plane_stride[2] =
			out_frame.frame.plane_stride[1];
		out_frame.cdata[2] =
			out_frame.cdata[1] +
			out_frame.frame.plane_stride[1] *
				writer_config.info.resolution.height / 2;
		break;
	case VDEF_RAW_DATA_LAYOUT_SEMI_PLANAR:
		out_frame.frame.plane_stride[1] =
			out_frame.frame.plane_stride[0];
		break;
	default:
		break;
	}

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

		out_frame.frame.info.timestamp = in_frame.frame.info.timestamp;
		out_frame.frame.info.timescale = in_frame.frame.info.timescale;
		out_frame.frame.info.capture_timestamp =
			in_frame.frame.info.capture_timestamp;
		out_frame.frame.info.index = in_frame.frame.info.index;

		if (vdef_raw_format_cmp(&writer_config.format,
					&vdef_nv21_10_packed)) {
			frame_to_nv21_10packed(&in_frame, &out_frame);
		} else if (vdef_raw_format_cmp(&reader_config.format,
					       &vdef_nv21_10_packed)) {
			frame_from_nv21_10packed(&in_frame, &out_frame);
		}
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
	free(data_out);

	exit(ret);
}
