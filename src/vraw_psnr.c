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
#include <float.h>
#include <math.h>
#include <stdint.h>

#include <video-raw/vraw.h>

#define ULOG_TAG vraw
#include <ulog.h>


static int normalized_mse(const uint8_t *data1,
			  unsigned int step1,
			  size_t stride1,
			  const uint8_t *data2,
			  unsigned int step2,
			  size_t stride2,
			  unsigned int width,
			  unsigned int height,
			  unsigned int bit_depth,
			  double *mse_nor)
{
	double e = 0;
	double sum_e2 = 0;
	size_t i, j;

	unsigned int f = VDEF_ROUND_UP(bit_depth, 8);

	/* Check strides */
	if ((stride1 < (step1 * width * f)) ||
	    (stride2 < (step2 * width * f))) {
		ULOG_ERRNO("invalid stride", EINVAL);
		return -EINVAL;
	}

	if (f == 1) {
		for (j = 0; j < height; j++) {
			uint8_t *d1 = (uint8_t *)(data1 + j * stride1);
			uint8_t *d2 = (uint8_t *)(data2 + j * stride2);

			for (i = 0; i < width; i++, d1 += step1, d2 += step2) {
				e = (double)*d1 - (double)*d2;
				sum_e2 += e * e;
			}
		}
	} else {
		for (j = 0; j < height; j++) {
			uint16_t *d1 = (uint16_t *)(data1 + j * stride1);
			uint16_t *d2 = (uint16_t *)(data2 + j * stride2);

			for (i = 0; i < width; i++, d1 += step1, d2 += step2) {
				e = (double)*d1 - (double)*d2;
				sum_e2 += e * e;
			}
		}
	}

	/* MSE = cumulative squared error / (height * width) */

	double dyn = (1 << bit_depth) - 1;
	*mse_nor = sum_e2 / (width * height * dyn * dyn);

	return 0;
}


static int get_chroma_params(const struct vraw_frame *frame,
			     uint8_t **u,
			     uint32_t *stride_u,
			     uint8_t **v,
			     uint32_t *stride_v,
			     uint32_t *step)
{
	int res = 0;
	size_t el_size = frame->frame.format.data_size / 8;

	switch (frame->frame.format.data_layout) {
	case VDEF_RAW_DATA_LAYOUT_PLANAR:
		*step = 1;
		switch (frame->frame.format.pix_order) {
		case VDEF_RAW_PIX_ORDER_YUV:
			*u = frame->data[1];
			*v = frame->data[2];
			*stride_u = frame->frame.plane_stride[1];
			*stride_v = frame->frame.plane_stride[2];
			break;
		case VDEF_RAW_PIX_ORDER_YVU:
			*u = frame->data[2];
			*v = frame->data[1];
			*stride_u = frame->frame.plane_stride[2];
			*stride_v = frame->frame.plane_stride[1];
			break;
		default: /* not supported */
			res = -ENOSYS;
			ULOGE("unsupported pixel order: %s",
			      vdef_raw_pix_order_to_str(
				      frame->frame.format.pix_order));
			break;
		}
		break;

	case VDEF_RAW_DATA_LAYOUT_SEMI_PLANAR:
		*step = 2;
		*stride_u = frame->frame.plane_stride[1];
		*stride_v = frame->frame.plane_stride[1];

		switch (frame->frame.format.pix_order) {
		case VDEF_RAW_PIX_ORDER_YUV:
			*u = frame->data[1];
			*v = frame->data[1] + el_size;
			break;
		case VDEF_RAW_PIX_ORDER_YVU:
			*u = frame->data[1] + el_size;
			*v = frame->data[1];
			break;
		default: /* not supported */
			res = -ENOSYS;
			ULOGE("unsupported pixel order: %s",
			      vdef_raw_pix_order_to_str(
				      frame->frame.format.pix_order));
			break;
		}
		break;

	default: /* not suported */
		res = -ENOSYS;
		ULOGE("unsupported pix_order : %s",
		      vdef_raw_data_layout_to_str(
			      frame->frame.format.data_layout));
		break;
	}

	return res;
}


static double mse_norm_to_psnr(double mse_norm)
{
	/* psnr = -10 * log10(mse_norm)
	 * mse_norm = mse / (width * height * dyn * dyn)
	 * dyn is max_pixel_value = (1 << bit_depth) - 1 */
	if (mse_norm == 0.0) {
		ULOGI("MSE is null; PSNR is set to 1000.0");
		return 1000.0;
	} else {
		return -10 * log10(mse_norm);
	}
}


int vraw_compute_psnr(const struct vraw_frame *frame1,
		      const struct vraw_frame *frame2,
		      double psnr[4])
{
	int res = 0;

	/* Check resolution */
	if (!vdef_dim_cmp(&frame1->frame.info.resolution,
			  &frame2->frame.info.resolution)) {
		ULOGE("resolution mismatch");
		return -EINVAL;
	}

	if ((frame1->frame.info.resolution.width == 0) ||
	    (frame1->frame.info.resolution.height == 0)) {
		ULOGE("invalid resolution");
		return -EINVAL;
	}

	if (frame1->frame.format.pix_size != frame2->frame.format.pix_size) {
		ULOGE("bit depth mismatch");
		return -EINVAL;
	}

	/* Check data_size */
	if (frame1->frame.format.data_size != frame2->frame.format.data_size) {
		ULOGE("data size mismatch");
		return -EINVAL;
	}

	ULOG_ERRNO_RETURN_ERR_IF(frame1 == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(frame1->frame.info.resolution.width == 0,
				 EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(frame1->frame.info.resolution.height == 0,
				 EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(frame2 == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(frame2->frame.info.resolution.width == 0,
				 EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(frame2->frame.info.resolution.height == 0,
				 EINVAL);

	/* Check resolution */
	if (!vdef_dim_cmp(&frame1->frame.info.resolution,
			  &frame2->frame.info.resolution)) {
		ULOGE("resolution mismatch");
		return -EINVAL;
	}

	uint8_t *u1, *v1, *u2, *v2;
	uint32_t step1, stride_u1, stride_v1, step2, stride_u2, stride_v2;

	res = get_chroma_params(
		frame1, &u1, &stride_u1, &v1, &stride_v1, &step1);
	if (res != 0)
		return res;

	res = get_chroma_params(
		frame2, &u2, &stride_u2, &v2, &stride_v2, &step2);
	if (res != 0)
		return res;

	double mse_norm[3];
	memset(mse_norm, 0, sizeof(mse_norm));

	/* Process Y */
	res = normalized_mse(frame1->cdata[0],
			     1,
			     frame1->frame.plane_stride[0],
			     frame2->cdata[0],
			     1,
			     frame2->frame.plane_stride[0],
			     frame1->frame.info.resolution.width,
			     frame1->frame.info.resolution.height,
			     frame1->frame.format.pix_size,
			     &mse_norm[0]);
	if (res != 0)
		return res;

	/* Process U */
	res = normalized_mse(u1,
			     step1,
			     stride_u1,
			     u2,
			     step2,
			     stride_u2,
			     frame1->frame.info.resolution.width / 2,
			     frame1->frame.info.resolution.height / 2,
			     frame1->frame.format.pix_size,
			     &mse_norm[1]);
	if (res != 0)
		return res;

	/* Process V */
	res = normalized_mse(v1,
			     step1,
			     stride_v1,
			     v2,
			     step2,
			     stride_v2,
			     frame1->frame.info.resolution.width / 2,
			     frame1->frame.info.resolution.height / 2,
			     frame1->frame.format.pix_size,
			     &mse_norm[2]);
	if (res != 0)
		return res;

	memset(psnr, 0, sizeof(double) * 4);

	for (int i = 0; i < 3; i++)
		psnr[i] = mse_norm_to_psnr(mse_norm[i]);

	return res;
}
