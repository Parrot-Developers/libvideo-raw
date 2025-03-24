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

#ifndef _VRAW_H_
#define _VRAW_H_

#include <stdint.h>
#include <unistd.h>

#include <video-defs/vdefs.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* To be used for all public API */
#ifdef VRAW_API_EXPORTS
#	ifdef _WIN32
#		define VRAW_API __declspec(dllexport)
#	else /* !_WIN32 */
#		define VRAW_API __attribute__((visibility("default")))
#	endif /* !_WIN32 */
#else /* !VRAW_API_EXPORTS */
#	define VRAW_API
#endif /* !VRAW_API_EXPORTS */


/* Forward declarations */
struct vraw_reader;
struct vraw_writer;


/* Frame data */
struct vraw_frame {
	/* Plane data pointers; the number of effective
	 * planes depends on the data format */
	union {
		/* To be used for example by the reader */
		uint8_t *data[VDEF_RAW_MAX_PLANE_COUNT];

		/* To be used for example by the writer */
		const uint8_t *cdata[VDEF_RAW_MAX_PLANE_COUNT];
	};

	/* Raw frame metadata */
	struct vdef_raw_frame frame;
};


/* Reader configuration */
struct vraw_reader_config {
	/* YUV4MPEG2 (*.y4m) file format (if not 0) */
	int y4m;

	/* Begin reading from a frame index (if not 0) */
	unsigned int start_index;

	/* Begin reading in reverse order (to be used with loop = -1) */
	bool start_reversed;

	/* Maximum number of frames to read (if not 0, otherwise read until
	 * the end of the file) */
	unsigned int max_count;

	/* Reading loop configuration: 0 = no loop, 1 = loop from
	 * the beginning, -1 = loop with reverse */
	int loop;

	/* Raw format (can be empty for y4m files, mandatory otherwise) */
	struct vdef_raw_format format;

	/* Format information */
	struct vdef_format_info info;

	/* Hardware constraints information */
	unsigned int plane_stride_align[VDEF_RAW_MAX_PLANE_COUNT];
	unsigned int plane_scanline_align[VDEF_RAW_MAX_PLANE_COUNT];
	unsigned int plane_size_align[VDEF_RAW_MAX_PLANE_COUNT];
};


/* Writer configuration */
struct vraw_writer_config {
	/* YUV4MPEG2 (*.y4m) file format (if not 0) */
	int y4m;

	/* Data format (mandatory) */
	struct vdef_raw_format format;

	/* Format information */
	struct vdef_format_info info;
};


/**
 * Create a file reader instance.
 * The configuration structure must be filled.
 * The instance handle is returned through the ret_obj parameter.
 * When no longer needed, the instance must be freed using the
 * vraw_reader_destroy() function.
 * @param filename: file name
 * @param config: reader configuration
 * @param ret_obj: reader instance handle (output)
 * @return 0 on success, negative errno value in case of error
 */
VRAW_API int vraw_reader_new(const char *filename,
			     const struct vraw_reader_config *config,
			     struct vraw_reader **ret_obj);


/**
 * Free a reader instance.
 * This function frees all resources associated with a reader instance.
 * @param self: reader instance handle
 * @return 0 on success, negative errno value in case of error
 */
VRAW_API int vraw_reader_destroy(struct vraw_reader *self);


/**
 * Get the reader configuration.
 * The configuration structure is filled by the function.
 * @param self: reader instance handle
 * @param config: reader configuration (output)
 * @return 0 on success, negative errno value in case of error
 */
VRAW_API int vraw_reader_get_config(struct vraw_reader *self,
				    struct vraw_reader_config *config);


/**
 * Get the minimum buffer size for reading a frame.
 * @param self: reader instance handle
 * @return buffer size on success, negative errno value in case of error
 */
VRAW_API ssize_t vraw_reader_get_min_buf_size(struct vraw_reader *self);


/**
 * Get the file frame count.
 * @param self: reader instance handle
 * @return file frame count on success, negative errno value in case of error
 */
VRAW_API ssize_t vraw_reader_get_file_frame_count(struct vraw_reader *self);


/**
 * Set the reader framerate.
 * @param self: reader instance handle
 * @param framerate: new framerate
 * @return 0 on success, negative errno value in case of error
 */
VRAW_API int vraw_reader_set_framerate(struct vraw_reader *self,
				       const struct vdef_frac *framerate);


/**
 * Read a frame.
 * Reads a frame from the file into the provided data buffer.
 * The buffer size must be large enough to hold the frame data.
 * The vraw_reader_get_min_buf_size() function can be used to get the
 * minimum required buffer size.
 * The frame structure is filled by the function with the frame metadata.
 * @param self: reader instance handle
 * @param data: pointer on the buffer to fill
 * @param len: buffer size
 * @param frame: frame metadata (output)
 * @return 0 on success, negative errno value in case of error
 */
VRAW_API int vraw_reader_frame_read(struct vraw_reader *self,
				    uint8_t *data,
				    size_t len,
				    struct vraw_frame *frame);


/**
 * Create a file writer instance.
 * The configuration structure must be filled.
 * The instance handle is returned through the ret_obj parameter.
 * When no longer needed, the instance must be freed using the
 * vraw_writer_destroy() function.
 * @param filename: file name
 * @param config: writer configuration
 * @param ret_obj: writer instance handle (output)
 * @return 0 on success, negative errno value in case of error
 */
VRAW_API int vraw_writer_new(const char *filename,
			     const struct vraw_writer_config *config,
			     struct vraw_writer **ret_obj);


/**
 * Free a writer instance.
 * This function frees all resources associated with a writer instance.
 * @param self: writer instance handle
 * @return 0 on success, negative errno value in case of error
 */
VRAW_API int vraw_writer_destroy(struct vraw_writer *self);


/**
 * Write a frame.
 * Writes a frame to the file. The profided frame structure must be filled
 * with the frame metadata.
 * @param self: writer instance handle
 * @param frame: frame metadata
 * @return 0 on success, negative errno value in case of error
 */
VRAW_API int vraw_writer_frame_write(struct vraw_writer *self,
				     const struct vraw_frame *frame);


/**
 * Compute the Peak Signal to Noise Ratio (PSNR) between 2 frames
 * @param frame1: pointer a structure containing frame1 info.
 * @param frame2: pointer a structure containing frame2 info.
 * @param psnr: array of 4 double containing the computed PSNR values.
 * @return 0 on success, negative errno value in case of error
 */
VRAW_API int vraw_compute_psnr(const struct vraw_frame *frame1,
			       const struct vraw_frame *frame2,
			       double psnr[4]);


/**
 * Read an image file.
 * Reads an image file into the provided frame structure and data buffer.
 * Only the PNG format is supported.
 * The buffer size must be large enough to hold the frame data.
 * If the function is called with a NULL data pointer, the len parameter
 * is filled with the required size for buffer allocation. The function can
 * be called first with a NULL data pointer to retrieve the required
 * allocation size, then perform the required allocation and call the
 * function again with the allocated buffer.
 * When called with a non-NULL data pointer, the value pointed by len must
 * be the data buffer size.
 * The frame structure is filled by the function with the frame metadata.
 * @param file_name: null-terminated file name string
 * @param data: pointer on the buffer to fill
 * @param len: pointer to the buffer size (input/output)
 * @param frame: frame metadata (output)
 * @return 0 on success, negative errno value in case of error
 */
VRAW_API int vraw_image_read(const char *filename,
			     uint8_t *data,
			     size_t *len,
			     struct vraw_frame *frame);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* !_VRAW_H_ */
