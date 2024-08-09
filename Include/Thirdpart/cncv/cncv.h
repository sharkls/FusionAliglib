/*************************************************************************
 * Copyright (C) [2019-2022] by Cambricon, Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *************************************************************************/
#ifndef CNCV_INCLUDE_CNCV_H_
#define CNCV_INCLUDE_CNCV_H_

/******************************************************************************
 * CNCV: Cambricon Neuware Computer Vision library
 ******************************************************************************/

#define CNCV_MAJOR 0
#define CNCV_MINOR 4
#define CNCV_PATCHLEVEL 302
#define CNCV_VERSION (CNCV_MAJOR * 1000 + CNCV_MINOR * 100 + CNCV_PATCHLEVEL)

#define CNCV_PLANE_MAX_NUM 6

#include <stdint.h>
#include "cnrt.h"

#if defined(__cplusplus)
extern "C" {
#endif

/******************************************************************************
 * CNCV Return Status
 ******************************************************************************/
//! @brief
/*! Enumeration variables describe function return status. */
typedef enum {
  CNCV_STATUS_SUCCESS                      = 0,
  //!< The called function has run successfully.
  CNCV_STATUS_NOT_INITIALIZED              = 1,
  //!< The CNCV library has not been initialized.
  CNCV_STATUS_ALLOC_FAILED                 = 2,
  //!< Memory allocation failed.
  CNCV_STATUS_BAD_PARAM                    = 3,
  //!< An illegal parameter has been passed to the function.
  CNCV_STATUS_INTERNAL_ERROR               = 4,
  //!< An internal error occurred.
  CNCV_STATUS_ARCH_MISMATCH                = 5,
  //!< Device arch is not matched.
  CNCV_STATUS_EXECUTION_FAILED             = 6,
  //!< Function execution failed.
  CNCV_STATUS_NOT_SUPPORTED                = 7,
  //!< The used parameter is not supported.
  CNCV_STATUS_NOT_IMPLEMENTED              = 8,
  //!< The called function has not been implemented.
  CNCV_STATUS_PRECISION_ERROR              = 9,
  //!< Precision overflow or underflow occurred.
} cncvStatus_t;

/******************************************************************************
 * CNCV Pixel Format
 ******************************************************************************/
//! @brief
/*! Enumeration variables describe the image pixel formats. */
typedef enum {
  CNCV_PIX_FMT_NV12         = 0,   //!< Semi-planar Y4-U1V1.
  CNCV_PIX_FMT_NV21         = 1,   //!< Semi-planar Y4-V1U1.
  CNCV_PIX_FMT_I420         = 2,   //!< Planar Y4-U1-V1.
  CNCV_PIX_FMT_YV12         = 3,   //!< Planar Y4-V1-U1.
  CNCV_PIX_FMT_YUYV         = 4,   //!< Packed Y2U1Y2V1.
  CNCV_PIX_FMT_UYVY         = 5,   //!< Packed U1Y2V1Y2.
  CNCV_PIX_FMT_YVYU         = 6,   //!< Packed Y2V1Y2U1.
  CNCV_PIX_FMT_VYUY         = 7,   //!< Packed V1Y2U1Y2.
  CNCV_PIX_FMT_P010         = 8,   //!< 10-bit semi-planar Y4-U1V1.
  CNCV_PIX_FMT_YUV420_10BIT = 9,   //!< 10-bit planar Y4-U1-V1.
  CNCV_PIX_FMT_YUV444_10BIT = 10,  //!< 10-bit planar Y4-U4-V4.
  CNCV_PIX_FMT_RGB          = 11,  //!< Packed RGB.
  CNCV_PIX_FMT_BGR          = 12,  //!< Packed BGR.
  CNCV_PIX_FMT_ARGB         = 13,  //!< Packed ARGB.
  CNCV_PIX_FMT_ABGR         = 14,  //!< Packed ABGR.
  CNCV_PIX_FMT_BGRA         = 15,  //!< Packed BGRA.
  CNCV_PIX_FMT_RGBA         = 16,  //!< Packed RGBA.
  CNCV_PIX_FMT_AYUV         = 17,  //!< Packed AYUV.
  CNCV_PIX_FMT_RGB565       = 18,  //!< 8-bit packed R5G6B5.
  CNCV_PIX_FMT_GRAY         = 19,  //!< Packed Gray.
  CNCV_PIX_FMT_LAB          = 20,  //!< Packed Lab.
  CNCV_PIX_FMT_INVALID      = 21,  //!< Invalid pixel format.
} cncvPixelFormat;

/******************************************************************************
 * CNCV Color Space
 ******************************************************************************/
//! @brief
/*! Enumeration variables describe image color spaces. */
typedef enum {
  CNCV_COLOR_SPACE_BT_709    = 0,
  //!< ITU BT.709 color space.
  CNCV_COLOR_SPACE_BT_601    = 1,
  //!< ITU BT.601 color space.
  CNCV_COLOR_SPACE_BT_2020   = 2,
  //!< ITU BT.2020 color space.
  CNCV_COLOR_SPACE_BT_601_ER = 3,
  //!< ITU BT.601 color space with extended-range values.
  CNCV_COLOR_SPACE_BT_709_ER = 4,
  //!< ITU BT.709 color space with extended-range values.
  CNCV_COLOR_SPACE_INVALID   = 5,
  //!< Invalid color space.
} cncvColorSpace;

/******************************************************************************
 * CNCV Image Depth
 ******************************************************************************/
//! @brief
/*! Enumeration variables describe image depths. */
typedef enum {
  CNCV_DEPTH_8U  = 0,          //!< Unsigned 8-bit integer.
  CNCV_DEPTH_8S  = 1,          //!< Signed 8-bit integer.
  CNCV_DEPTH_16U = 2,          //!< Unsigned 16-bit integer (reserved, unsupported currently).
  CNCV_DEPTH_16S = 3,          //!< Signed 16-bit integer.
  CNCV_DEPTH_32U = 4,          //!< Unsigned 32-bit integer (reserved, unsupported currently).
  CNCV_DEPTH_32S = 5,          //!< Signed 32-bit integer.
  CNCV_DEPTH_16F = 6,          //!< 16-bit floating point.
  CNCV_DEPTH_32F = 7,          //!< 32-bit floating point.
  CNCV_DEPTH_INVALID = 8,  //!< Invalid image depth.
} cncvDepth_t;
//! @brief
/*! Enumeration variables describe interpolation methods. */
typedef enum {
  CNCV_INTER_BILINEAR   = 0,          //!< Bilinear interpolation.
} cncvInterpolation;

/*! @brief
 *
 * Enumeration variables describe the threshold types that are used in the
 * implementation of the Threshold function.
 *
 */
typedef enum {
  CNCV_THRESH_BINARY     = 0,  //!< The binary threshold type.
  CNCV_THRESH_BINARY_INV = 1,  //!< The binary inv threshold type.
} cncvThreshType;

/*!
 * @brief
 *
 * Enumeration variables describe the kernel shapes that are used in the
 * implementation of the Dilate Erode function.
 *
 */
typedef enum {
  CNCV_DILATE_ERODE_RECT    = 0,  //!< The rectangular kernel shape.
  CNCV_DILATE_ERODE_CROSS   = 1,  //!< The cross kernel shape.
  CNCV_DILATE_ERODE_ELLIPSE = 2,  //!< The elliptic kernel shape.
} cncvDilateErodeShape_t;

/*!
 * @brief
 *
 * Enumeration variables describe the compute modes that are used in the
 * implementation of the Dilate Erode function.
 *
 */
typedef enum {
  CNCV_DILATE_OP = 0,  //!< The dilate compute mode.
  CNCV_ERODE_OP  = 1,  //!< The erode compute mode.
} cncvDilateErodeType_t;


/*!
 * @struct cncvContext
 * @brief
 *
 * The cncvContext is a struct describing the CNCV library context.
 *
 */
struct cncvContext;

/*!
 *
 * cncvHandle_t is a pointer to the struct that holds the CNCV library context.
 * The CNCV library context must be created by using cncvCreate(), and the returned
 * handle must be passed to all the subsequent function calls. Destroy
 * the context at the end by using cncvDestroy().
 *
 */
typedef struct cncvContext *cncvHandle_t;

/*!
 * @struct cncvRect
 * @brief
 *
 * This struct describes the bounding box of ROI,
 * which means all bounding boxes are described in terms of (x, y) + (w, h).
 *
 */
typedef struct cncvRect {
  uint32_t x;  //!< The x-coordinate of ROI.
  uint32_t y;  //!< The y-coordinate of ROI.
  uint32_t w;  //!< The width of ROI.
  uint32_t h;  //!< The height of ROI.
} cncvRect;

/*!
 *
 * cncvRect_t is a pointer to the struct that holds the bounding box of ROI.
 *
 */
typedef struct cncvRect *cncvRect_t;

/*!
 * @struct cncvImageDescriptor
 * @brief
 *
 * This struct describes the CNCV input/output image data.
 *
 */
typedef struct cncvImageDescriptor {
  uint32_t width;      //!< The width of an image.
  uint32_t height;     //!< The height of an image.
  uint32_t stride[CNCV_PLANE_MAX_NUM];  //!< The stride of an image.
  cncvPixelFormat pixel_fmt;      //!< The pixel format of an image.
  cncvColorSpace color_space;     //!< The color space of an image.
  cncvDepth_t depth;              //!< The depth of an image.
} cncvImageDescriptor;

/*!
 *
 * cncvImageDescriptor_t is a pointer to the struct that holds the CNCV
 * input/output image data.
 *
 */
typedef struct cncvImageDescriptor *cncvImageDescriptor_t;

/******************************************************************************
 * CNCV PadType
 ******************************************************************************/
//! @brief
/*! Enumeration variables describe image PadTypes. */
//! Various pad types and image boundaries are denoted with `|`.
typedef enum {
  CNCV_PAD_CONSTANT    = 0,  //!< `iiiiii|abcdefgh|iiiiiii`  with some specified `i`s.
  CNCV_PAD_REPLICATE   = 1,  //!< `aaaaaa|abcdefgh|hhhhhhh`.
  CNCV_PAD_REFLECT     = 2,  //!< `fedcba|abcdefgh|hgfedcb`.
  CNCV_PAD_WRAP        = 3,  //!< `cdefgh|abcdefgh|abcdefg`  use other side to pad.
  CNCV_PAD_REFLECT_101 = 4,  //!< `gfedcb|abcdefgh|gfedcba`.
  CNCV_PAD_TRANSPARENT = 5,  //!< `uvwxyz|abcdefgh|ijklmno`.
  CNCV_PAD_REFLECT101  = CNCV_PAD_REFLECT_101,  //!< Same as BORDER_REFLECT_101.
  CNCV_PAD_DEFAULT     = CNCV_PAD_REFLECT_101,  //!< Same as BORDER_REFLECT_101.
  CNCV_PAD_ISOLATED    = 16                     //!< Do not look outside of ROI.
} cncvPadType;


/******************************************************************************
 * CNCV Runtime Management
 ******************************************************************************/
/*!
 *  @brief
 *
 * This function initializes the CNCV library and creates a handle to the struct
 * that holds the CNCV library context.
 *
 * @param[out] handle
 *   Output. A pointer to the allocated CNCV handle.
 * @retval CNCV_STATUS_SUCCESS
 *    The function ended successfully.
 * @retval CNCV_STATUS_BAD_PARAM
 *   One of the following conditions are met:
 *   - The handle was empty.
 *   - The queue was empty.
 * @note
 * - It allocates hardware resources on the host and device.
 * - You need to call this function before any other CNCV function is called.
 * - You need to call the cncvDestroy() function to release the resources later.
 * @par Requirements
 * - None.
 * @par Example
 * - None.
 *
 */
cncvStatus_t cncvCreate(cncvHandle_t *handle);

/*!
 * @brief
 *
 * This function releases the resources of the specified CNCV handle that was
 * created by the cncvCreate() function.
 *
 * @param[in] handle
 *   Input. A pointer to the CNCV handle to be destroyed.
 * @retval CNCV_STATUS_SUCCESS
 *   The CNCV context was successfully released.
 * @retval CNCV_STATUS_BAD_PARAM
 *   One of the following conditions are met:
 *   - The handle was empty.
 *   - The queue was empty.
 * @note
 * - It is usually the last call to destroy the CNCV handle.
 * @par Requirements
 * - None.
 * @par Example
 * - None.
 *
 */
cncvStatus_t cncvDestroy(cncvHandle_t handle);

/*!
 * @brief
 *
 * This function sets the runtime queue in the handle. The queue is used to
 * launch kernels or to synchronize them to this queue.
 *
 * @param[in] handle
 *   Input. A pointer to the CNCV handle.
 * @param[in] queue
 *   Input. The CNRT queue to be set to the CNCV handle.
 * @retval CNCV_STATUS_SUCCESS
 *   The CNRT queue was set successfully.
 * @retval CNCV_STATUS_BAD_PARAM
 *   One of the following conditions are met:
 *   - The handle was empty.
 *   - The queue was empty.
 * @note
 * - Before setting a queue, you need to call the cncvCreate() function to initialize
 *   the CNCV library and call the cnrtCreateQueue() function to create a queue.
 * @par Requirements
 * - None.
 * @par Example
 * - None.
 *
 */
cncvStatus_t cncvSetQueue(cncvHandle_t handle, cnrtQueue_t queue);

/*!
 * @brief
 *
 * This function retrieves the queue that was previously set to the handle.
 *
 * @param[in] handle
 *   Input. A pointer to the CNCV handle.
 * @param[out] queue
 *   Output. A pointer to the CNRT queue that was set to the specified handle.
 * @retval CNCV_STATUS_SUCCESS
 *   The CNRT queue was returned successfully.
 * @retval CNCV_STATUS_BAD_PARAM
 *   One of the following conditions are met:
 *   - The handle was empty.
 *   - The queue was empty.
 * @note
 * - None.
 * @par Requirements
 * - None.
 * @par Example
 * - None.
 *
 */
cncvStatus_t cncvGetQueue(cncvHandle_t handle, cnrtQueue_t *queue);

/*!
 * @brief
 *
 * This function syncs the queue in handle.
 *
 * @param[in] handle
 *   Input. A pointer to the CNCV handle.
 * @retval CNCV_STATUS_SUCCESS
 *   The CNRT queue was returned successfully.
 * @retval CNCV_STATUS_BAD_PARAM
 *   One of the following conditions are met:
 *   - The handle was empty.
 *   - The queue was empty.
 * @note
 * - None.
 * @par Requirements
 * - None.
 * @par Example
 * - None.
 *
 */
cncvStatus_t cncvSyncQueue(cncvHandle_t handle);

/*!
 * @brief
 *
 * This function converts the CNCV enumerated status code to ASCIIZ static string.
 *
 * @param[in] status
 *   Input. The CNCV enumerated status code.
 * @return
 *   A pointer to ASCIIZ static string with the status name.
 * @note
 * - None.
 * @par Requirements
 * - None.
 * @par Example
 * - None.
 *
 */
const char *cncvGetErrorString(cncvStatus_t status);

/*!
 * @brief
 *
 * This function is used for the ``CNCV_CHECK()`` macro, which checks the
 * cncvStatus_t value and prints the error information.
 *
 * @param[in] result
 *   Input. The returned status enum by CNCV library functions.
 * @param[in] func
 *   Input. The name of CNCV function to be checked.
 * @param[in] file
 *   Input. The ``__FILE__`` macro.
 * @param[in] line
 *   Input. The ``__LINE__`` macro.
 * @return void.
 * @note
 * - None.
 * @par Requirements
 * - None.
 * @par Example
 * - None.
 *
 */
void cncvCheck(cncvStatus_t result, char const *const func, const char *const file, int const line);

#define CNCV_CHECK(val) cncvCheck((val), #val, __FILE__, __LINE__)

/*!
 * @brief
 *
 * This function retrieves the version of the CNCV library. This function is deprecated, and it is recommended to use ::cncvGetLibVersion instead.
 *
 * @return The version of the CNCV library.
 * @note
 * - None.
 * @par Requirements
 * - None.
 * @par Example
 * - None.
 *
 */
size_t cncvGetVersion(void);

/*!
 * @brief
 *
 * This function retrieves the version of the CNCV library.
 *
 * @param[in] major
 *   Input. A pointer to the scale factor that gets the major version of the CNCV library.
 * @param[in] minor
 *   Input. A pointer to the scale factor that gets the minor version of the CNCV library.
 * @param[in] patch
 *   Input. A pointer to the scale factor that gets the patch version of the CNCV library.
 *
 * @return The version of the CNCV library.
 * @note
 * - None.
 * @par Requirements
 * - None.
 * @par Example
 * - None.
 *
 */
void cncvGetLibVersion(int* major,
                       int* minor,
                       int* patch);

/*********************************************************************
* Arithmetic and Logical Operations
********************************************************************/
/*!
 * @brief
 *
 * This function performs the add operation.
 *
 * @param[in] handle
 *   Input. Pointer to the CNCV context.
 * @param[in] lhs_desc
 *   Input. Description (width, height, stride, pixel_fmt, color_space, depth) of the lhs input data.
 *   It has the following limitations:
 *   - 0 < height <= 8192, 0 < width <= 8192.
 *   - stride >= width * channel_num * sizeof(dataType).
 *   - The member depth now supports CNCV_DEPTH_8U and CNCV_DEPTH_32F.
 *   - The member pixel_fmt now supports CNCV_PIX_FMT_RGB, CNCV_PIX_FMT_BGR,
 *                                      CNCV_PIX_FMT_RGBA, CNCV_PIX_FMT_BGRA,
 *                                      CNCV_PIX_FMT_ARGB, CNCV_PIX_FMT_ABGR, and 
 *                                      CNCV_PIX_FMT_GRAY.
 * @param[in] lhs
 *   Input. Pointer to source lhs data on the MLU.
 * @param[in] rhs_desc
 *   Input. Description (width, height, stride, pixel_fmt, color_space, depth) of the rhs input data.
 *   It has the following limitations:
 *   - 0 < height <= 8192, 0 < width <= 8192.
 *   - stride >= width * channel_num * sizeof(dataType).
 *   - The member depth now supports CNCV_DEPTH_8U and CNCV_DEPTH_32F.
 *   - The member pixel_fmt now supports CNCV_PIX_FMT_RGB, CNCV_PIX_FMT_BGR,
 *                                      CNCV_PIX_FMT_RGBA, CNCV_PIX_FMT_BGRA,
 *                                      CNCV_PIX_FMT_ARGB, CNCV_PIX_FMT_ABGR, and 
 *                                      CNCV_PIX_FMT_GRAY.
 * @param[in] rhs
 *   Input. Pointer to source rhs data on MLU.
 * @param[in] mask_desc
 *   Input. Description (width, height, stride, pixel_fmt, color_space, depth) of input mask data.
 *   It has the following limitations:
 *   - 0 < height <= 8192, 0 < width <= 8192.
 *   - stride >= width * channel_num * sizeof(dataType).
 *   - The member depth now supports CNCV_DEPTH_8U and CNCV_DEPTH_32F.
 *   - The member pixel_fmt now supports CNCV_PIX_FMT_RGB, CNCV_PIX_FMT_BGR,
 *                                      CNCV_PIX_FMT_RGBA, CNCV_PIX_FMT_BGRA,
 *                                      CNCV_PIX_FMT_ARGB, CNCV_PIX_FMT_ABGR, and 
 *                                      CNCV_PIX_FMT_GRAY.
 * @param[in] mask
 *   Input. Pointer to source mask data on the MLU.
 *          When mask is not zero, the corresponding output will be changed.
 * @param[in] dst_desc
 *   Output. Description (width, height, stride, pixel_fmt, color_space, depth) of output destination data.
 *   It has the following limitations:
 *   - 0 < height <= 8192, 0 < width <= 8192.
 *   - stride >= width * channel_num * sizeof(dataType).
 *   - The member depth now supports CNCV_DEPTH_8U and CNCV_DEPTH_32F.
 *   - The member pixel_fmt now supports CNCV_PIX_FMT_RGB, CNCV_PIX_FMT_BGR,
 *                                      CNCV_PIX_FMT_RGBA, CNCV_PIX_FMT_BGRA,
 *                                      CNCV_PIX_FMT_ARGB, CNCV_PIX_FMT_ABGR, and 
 *                                      CNCV_PIX_FMT_GRAY.
 * @param[out] dst
 *   Output. Pointer to output destination data on the MLU.
 * @retval CNCV_STATUS_SUCCESS
 *   The function ends normally.
 * @note
 * - lhs stands for the first image data used in computing.
 * - rhs stands for the second image data used in computing.
 * - mask stands for the third image data used in computing.
 * - dst stands for the final result after computing.
 * - This function only supports one batch.
 * @par Requirements
 * - None.
 * @par Example
 * - None.
 *
 */
cncvStatus_t cncvAdd(cncvHandle_t handle,
                     const cncvImageDescriptor lhs_desc,
                     void *lhs,
                     const cncvImageDescriptor rhs_desc,
                     void *rhs,
                     const cncvImageDescriptor mask_desc,
                     void *mask,
                     const cncvImageDescriptor dst_desc,
                     void *dst);

/*!
 * @brief
 *
 * This function performs the add_weighted operation.
 *
 * @param[in] handle
 *   Input. Pointer to the CNCV context.
 * @param[in] lhs_desc
 *   Input. Description (width, height, stride, pixel_fmt, color_space, depth) of the lhs input data.
 *   It has the following limitations:
 *   - 0 < height <= 8192, 0 < width <= 8192.
 *   - stride >= width * channel_num * sizeof(dataType).
 *   - The member depth now supports CNCV_DEPTH_8U and CNCV_DEPTH_32F.
 *   - The member pixel_fmt now supports CNCV_PIX_FMT_RGB, CNCV_PIX_FMT_BGR,
 *                                      CNCV_PIX_FMT_RGBA, CNCV_PIX_FMT_BGRA,
 *                                      CNCV_PIX_FMT_ARGB, CNCV_PIX_FMT_ABGR, and 
 *                                      CNCV_PIX_FMT_GRAY.
 * @param[in] lhs
 *   Input. Pointer to source lhs data on the MLU.
 *   Input. The additional parameter used as the coefficient of alpha * lhs.
 * @param[in] rhs_desc
 *   Input. Description (width, height, stride, pixel_fmt, color_space, depth) of the rhs input data.
 *   It has the following limitations:
 *   - 0 < height <= 8192, 0 < width <= 8192.
 *   - stride >= width * channel_num * sizeof(dataType).
 *   - The member depth now supports CNCV_DEPTH_8U and CNCV_DEPTH_32F.
 *   - The member pixel_fmt now supports CNCV_PIX_FMT_RGB, CNCV_PIX_FMT_BGR,
 *                                      CNCV_PIX_FMT_RGBA, CNCV_PIX_FMT_BGRA,
 *                                      CNCV_PIX_FMT_ARGB, CNCV_PIX_FMT_ABGR, and 
 *                                      CNCV_PIX_FMT_GRAY.
 * @param[in] rhs
 *   Input. Pointer to source rhs data on the MLU.
 *   Input. The additional parameter used as the coefficient of beta * rhs.
 * @param[in] gamma
 *   Input. The additional parameter used as a bias.
 * @param[in] dst_desc
 *   Output. Description (width, height, stride, pixel_fmt, color_space, depth) of output destination data.
 *   It has the following limitations:
 *   - 0 < height <= 8192, 0 < width <= 8192.
 *   - stride >= width * channel_num * sizeof(dataType).
 *   - The member depth now supports CNCV_DEPTH_8U and CNCV_DEPTH_32F.
 *   - The member pixel_fmt now supports CNCV_PIX_FMT_RGB, CNCV_PIX_FMT_BGR,
 *                                      CNCV_PIX_FMT_RGBA, CNCV_PIX_FMT_BGRA,
 *                                      CNCV_PIX_FMT_ARGB, CNCV_PIX_FMT_ABGR, and 
 *                                      CNCV_PIX_FMT_GRAY.
 * @param[out] dst
 *   Output. Pointer to output destination data on the MLU.
 * @retval CNCV_STATUS_SUCCESS
 *   The function ends normally.
 * @note
 * - lhs stands for the first image data used in computing.
 * - rhs stands for the second image data used in computing.
 * - dst stands for the final result after computing.
 * - alpha and beta stand for the coefficients used in computing.
 * - gamma stands for a bias used in computing.
 * - This function only supports one batch.
 * @par Requirements
 * - None.
 * @par Example
 * - None.
 *
 */
cncvStatus_t cncvAddWeighted(cncvHandle_t handle,
                             const cncvImageDescriptor lhs_desc,
                             void *lhs,
                             const float alpha,
                             const cncvImageDescriptor rhs_desc,
                             void *rhs,
                             const float beta,
                             const float gamma,
                             const cncvImageDescriptor dst_desc,
                             void *dst);

/*!
 * @brief
 *
 * This function performs the scale add operation.
 *
 * @param[in] handle
 *   Input. Pointer to the CNCV context.
 * @param[in] lhs_desc
 *   Input. Description (width, height, stride, pixel_fmt, color_space, depth) of the lhs input data.
 *   It has the following limitations:
 *   - 0 < height <= 8192, 0 < width <= 8192.
 *   - stride >= width * channel_num * sizeof(dataType).
 *   - The member depth now supports CNCV_DEPTH_8U and CNCV_DEPTH_32F.
 *   - The member pixel_fmt now supports CNCV_PIX_FMT_RGB, CNCV_PIX_FMT_BGR,
 *                                      CNCV_PIX_FMT_RGBA, CNCV_PIX_FMT_BGRA,
 *                                      CNCV_PIX_FMT_ARGB, CNCV_PIX_FMT_ABGR, and 
 *                                      CNCV_PIX_FMT_GRAY.
 * @param[in] lhs
 *   Input. Pointer to source lhs data on the MLU.
 * @param[in] alpha
 *   Input. The additional parameter used as the coefficient of alpha * lhs.
 * @param[in] rhs_desc
 *   Input. Description (width, height, stride, pixel_fmt, color_space, depth) of the rhs input data.
 *   It has the following limitations:
 *   - 0 < height <= 8192, 0 < width <= 8192.
 *   - stride >= width * channel_num * sizeof(dataType).
 *   - The member depth now supports CNCV_DEPTH_8U and CNCV_DEPTH_32F.
 *   - The member pixel_fmt now supports CNCV_PIX_FMT_RGB, CNCV_PIX_FMT_BGR,
 *                                      CNCV_PIX_FMT_RGBA, CNCV_PIX_FMT_BGRA,
 *                                      CNCV_PIX_FMT_ARGB, CNCV_PIX_FMT_ABGR, and 
 *                                      CNCV_PIX_FMT_GRAY.
 * @param[in] rhs
 *   Input. Pointer to source rhs data on the MLU.
 * @param[in] dst_desc
 *   Output. Description (width, height, stride, pixel_fmt, color_space, depth) of output destination data.
 *   It has the following limitations:
 *   - stride >= width * channel_num * sizeof(dataType).
 *   - The member depth now supports CNCV_DEPTH_8U and CNCV_DEPTH_32F.
 *   - The member pixel_fmt now supports CNCV_PIX_FMT_RGB, CNCV_PIX_FMT_BGR,
 *                                      CNCV_PIX_FMT_RGBA, CNCV_PIX_FMT_BGRA,
 *                                      CNCV_PIX_FMT_ARGB, CNCV_PIX_FMT_ABGR, and 
 *                                      CNCV_PIX_FMT_GRAY.
 * @param[out] dst
 *   Output. Pointer to output destination data on the MLU.
 * @retval CNCV_STATUS_SUCCESS
 *   The function ends normally.
 * @note
 * - lhs stands for the first image data used in computing.
 * - rhs stands for the second image data used in computing.
 * - dst stands for the final result after computing.
 * - This function only supports one batch.
 * @par Requirements
 * - None.
 * @par Example
 * - None.
 *
 */
cncvStatus_t cncvScaleAdd(cncvHandle_t handle,
                          const cncvImageDescriptor lhs_desc,
                          void *lhs,
                          const float alpha,
                          const cncvImageDescriptor rhs_desc,
                          void *rhs,
                          const cncvImageDescriptor dst_desc,
                          void *dst);

/*!
 * @brief
 *
 * This function performs the multiply operation.
 *
 * @param[in] handle
 *   Input. Pointer to the CNCV context.
 * @param[in] lhs_desc
 *   Input. Description (width, height, stride, pixel_fmt, color_space, depth) of the lhs input data.
 *   It has the following limitations:
 *   - 0 < height <= 8192, 0 < width <= 8192.
 *   - stride >= width * channel_num * sizeof(dataType).
 *   - The member depth now supports CNCV_DEPTH_8U and CNCV_DEPTH_32F.
 *   - The member pixel_fmt now supports CNCV_PIX_FMT_RGB, CNCV_PIX_FMT_BGR,
 *                                      CNCV_PIX_FMT_RGBA, CNCV_PIX_FMT_BGRA,
 *                                      CNCV_PIX_FMT_ARGB, CNCV_PIX_FMT_ABGR, and 
 *                                      CNCV_PIX_FMT_GRAY.
 * @param[in] lhs
 *   Input. Pointer to source lhs data on the MLU.
 * @param[in] alpha
 *   Input. The additional parameter used as the coefficient of alpha * lhs.
 * @param[in] rhs_desc
 *   Input. Description (width, height, stride, pixel_fmt, color_space, depth) of the rhs input data.
 *   It has the following limitations:
 *   - 0 < height <= 8192, 0 < width <= 8192.
 *   - stride >= width * channel_num * sizeof(dataType).
 *   - The member depth now supports CNCV_DEPTH_8U and CNCV_DEPTH_32F.
 *   - The member pixel_fmt now supports CNCV_PIX_FMT_RGB, CNCV_PIX_FMT_BGR,
 *                                      CNCV_PIX_FMT_RGBA, CNCV_PIX_FMT_BGRA,
 *                                      CNCV_PIX_FMT_ARGB, CNCV_PIX_FMT_ABGR, and 
 *                                      CNCV_PIX_FMT_GRAY.
 * @param[in] rhs
 *   Input. Pointer to source rhs data on the MLU.
 * @param[in] dst_desc
 *   Output. Description (width, height, stride, pixel_fmt, color_space, depth) of output destination data.
 *   It has the following limitations:
 *   - stride >= width * channel_num * sizeof(dataType).
 *   - The member depth now supports CNCV_DEPTH_8U and CNCV_DEPTH_32F.
 *   - The member pixel_fmt now supports CNCV_PIX_FMT_RGB, CNCV_PIX_FMT_BGR,
 *                                      CNCV_PIX_FMT_RGBA, CNCV_PIX_FMT_BGRA,
 *                                      CNCV_PIX_FMT_ARGB, CNCV_PIX_FMT_ABGR, and 
 *                                      CNCV_PIX_FMT_GRAY.
 * @param[out] dst
 *   Output. Pointer to output destination data on the MLU.
 * @retval CNCV_STATUS_SUCCESS
 *   The function ends normally.
 * @note
 * - lhs stands for the first image data used in computing.
 * - rhs stands for the second image data used in computing.
 * - dst stands for the final result after computing.
 * - This function only supports one batch.
 * @par Requirements
 * - None.
 * @par Example
 * - None.
 *
 */
cncvStatus_t cncvMultiply(cncvHandle_t handle,
                          const cncvImageDescriptor lhs_desc,
                          void *lhs,
                          const float alpha,
                          const cncvImageDescriptor rhs_desc,
                          void *rhs,
                          const cncvImageDescriptor dst_desc,
                          void *dst);

/*!
 * @brief
 *
 * This function performs the subtract operation.
 *
 * @param[in] handle
 *   Input. Pointer to the CNCV context.
 * @param[in] lhs_desc
 *   Input. Description (width, height, stride, pixel_fmt, color_space, depth) of the lhs input data.
 *   It has the following limitations:
 *   - 0 < height <= 8192, 0 < width <= 8192.
 *   - stride >= width * channel_num * sizeof(dataType).
 *   - The member depth now supports CNCV_DEPTH_8U and CNCV_DEPTH_32F.
 *   - The member pixel_fmt now supports CNCV_PIX_FMT_RGB, CNCV_PIX_FMT_BGR,
 *                                      CNCV_PIX_FMT_RGBA, CNCV_PIX_FMT_BGRA,
 *                                      CNCV_PIX_FMT_ARGB, CNCV_PIX_FMT_ABGR, and 
 *                                      CNCV_PIX_FMT_GRAY.
 * @param[in] lhs
 *   Input. Pointer to source lhs data on the MLU.
 * @param[in] rhs_desc
 *   Input. Description (width, height, stride, pixel_fmt, color_space, depth) of the rhs input data.
 *   It has the following limitations:
 *   - 0 < height <= 8192, 0 < width <= 8192.
 *   - stride >= width * channel_num * sizeof(dataType).
 *   - The member depth now supports CNCV_DEPTH_8U and CNCV_DEPTH_32F.
 *   - The member pixel_fmt now supports CNCV_PIX_FMT_RGB, CNCV_PIX_FMT_BGR,
 *                                      CNCV_PIX_FMT_RGBA, CNCV_PIX_FMT_BGRA,
 *                                      CNCV_PIX_FMT_ARGB, CNCV_PIX_FMT_ABGR, and 
 *                                      CNCV_PIX_FMT_GRAY.
 * @param[in] rhs
 *   Input. Pointer to source rhs data on the MLU.
 * @param[in] mask_desc
 *   Input. Description (width, height, stride, pixel_fmt, color_space, depth) of input mask data.
 *   It has the following limitations:
 *   - 0 < height <= 8192, 0 < width <= 8192.
 *   - stride >= width * channel_num * sizeof(dataType).
 *   - The member depth now supports CNCV_DEPTH_8U and CNCV_DEPTH_32F.
 *   - The member pixel_fmt now supports CNCV_PIX_FMT_RGB, CNCV_PIX_FMT_BGR,
 *                                      CNCV_PIX_FMT_RGBA, CNCV_PIX_FMT_BGRA,
 *                                      CNCV_PIX_FMT_ARGB, CNCV_PIX_FMT_ABGR, and 
 *                                      CNCV_PIX_FMT_GRAY.
 * @param[in] mask
 *   Input. Pointer to source mask data on the MLU.
 *          When mask is not zero, the corresponding output will be changed.
 * @param[in] dst_desc
 *   Output. Description (width, height, stride, pixel_fmt, color_space, depth) of output destination data.
 *   It has the following limitations:
 *   - 0 < height <= 8192, 0 < width <= 8192.
 *   - stride >= width * channel_num * sizeof(dataType).
 *   - The member depth now supports CNCV_DEPTH_8U and CNCV_DEPTH_32F.
 *   - The member pixel_fmt now supports CNCV_PIX_FMT_RGB, CNCV_PIX_FMT_BGR,
 *                                      CNCV_PIX_FMT_RGBA, CNCV_PIX_FMT_BGRA,
 *                                      CNCV_PIX_FMT_ARGB, CNCV_PIX_FMT_ABGR, and 
 *                                      CNCV_PIX_FMT_GRAY.
 * @param[out] dst
 *   Output. Pointer to output destination data on the MLU.
 * @retval CNCV_STATUS_SUCCESS
 *   The function ends normally.
 * @note
 * - lhs stands for the first image data used in computing.
 * - rhs stands for the second image data used in computing.
 * - mask stands for the third image data used in computing.
 * - dst stands for the final result after computing.
 * - This function only supports one batch.
 * @par Requirements
 * - None.
 * @par Example
 * - None.
 *
 */
cncvStatus_t cncvSubtract(cncvHandle_t handle,
                          const cncvImageDescriptor lhs_desc,
                          void *lhs,
                          const cncvImageDescriptor rhs_desc,
                          void *rhs,
                          const cncvImageDescriptor mask_desc,
                          void *mask,
                          const cncvImageDescriptor dst_desc,
                          void *dst);

/*!
 * @brief
 *
 * This function performs the Divide operation between two images, supporting
 * CNCV_DEPTH_8U, CNCV_DEPTH_32F data and Batch.
 *
 * @param[in] handle
 *   Input. Pointer to the CNCV context.
 * @param[in] batch_size
 *   Input. Number of images in batch.
 * @param[in] scale
 *   Input. The scalar factor.
 * @param[in] lhs_desc
 *   Input. Description (w, h, stride) of the first input image data.
 * @param[in] lhs
 *   Input. Pointer to the first input image data on the MLU.
 * @param[in] rhs_desc
 *   Input. Description (w, h, stride) of the second input image data.
 * @param[in] rhs
 *   Input. Pointer to the second input image data on the MLU.
 * @param[in] dst_desc
 *   Input. Description (w, h, stride) of output image data.
 * @param[out] dst
 *   Output. Pointer to the MLU space for storing output data.
 * @retval CNCV_STATUS_SUCCESS
 *   The function ends normally.
 * @note
 * - lhs stands for the first image data used in computing.
 * - rhs stands for the second image data used in computing.
 * - dst stands for the final result after computing.
 * @par Requirements
 * - None.
 * @par Example
 * - None.
 *
 */
cncvStatus_t cncvDivide(cncvHandle_t handle,
                        const uint32_t batch_size,
                        float scale,
                        const cncvImageDescriptor lhs_desc,
                        void **lhs,
                        const cncvImageDescriptor rhs_desc,
                        void **rhs,
                        const cncvImageDescriptor dst_desc,
                        void **dst);

/*!
 * @brief
 *
 * This function performs the Absdiff operation.
 *
 * @param[in] handle
 *   Input. Pointer to the CNCV context.
 * @param[in] src1_desc
 *   Input. Description (width, height, stride, pixel_fmt, depth) of input src1 data.
 *   Input. Pixel format of the src1 data.
 *          It now supports CNCV_PIX_FMT_RGB, CNCV_PIX_FMT_BGR,
 *                           CNCV_PIX_FMT_RGBA, CNCV_PIX_FMT_BGRA,
 *                           CNCV_PIX_FMT_ARGB, CNCV_PIX_FMT_ABGR, and 
 *                           CNCV_PIX_FMT_GRAY.
 *   Input. Pixel depth of the src1 image.
 *          It now supports CNCV_DEPTH_8U and CNCV_DEPTH_32F.
 *   Input. It has the following limitations:
 *          - 0 < height <= 8192, 0 < width <= 8192.
 *          - stride >= width * channel_num * sizeof(dataType).
 * @param[in] src1
 *   Input. Pointer to source src1 data on the MLU.
 * @param[in] src2_desc
 *   Input. Description (width, height, stride, pixel_fmt, depth) of input src2 data.
 *   Input. Pixel format of the src2 data.
 *          It now supports CNCV_PIX_FMT_RGB and CNCV_PIX_FMT_BGR,
 *                           CNCV_PIX_FMT_RGBA, CNCV_PIX_FMT_BGRA,
 *                           CNCV_PIX_FMT_ARGB, CNCV_PIX_FMT_ABGR, and 
 *                           CNCV_PIX_FMT_GRAY.
 *   Input. Pixel depth of the src2 image.
 *          It now supports CNCV_DEPTH_8U and CNCV_DEPTH_32F.
 *   Input. It has the following limitations:
 *          - 0 < height <= 8192, 0 < width <= 8192.
 *          - stride >= width * channel_num * sizeof(dataType).
 * @param[in] src2
 *   Input. Pointer to source src2 data on the MLU.
 * @param[in] dst_desc
 *   Input. Description (width, height, stride, pixel_fmt, depth) of output destination data.
 *   Input. Pixel format of the destination data.
 *          It now supports CNCV_PIX_FMT_RGB and CNCV_PIX_FMT_BGR,
 *                           CNCV_PIX_FMT_RGBA, CNCV_PIX_FMT_BGRA,
 *                           CNCV_PIX_FMT_ARGB, CNCV_PIX_FMT_ABGR, and 
 *                           CNCV_PIX_FMT_GRAY.
 *   Input. Pixel depth of the destination image.
 *          It now supports CNCV_DEPTH_8U and CNCV_DEPTH_32F.
 *   Input. It has the following limitations:
 *          - 0 < height <= 8192, 0 < width <= 8192.
 *          - stride >= width * channel_num * sizeof(dataType).
 * @param[out] dst
 *   Output. Pointer to output destination data on the MLU.
 * @retval CNCV_STATUS_SUCCESS
 *   The function ends normally.
 * @note
 * - src1 stands for the first image data used in computing.
 * - src2 stands for the second image data used in computing.
 * - dst stands for the final result after computing.
 * - This function only supports one batch.
 * @par Requirements
 * - None.
 * @par Example
 * - None.
 *
 */
cncvStatus_t cncvAbsdiff(cncvHandle_t handle,
                           const cncvImageDescriptor src1_desc,
                           void *src1,
                           const cncvImageDescriptor src2_desc,
                           void *src2,
                           const cncvImageDescriptor dst_desc,
                           void *dst);

/*!
 * @brief
 *
 * This function performs the BitwiseAnd operation.
 *
 * @param[in] handle
 *   Input. Pointer to the CNCV context.
 * @param[in] src1_desc
 *   Input. Description (width, height, stride, pixel_fmt, depth) of input src1 data.
 *   Input. Pixel format of the src1 data.
 *          It now supports CNCV_PIX_FMT_RGB and CNCV_PIX_FMT_BGR,
 *                           CNCV_PIX_FMT_RGBA, CNCV_PIX_FMT_BGRA,
 *                           CNCV_PIX_FMT_ARGB, CNCV_PIX_FMT_ABGR, and 
 *                           CNCV_PIX_FMT_GRAY.
 *   Input. Pixel depth of the src1 image.
 *          It now supports CNCV_DEPTH_8U.
 *   Input. It has the following limitations:
 *          - 0 < height <= 8192, 0 < width <= 8192.
 *          - stride >= width * channel_num * sizeof(dataType).
 * @param[in] src1
 *   Input. Pointer to source src1 data on the MLU.
 * @param[in] src2_desc
 *   Input. Description (width, height, stride, pixel_fmt, depth) of input src2 data.
 *   Input. Pixel format of the src2 data.
 *          It now supports CNCV_PIX_FMT_RGB, CNCV_PIX_FMT_BGR,
 *                           CNCV_PIX_FMT_RGBA, CNCV_PIX_FMT_BGRA,
 *                           CNCV_PIX_FMT_ARGB, CNCV_PIX_FMT_ABGR, and 
 *                           CNCV_PIX_FMT_GRAY.
 *   Input. Pixel depth of the scr2 image.
 *          It now supports CNCV_DEPTH_8U.
 *   Input. It has the following limitations:
 *          - 0 < height <= 8192, 0 < width <= 8192.
 *          - stride >= width * channel_num * sizeof(dataType).
 * @param[in] scr2
 *   Input. Pointer to source src2 data on the MLU.
 * @param[in] mask_desc
 *   Input. Description (width, height, stride, pixel_fmt, depth) of input mask data.
 * @param[in] mask_pixfmt
 *   Input. Pixel format of the mask data.
 *          It now supports CNCV_PIX_FMT_RGB, CNCV_PIX_FMT_BGR,
 *                           CNCV_PIX_FMT_RGBA, CNCV_PIX_FMT_BGRA,
 *                           CNCV_PIX_FMT_ARGB, CNCV_PIX_FMT_ABGR, and 
 *                           CNCV_PIX_FMT_GRAY.
 *   Input. Pixel depth of the mask image.
 *          It now supports CNCV_DEPTH_8U.
 *   Input. It has the following limitations:
 *          - 0 < height <= 8192, 0 < width <= 8192.
 *          - stride >= width * channel_num * sizeof(dataType).
 * @param[in] mask
 *   Input. Pointer to source mask data on the MLU. Mask is only used in
 *          CNCV_BINARY_OP_ADD, CNCV_BINARY_OP_SUBTRACT and CNCV_BINARY_OP_BITWISE_AND.
 *          When mask is not zero, the corresponding output will be changed.
 * @param[in] dst_desc
 *   Input. Description (width, height, stride, pixel_fmt, depth) of output destination data.
 *   Input. Pixel format of the destination data.
 *          It now supports CNCV_PIX_FMT_RGB, CNCV_PIX_FMT_BGR,
 *                           CNCV_PIX_FMT_RGBA, CNCV_PIX_FMT_BGRA,
 *                           CNCV_PIX_FMT_ARGB, CNCV_PIX_FMT_ABGR, and 
 *                           CNCV_PIX_FMT_GRAY.
 *   Input. Pixel depth of the destination image.
 *          It now supports CNCV_DEPTH_8U.
 *   Input. It has the following limitations:
 *          - 0 < height <= 8192, 0 < width <= 8192.
 *          - stride >= width * channel_num * sizeof(dataType).
 * @param[out] dst
 *   Output. Pointer to output destination data on the MLU.
 * @retval CNCV_STATUS_SUCCESS
 *   The function ends normally.
 * @note
 * - src1 stands for the first image data used in computing.
 * - src2 stands for the second image data used in computing.
 * - mask stands for the third image data used in computing.
 * - dst stands for the final result after computing.
 * - This function only supports one batch.
 * @par Requirements
 * - None.
 * @par Example
 * - None.
 *
 */
cncvStatus_t cncvBitwiseAnd(cncvHandle_t handle,
                           const cncvImageDescriptor src1_desc,
                           void *src1,
                           const cncvImageDescriptor src2_desc,
                           void *src2,
                           const cncvImageDescriptor mask_desc,
                           void *mask,
                           const cncvImageDescriptor dst_desc,
                           void *dst);

/*!
 * @brief
 *
 * This function performs the alpha blending operation.
 *
 * @param[in] handle
 *   Input. Pointer to the CNCV context.
 * @param[in] src1_desc
 *   Input. Description (width, height, stride, pixel_fmt, color_space, depth) of the src1
 * image.
 *   It has the following limitations:
 *   - 0 < height <= 8192, 0 < width <= 8192.
 *   - stride >= width * channel_num * sizeof(dataType).
 *   - The member depth now supports CNCV_DEPTH_8U and CNCV_DEPTH_32F.
 *   - The member pixel_fmt now supports CNCV_PIX_FMT_BGR, CNCV_PIX_FMT_RGB,
 *     CNCV_PIX_FMT_RGBA, CNCV_PIX_FMT_BGRA, CNCV_PIX_FMT_ARGB, and CNCV_PIX_FMT_ABGR.
 *   - The member color_space is not used in this operator.
 * @param[in] src1_roi
 *   Input. ROI of the src1 image.
 * @param[in] alpha
 *   Input. The weight parameter of src1 when the pixel_fmt is CNCV_PIX_FMT_BGR
 *   or CNCV_PIX_FMT_BGR.
 * @param[in] src2
 *   Input. Pointer to the MLU space for storing src2 image data.
  * @param[in] src2_desc
 *   Input. Description (width, height, stride, pixel_fmt, color_space, depth) of the src2
 * image.
 *   It has the following limitations:
 *   - 0 < height <= 8192, 0 < width <= 8192, height == src1_roi.h, width == src1_roi.w.
 *   - stride >= width * channel_num * sizeof(dataType).
 *   - The member depth now supports CNCV_DEPTH_8U and CNCV_DEPTH_32F.
 *   - The member pixel_fmt now supports CNCV_PIX_FMT_BGR, CNCV_PIX_FMT_RGB,
 *     CNCV_PIX_FMT_RGBA, CNCV_PIX_FMT_BGRA, CNCV_PIX_FMT_ARGB, and CNCV_PIX_FMT_ABGR.
 *   - The member color_space is not used in this operator.
 * @param[in] src2
 *   Input. Pointer to the MLU space for storing src1 image data.
 * @param[in] beta
 *   Input. The weight parameter of src2 when pixel_fmt is CNCV_PIX_FMT_BGR
 *   or CNCV_PIX_FMT_BGR.
 * @param[in] gamma
 *   Input. The weight parameter when pixel_fmt is CNCV_PIX_FMT_BGR
 *   or CNCV_PIX_FMT_BGR.
 * @param[in] dst_desc
 *   Input. Description (width, height, stride, pixel_fmt, color_space, depth) of the destination
 * images. All the fields must be the same as src1_desc.
 * @param[out] dst
 *   Input. Pointer to the MLU space for storing destination image data.
 * @retval CNCV_STATUS_SUCCESS
 *   The function ends normally.
 * @note
 * - None.
 * @par Requirements
 * - None.
 * @par Example
 * - None.
 *
 */
cncvStatus_t cncvAlphaBlend(cncvHandle_t handle,
                            const cncvImageDescriptor src1_desc,
                            const cncvRect src1_roi,
                            void *src1,
                            float alpha,
                            const cncvImageDescriptor src2_desc,
                            void *src2,
                            float beta,
                            float gamma,
                            const cncvImageDescriptor dst_desc,
                            void *dst);

/*!
 * @brief
 *
 * This function gets the size of workspace for the ::cncvMeanStd function.
 *
 * @param[in] channel_num
 *   Input. The number of the image channel.
 * @param[out] workspace_size
 *   Output. The value of the workspace size.
 * @retval CNCV_STATUS_SUCCESS
 *   The function ends normally.
 * @note
 * - None.
 * @par Requirements
 * - None.
 * @par Example
 * - None.
 *
 */
cncvStatus_t cncvGetMeanStdWorkspaceSize(const uint32_t channel_num,
                                         size_t *workspace_size);

/*!
 * @brief
 *
 * This function performs normalization with the given mean and std.
 *
 * @param[in] handle
 *   Input. Pointer to the CNCV context.
 * @param[in] batch_size
 *   Input. The number of images in batch.
 * @param[in] src_desc
 *   Input. Description (width, height, stride, pixel_fmt, color_space, depth) of the src
 * image.
 *   It has the following limitations:
 *   - 0 < height <= 8192, 0 < width <= 8192.
 *   - stride >= width * channel_num * sizeof(dataType).
 *   - The member depth now supports CNCV_DEPTH_8U, CNCV_DEPTH_16F, and CNCV_DEPTH_32F.
 *   - The member pixel_fmt now supports CNCV_PIX_FMT_GRAY, CNCV_PIX_FMT_BGR, CNCV_PIX_FMT_RGB,
 *     CNCV_PIX_FMT_RGBA, CNCV_PIX_FMT_BGRA, CNCV_PIX_FMT_ARGB, and CNCV_PIX_FMT_ABGR.
 *   - The member color_space is not used in this operator.
 * @param[in] mean
 *   Input. The given mean.
 * @param[in] std
 *   Input. The give std.
 * @param[in] dst_desc
 *   Input. Description (width, height, stride, pixel_fmt, color_space, depth) of the destination
 * images.
*   It has the following limitations:
 *   - 0 < height <= 8192, 0 < width <= 8192.
 *   - stride >= width * channel_num * sizeof(dataType).
 *   - The member depth now supports CNCV_DEPTH_16F and CNCV_DEPTH_32F.
 *   - The member pixel_fmt now supports CNCV_PIX_FMT_GRAY, CNCV_PIX_FMT_BGR, CNCV_PIX_FMT_RGB,
 *     CNCV_PIX_FMT_RGBA, CNCV_PIX_FMT_BGRA, CNCV_PIX_FMT_ARGB, CNCV_PIX_FMT_ABGR.
 *   - The member color_space is not used in this operator.
 * @param[out] dst
 *   Input. Pointer to the MLU space for storing destination image data.
 * @param[in] workspace_size
 *   Input. Size of the extra space on the MLU for this operator.
 * @param[in] workspace
 *   Input. Pointer to the extra space on the MLU for this operator.
 * @retval CNCV_STATUS_SUCCESS
 *   The function ends normally.
 * @note
 * - None.
 * @par Requirements
 * - None.
 * @par Example
 * - None.
 *
 */
cncvStatus_t cncvMeanStd(cncvHandle_t handle,
                         const uint32_t batch_size,
                         const cncvImageDescriptor src_desc,
                         void **src,
                         const float *mean,
                         const float *std,
                         const cncvImageDescriptor dst_desc,
                         void **dst,
                         const size_t workspace_size,
                         void *workspace);

/*********************************************************************
* Color and Sampling Conversion
********************************************************************/
/*!
 * @brief
 *
 * This function gets the size of workspace for converting YUV420SP images to RGBX images.
 *
 * @deprecated. This API will be deleted in future version. cncvGetYuvToRgbxWorkspaceSize is recommended to use.
 *
 * @param[in] src_pixfmt
 *   Input. Pixel format of the source data.
 *          It now supports CNCV_PIX_FMT_NV12 and CNCV_PIX_FMT_NV21.
 * @param[in] dst_pixfmt
 *   Input. Pixel format of the destination data.
 *          It now supports CNCV_PIX_FMT_RGB, CNCV_PIX_FMT_BGR,
 *                        CNCV_PIX_FMT_RGBA, CNCV_PIX_FMT_BGRA,
 *                        CNCV_PIX_FMT_ARGB, and CNCV_PIX_FMT_ABGR.
 * @param[out] size
 *   Output. The value of workspace size.
 * @retval CNCV_STATUS_SUCCESS
 *   The function ends normally.
 * @note
 * - None.
 * @par Requirements
 * - None.
 * @par Example
 * - None.
 *
 */
cncvStatus_t cncvGetYuv420spToRgbxWorkspaceSize(cncvPixelFormat src_pixfmt,
                                                cncvPixelFormat dst_pixfmt,
                                                size_t *size);

/*!
 * @brief
 *
 * This function converts YUV420SP images to RGBX images. The CNCV_DEPTH_8U image
 * depth is supported.
 *
 * @deprecated. This API will be deleted in future version. cncvYuvToRgbx is recommended to use.
 *
 * @param[in] handle
 *   Input. Pointer to the CNCV context.
 * @param[in] batch_size
 *   Input. The number of images in batch.
 * @param[in] src_desc
 *   Input. Description (width, height, stride, pixel_fmt, color_space, depth) of the source images.
 *          The member color_space now supports CNCV_COLOR_SPACE_BT_601 and CNCV_COLOR_SPACE_BT_709.
 *          The member pixel_fmt now supports CNCV_PIX_FMT_NV12 and CNCV_PIX_FMT_NV21.
 * @param[in] src
 *   Input. Pointer to source image data on the MLU.
 * @param[in] dst_desc
 *   Input. Description (width, height, stride, pixel_fmt, color_space, depth) of the destination images.
 *          The member color_space now supports CNCV_COLOR_SPACE_BT_601 and CNCV_COLOR_SPACE_BT_709.
 *          The member pixel_fmt now supports
 *                         CNCV_PIX_FMT_RGB, CNCV_PIX_FMT_BGR,
 *                         CNCV_PIX_FMT_RGBA, CNCV_PIX_FMT_BGRA,
 *                         CNCV_PIX_FMT_ARGB, and CNCV_PIX_FMT_ABGR.
 *
 * @param[out] dst
 *   Output. Pointer to the MLU space for storing output data.
 * @param[in] workspace_size
 *   Input. Size of the extra space to convert YUV420SP to RGBX on the MLU.
 * @param[in] workspace
 *   Input. Pointer to the extra space to convert YUV420SP to RGBX on the MLU.
 * @retval CNCV_STATUS_SUCCESS
 *   The function ends normally.
 * @note
 * - None.
 * @par Requirements
 * - None.
 * @par Example
 * - None.
 *
 */
cncvStatus_t cncvYuv420spToRgbx(cncvHandle_t handle,
                                const uint32_t batch_size,
                                const cncvImageDescriptor src_desc,
                                void **src,
                                const cncvImageDescriptor dst_desc,
                                void **dst,
                                const size_t workspace_size,
                                void *workspace);

/*!
 * @brief
 *
 * This function gets the size of workspace for converting YUV images to RGBX images.
 *
 * @param[in] src_pixfmt
 *   Input. Pixel format of the source data.
 *          It now supports CNCV_PIX_FMT_NV12, CNCV_PIX_FMT_NV21, CNCV_PIX_FMT_YUYV,
 *          CNCV_PIX_FMT_YVYU, CNCV_PIX_FMT_UYVY, and CNCV_PIX_FMT_VYUY.
 * @param[in] dst_pixfmt
 *   Input. Pixel format of the destination data.
 *          It now supports CNCV_PIX_FMT_RGB, CNCV_PIX_FMT_BGR,
 *                        CNCV_PIX_FMT_RGBA, CNCV_PIX_FMT_BGRA,
 *                        CNCV_PIX_FMT_ARGB, and CNCV_PIX_FMT_ABGR.
 * @param[out] size
 *   Output. The value of the workspace size.
 * @retval CNCV_STATUS_SUCCESS
 *   The function ends normally.
 * @note
 * - None.
 * @par Requirements
 * - None.
 * @par Example
 * - None.
 *
 */
cncvStatus_t cncvGetYuvToRgbxWorkspaceSize(cncvPixelFormat src_pixfmt,
                                           cncvPixelFormat dst_pixfmt,
                                           size_t *size);

/*!
 * @brief
 *
 * This function converts YUV images to RGBX images. The CNCV_DEPTH_8U image
 * depth is supported. YUV images only support YUV 420sp or YUV 422 packed.
 *
 * @param[in] handle
 *   Input. Pointer to the CNCV context.
 * @param[in] batch_size
 *   Input. The number of images in batch.
 * @param[in] src_desc
 *   Input. Description (width, height, stride, pixel_fmt, color_space, depth) of the source images.
 *          The member color_space now supports CNCV_COLOR_SPACE_BT_601 and CNCV_COLOR_SPACE_BT_709.
 *          The member pixel_fmt now supports CNCV_PIX_FMT_NV12, CNCV_PIX_FMT_NV21,
 *          CNCV_PIX_FMT_YUYV, CNCV_PIX_FMT_YVYU, CNCV_PIX_FMT_YVY, and CNCV_PIX_FMT_VYUY.
 * @param[in] src
 *   Input. Pointer to source image data on the MLU.
 * @param[in] dst_desc
 *   Input. Description (width, height, stride, pixel_fmt, color_space, depth) of the destination images.
 *          The member color_space now supports CNCV_COLOR_SPACE_BT_601 and CNCV_COLOR_SPACE_BT_709.
 *          The member pixel_fmt now supports
 *                         CNCV_PIX_FMT_RGB, CNCV_PIX_FMT_BGR,
 *                         CNCV_PIX_FMT_RGBA, CNCV_PIX_FMT_BGRA,
 *                         CNCV_PIX_FMT_ARGB, and CNCV_PIX_FMT_ABGR.
 *
 * @param[out] dst
 *   Output. Pointer to the MLU space for storing output data.
 * @param[in] workspace_size
 *   Input. Size of the extra space to convert YUV to RGBX on the MLU.
 * @param[in] workspace
 *   Input. Pointer to the extra space to convert YUV to RGBX on the MLU.
 * @retval CNCV_STATUS_SUCCESS
 *   The function ends normally.
 * @note
 * - None.
 * @par Requirements
 * - None.
 * @par Example
 * - None.
 *
 */
cncvStatus_t cncvYuvToRgbx(cncvHandle_t handle,
                           const uint32_t batch_size,
                           const cncvImageDescriptor src_desc,
                           void **src,
                           const cncvImageDescriptor dst_desc,
                           void **dst,
                           const size_t workspace_size,
                           void *workspace);

/*!
 * @brief
 *
 * This function realizes the conversion between RGB, BGR, RGBA, BGRA, ARGB, and ABGR data.
 *
 * @param[in] handle
 *   Input. Pointer to the CNCV context.
 * @param[in] batch_size
 *   Input. Number of images in batch.
 * @param[in] src_desc
 *   Input. Description (width, height, stride, color_space, depth, pixel_fmt) of the
 *   source image. The description of each source image is same.
 *   The member depth now supports CNCV_DEPTH_8U, CNCV_DEPTH_32F.
 *   The member pixel_fmt now supports CNCV_PIX_FMT_BGR, CNCV_PIX_FMT_BGRA,
 *                                      CNCV_PIX_FMT_RGB, CNCV_PIX_FMT_RGBA,
 *                                      CNCV_PIX_FMT_ARGB, CNCV_PIX_FMT_ABGR.
 * @param[in] src_roi
 *   Input. ROI of the source image. The ROI of each source image is same.
 * @param[in] src[in]
 *   Input. Pointer to the source image data on the MLU.
 * @param[in] dst_desc
 *   Input. Description (width, height, stride, color_space, depth, pixel_fmt) of the
 *   destination image. The description of each destination image is same.
 *   The member depth now supports CNCV_DEPTH_8U, CNCV_DEPTH_32F.
 *   The member pixel_fmt now supports CNCV_PIX_FMT_BGR, CNCV_PIX_FMT_BGRA,
 *                                      CNCV_PIX_FMT_RGB, CNCV_PIX_FMT_RGBA,
 *                                      CNCV_PIX_FMT_ARGB, CNCV_PIX_FMT_ABGR.
 * @param[in] dst_roi
 *   Input. ROI of the destination image. The ROI of each destination image is same.
 * @param[out] dst
 *   Output. Pointer to the MLU space for storing output data.
 * @retval CNCV_STATUS_SUCCESS
 *   The function ends normally.
 * @note
 * - None.
 * @par Requirements
 * - None.
 * @par Example
 * - None.
 *
 */
cncvStatus_t cncvRgbxToRgbx(cncvHandle_t handle,
                            const uint32_t batch_size,
                            const cncvImageDescriptor src_desc,
                            const cncvRect src_roi,
                            void **src,
                            const cncvImageDescriptor dst_desc,
                            const cncvRect dst_roi,
                            void **dst);

/*!
 * @brief
 *
 * This function realizes the conversion from RGBX to YUV.
 *
 * @param[in] handle
 *   Input. Pointer to the CNCV context.
 * @param[in] src_desc
 *   Input. Description (width, height, stride, color_space, depth, pixel_fmt) of the
 *   source image.
 *   The member depth now supports CNCV_DEPTH_8U.
 *   The member pixel_fmt now supports CNCV_PIX_FMT_BGR, CNCV_PIX_FMT_RGB,
 *                                      CNCV_PIX_FMT_BGRA, CNCV_PIX_FMT_RGBA,
 *                                      CNCV_PIX_FMT_ABGR, and CNCV_PIX_FMT_ARGB.
 * @param[in] src_roi
 *   Input. ROI of the source image.
 * @param[in] src[in]
 *   Input. Pointer to the source image data on the MLU.
 * @param[in] dst_desc
 *   Input. Description (width, height, stride, color_space, depth, pixel_fmt) of the
 *   destination image.
 *   The member depth now supports CNCV_DEPTH_8U.
 *   The member pixel_fmt now supports CNCV_PIX_FMT_NV12 and CNCV_PIX_FMT_NV21.
 * @param[out] dst
 *   Output. Pointer to the MLU space for storing output data.
 * @retval
 *   CNCV_STATUS_SUCCESS
 *   The function ends normally.
 *   CNCV_STATUS_BAD_PARAM
 *   The wrong parameter is input.
 * @par Limitation
 * - None.
 * @par Requirements
 * - None.
 * @par Example
 * - None.
 *
 */
cncvStatus_t cncvRgbxToYuv(cncvHandle_t handle,
                           const cncvImageDescriptor src_desc,
                           const cncvRect src_roi,
                           const void *src,
                           const cncvImageDescriptor dst_desc,
                           void **dst);

/*!
 * @brief 
 *
 * This function converts LAB data to BGR data, supporting CNCV_DEPTH_8U data
 * and CNCV_DEPTH_32F data.
 *
 * @param[in] handle
 *   Input. Pointer to the CNCV context.
 * @param[in] batch_size
 *   Input. Number of images in batch.
 * @param[in] src_desc
 *   Input. Description (width, height, stride, color_space, depth, pixel_fmt) of the
 *   source image.
 *   The member depth now supports CNCV_DEPTH_8U, CNCV_DEPTH_32F.
 *   The member pixel_fmt now supports CNCV_PIX_FMT_LAB.
 * @param[in] src[in]
 *   Input. Pointer to source image data on MLU.
 * @param[in] dst_desc
 *   Input. Description (width, height, stride, color_space, depth, pixel_fmt) of the
 *   destination image.
 *   The member depth now supports CNCV_DEPTH_8U and CNCV_DEPTH_32F.
 *   The member pixel_fmt now supports CNCV_PIX_FMT_BGR.
 * @param[out] dst
 *   Output. Pointer to MLU space for storing output data.
 * @retval CNCV_STATUS_SUCCESS
 *   The function ends normally.
 * @note
 * - None.
 * @par Requirements
 * - None.
 * @par Example
 * - None.
 *
 */
cncvStatus_t cncvLabToBgr(cncvHandle_t handle,
                          const uint32_t batch_size,
                          const cncvImageDescriptor src_desc,
                          void **src,
                          const cncvImageDescriptor dst_desc,
                          void **dst);

/*!
 * @brief
 *
 * This function converts BGR data to LAB data, supporting CNCV_DEPTH_8U data
 * and CNCV_DEPTH_32F data.
 *
 * @param[in] handle
 *   Input. Pointer to the CNCV context.
 * @param[in] batch_size
 *   Input. Number of images in batch.
 * @param[in] src_desc
 *   Input. Description (width, height, stride, color_space, depth, pixel_fmt) of the
 *   source image.
 *   The member depth now supports CNCV_DEPTH_8U and CNCV_DEPTH_32F.
 *   The member pixel_fmt now supports CNCV_PIX_FMT_BGR.
 * @param[in] src[in]
 *   Input. Pointer to source image data on MLU.
 * @param[in] dst_desc
 *   Input. Description (width, height, stride, color_space, depth, pixel_fmt) of the
 *   destination image.
 *   The member depth now supports CNCV_DEPTH_8U and CNCV_DEPTH_32F.
 *   The member pixel_fmt now supports CNCV_PIX_FMT_LAB.
 * @param[out] dst
 *   Output. Pointer to MLU space for storing output data.
 * @retval CNCV_STATUS_SUCCESS
 *   The function ends normally.
 * @note
 * - None.
 * @par Requirements
 * - None.
 * @par Example
 * - None.
 *
 */
cncvStatus_t cncvBgrToLab(cncvHandle_t handle,
                          const uint32_t batch_size,
                          const cncvImageDescriptor src_desc,
                          void **src,
                          const cncvImageDescriptor dst_desc,
                          void **dst);

/*********************************************************************
 * Data Exchange and Initialization
********************************************************************/
/*!
 * @brief
 *
 * This function copies data to image ROI.
 * @param[in] handle
 *   Input. Pointer to the CNCV context.
 * @param[in] batch_size
 *   Input. Number of images in batch.
 * @param[in] src_desc
 *   Input. Description (width, height, stride, color_space, depth, pixel_fmt) of the source image.
 *   The member depth now supports CNCV_DEPTH_8U,
 *                         CNCV_DEPTH_16F, and CNCV_DEPTH_32F.
 *   The member pixel_fmt now supports CNCV_PIX_FMT_GRAY,
 *                         CNCV_PIX_FMT_BGR, CNCV_PIX_FMT_RGB,
 *                         CNCV_PIX_FMT_ABGR, CNCV_PIX_FMT_ARGB,
 *                         CNCV_PIX_FMT_BGRA, and CNCV_PIX_FMT_RGBA.
 * @param[in] src_roi
 *   Input. ROI of the source image.
 * @param[in] src
 *   Input. Source image.
 * @param[in] mask_desc
 *   Input. Description (width, height, stride, color_space, depth, pixel_fmt) of the mask image data.
 *   The member depth now supports CNCV_DEPTH_8U.
 *   The member pixel_fmt now supports CNCV_PIX_FMT_GRAY,
 *                         CNCV_PIX_FMT_BGR, CNCV_PIX_FMT_RGB,
 *                         CNCV_PIX_FMT_ABGR, CNCV_PIX_FMT_ARGB,
 *                         CNCV_PIX_FMT_BGRA, and CNCV_PIX_FMT_RGBA.
 * @param[in] mask_data
 *   Input. Pointer to the MLU space for storing mask data.
 * @param[in] dst_desc
 *   Input. Description (width, height, stride, pixel_fmt, color_space, depth) of the destination images.
 *   The member depth now supports CNCV_DEPTH_8U,
 *                         CNCV_DEPTH_16F, and CNCV_DEPTH_32F.
 *   The member pixel_fmt now supports CNCV_PIX_FMT_GRAY,
 *                         CNCV_PIX_FMT_BGR, CNCV_PIX_FMT_RGB,
 *                         CNCV_PIX_FMT_ABGR, CNCV_PIX_FMT_ARGB,
 *                         CNCV_PIX_FMT_BGRA, and CNCV_PIX_FMT_RGBA.
 * @param[in] dst_roi
 *   Input. ROI of destination image.
 * @param[out] dst
 *   Output. Pointer to the MLU space for storing destination image data.
 * @retval CNCV_STATUS_SUCCESS
 *   The function ends normally.
 * @note
 * - None.
 * @par Requirements
 * - None.
 * @par Example
 * - None.
 *
 */
  cncvStatus_t cncvCopyTo(cncvHandle_t handle,
                          const uint32_t batch_size,
                          const cncvImageDescriptor src_desc,
                          cncvRect src_roi,
                          void **src,
                          const cncvImageDescriptor mask_desc,
                          void **mask_data,
                          const cncvImageDescriptor dst_desc,
                          cncvRect dst_roi,
                          void **dst);


/*!
 * @brief
 *
 * This function fills data to image ROI.
 *
 * @param[in] handle
 *   Input. Pointer to the CNCV context.
 * @param[in] batch_size
 *   Input. Number of images in batch.
 * @param[in] scalar
 *   Input. Pointer to the MLU space for fill data.
 * @param[in] depth
 *   Input. Pixel depth of the destination image.
 * @param[in] desc
 *   Input. Description (width, height, stride, pixel_fmt, color_space, depth) of the destination images.
 *   It has the following limitations:
 *   - 0 < height <= 8192, 0 < width <= 8192.
 *   - The member depth now supports CNCV_DEPTH_8U, CNCV_DEPTH_16F, and CNCV_DEPTH_32F.
 *   - The member pixel_fmt now supports CNCV_PIX_FMT_GRAY,
 *                                      CNCV_PIX_FMT_BGR, CNCV_PIX_FMT_RGB,
 *                                      CNCV_PIX_FMT_ABGR, CNCV_PIX_FMT_ARGB,
 *                                      CNCV_PIX_FMT_BGRA, and CNCV_PIX_FMT_RGBA.
 * @param[in] roi
 *   Input. ROI of output image.
 * @param[out] dst
 *   Output. Pointer to the MLU space for storing output data.
 * @retval CNCV_STATUS_SUCCESS
 *   The function ends normally.
 * @note
 * - None.
 * @par Requirements
 * - None.
 * @par Example
 * - None.
 *
 */
cncvStatus_t cncvFill(cncvHandle_t handle,
                      const uint32_t batch_size,
                      void *scalar,
                      const cncvImageDescriptor desc,
                      cncvRect roi,
                      void **dst);

/*!
 * @brief
 *
 * This function pads image ROI.
 *
 * @param[in] handle
 *   Input. Pointer to the CNCV context.
 * @param[in] src_desc
 *   Input. Description (width, height, stride, pixel_fmt, color_space, depth) of the destination
 * images.
 *   It has the following limitations:
 *   - 0 < height <= 8192, 0 < width <= 8192.
 *   - stride >= width * channel_num * sizeof(dataType).
 *   - The member depth now supports CNCV_DEPTH_8U, CNCV_DEPTH_16F, and CNCV_DEPTH_32F.
 *   - The member pixel_fmt now supports CNCV_PIX_FMT_GRAY,
 *                                      CNCV_PIX_FMT_BGR, CNCV_PIX_FMT_RGB,
 *                                      CNCV_PIX_FMT_ABGR, CNCV_PIX_FMT_ARGB,
 *                                      CNCV_PIX_FMT_BGRA, and CNCV_PIX_FMT_RGBA.
 *   Color_space is not used in this operator.
 * @param[in] src_roi
 *   Input. ROI of source images to be padded.
 * @param[in] src
 *   Input. Pointer to the MLU space for storing source image data.
 * @param[in] dst_desc
 *   Input. Description (width, height, stride, pixel_fmt, color_space, depth) of the destination
 * images.
 *   It has the following limitations:
 *   - 0 < height <= 8192, 0 < width <= 8192.
 *   - height = src_roi.w + pad_left + pad_right.
 *   - width = src_roi.h + pad_top + pad_bottom.
 *   - stride = width*channel_num*sizeof(dataType).
 *   - The member depth now supports CNCV_DEPTH_8U, CNCV_DEPTH_16F, and CNCV_DEPTH_32F.
 *   - The member depth is the same as the src depth.
 *   - The member pixel_fmt now supports CNCV_PIX_FMT_GRAY,
 *                                      CNCV_PIX_FMT_BGR, CNCV_PIX_FMT_RGB,
 *                                      CNCV_PIX_FMT_ABGR, CNCV_PIX_FMT_ARGB,
 *                                      CNCV_PIX_FMT_BGRA, and CNCV_PIX_FMT_RGBA.
 *   - The member pixel_fmt is the same as the src pix_fmt.
 *   - The member color_space is not used in this operator.
 * @param[out] dst
 *   Input. Pointer to the MLU space for storing destination image data.
 * @param[in] pad_top
 *   Input. Number of top pixels to be padded.
 * @param[in] pad_bottom
 *   Input. Number of bottom pixels to be padded.
 * @param[in] pad_left
 *   Input. Number of left pixels to be padded.
 * @param[in] pad_right
 *   Input. Number of right pixels to be padded.
 * @param[in] pad_type
 *   Input. Pad type. It supports CNCV_PAD_CONSTANT,
 * CNCV_PAD_REPLICATE, CNCV_PAD_REFLECT, CNCV_PAD_WRAP, and CNCV_PAD_REFLECT_101.
 * @param[in] scalar
 *   Input. Pointer to the MLU space for padding data when the pad_type is CNCV_PAD_CONSTANT.
 * @retval CNCV_STATUS_SUCCESS
 *   The function ends normally.
 * @note
 * - None.
 * @par Requirements
 * - None.
 * @par Example
 * - None.
 *
 */
cncvStatus_t cncvPad(cncvHandle_t handle,
                     const cncvImageDescriptor src_desc,
                     const cncvRect src_roi,
                     void *src,
                     const cncvImageDescriptor dst_desc,
                     void *dst,
                     const uint32_t pad_top,
                     const uint32_t pad_bottom,
                     const uint32_t pad_left,
                     const uint32_t pad_right,
                     cncvPadType pad_type,
                     void *scalar);

/*!
 * @brief
 *
 * This function merges several single-channel arrays to make a multi-channel array.
 *
 * @param[in] handle
 *   Input. Pointer to the CNCV context.
 * @param[in] count
 *   Input. Number of input arrays. It must be equal to the channel number of output.
 * @param[in] src_desc
 *   Input. Description (width, height, stride, color_space, depth, pixel_fmt) of the
 *   source image. The description of each source image is same.
 *   The member depth now supports No limitation.
 *   The member pixel_fmt now supports CNCV_PIX_FMT_GRAY.
 * @param[in] src_roi
 *   Input. ROI of the source image. The ROI of each source image is same.
 * @param[in] src[in]
 *   Input. Pointer to the source image data on the MLU.
 * @param[in] dst_desc
 *   Input. Description (width, height, stride, color_space, depth, pixel_fmt) of the destination
 *   image. Output array has the same depth as input arrays. Output array has the same width and
 *   height as the src_roi.
 *   The member depth now supports No limitation.
 *   The member pixel_fmt now supports CNCV_PIX_FMT_BGR, CNCV_PIX_FMT_BGRA, CNCV_PIX_FMT_ABGR,
 *                                      CNCV_PIX_FMT_RGB, CNCV_PIX_FMT_RGBA, and CNCV_PIX_FMT_ARGB.
 * @param[out] dst
 *   Output. Pointer to the MLU space for storing output data.
 * @retval CNCV_STATUS_SUCCESS
 *   The function ends normally.
 * @note
 * - None.
 * @par Requirements
 * - None.
 * @par Example
 * - None.
 *
 */
cncvStatus_t cncvMerge(cncvHandle_t handle,
                       const uint32_t count,
                       const cncvImageDescriptor src_desc,
                       const cncvRect src_roi,
                       void **src,
                       const cncvImageDescriptor dst_desc,
                       void *dst);

/*!
 * @brief
 *
 * This function divides a multi-channel array into several single-channel arrays.
 *
 * @param[in] handle
 *   Input. Pointer to the CNCV context.
 * @param[in] src_desc
 *   Input. Description (width, height, stride, color_space, depth, pixel_fmt) of the
 *   source image.
 *   The member depth now supports No limitation.
 *   The member pixel_fmt now supports CNCV_PIX_FMT_BGR, CNCV_PIX_FMT_BGRA, CNCV_PIX_FMT_ABGR,
 *                                      CNCV_PIX_FMT_RGB, CNCV_PIX_FMT_RGBA, CNCV_PIX_FMT_ARGB.
 * @param[in] src_roi
 *   Input. ROI of the source image.
 * @param[in] src[in]
 *   Input. Pointer to the source image data on the MLU.
 * @param[in] dst_desc
 *   Input. Description (width, height, stride, color_space, depth, pixel_fmt) of the destination
 *   image. The description of each destination image is same. Output arrays have the same depth
 *   as input array. Output arrays have the same width and height as the src_roi.
 *   The member depth now supports No limitation.
 *   The member pixel_fmt now supports CNCV_PIX_FMT_GRAY.
 * @param[out] dst
 *   Output. Pointer to the MLU space for storing output data.
 * @retval CNCV_STATUS_SUCCESS
 *   The function ends normally.
 * @note
 * - None.
 * @par Requirements
 * - None.
 * @par Example
 * - None.
 *
 */
cncvStatus_t cncvSplit(cncvHandle_t handle,
                       const cncvImageDescriptor src_desc,
                       const cncvRect src_roi,
                       void *src,
                       const cncvImageDescriptor dst_desc,
                       void **dst);
/*!
 * @brief
 *
 * This function gets the minimum size of workspace for the ::cncvCrop function.
 *
 * @param[in] batch_size
 *   Input. The number of images in batch.
 * @param[out] workspace_size
 *   Output. The size of workspace.
 * @retval CNCV_STATUS_SUCCESS
 *   The function ends normally.
 * @note
 * - None.
 * @par Requirements
 * - None.
 * @par Example
 * - None.
 *
 */
cncvStatus_t cncvGetCropWorkspaceSize(const uint32_t batch_size,
                                      size_t *workspace_size);


/*!
 * @brief
 *
 * This function crops image data.
 *
 * @param[in] handle
 *   Input. Pointer to the CNCV context.
 * @param[in] batch_size
 *   Input. The number of images in batch.
 * @param[in] src_desc
 *   Input. Description (width, height, stride, pixel_fmt, color_space, depth) of the source images.
 *          Members width, height, and stride for each source images can be different, but other members must be the same.
 *          - The member depth supports CNCV_DEPTH_8U, when pixel_fmt is CNCV_PIX_FMT_NV12 or CNCV_PIX_FMT_NV21.
 *          - The member depth supports CNCV_DEPTH_8U, CNCV_DEPTH_16F and CNCV_DEPTH_32F,
 *            when pixel_fmt is CNCV_PIX_FMT_RGBA, CNCV_PIX_FMT_BGRA,
 *                         CNCV_PIX_FMT_ARGB, CNCV_PIX_FMT_ABGR,
 *                         CNCV_PIX_FMT_RGB, CNCV_PIX_FMT_BGR,
 *                         or CNCV_PIX_FMT_GRAY.
 * @param[in] src_rois
 *   Input. Pointer to the CPU space for storing ROIs on source images.
 *          The number of ROIs must be equal to batch_size. The ROIs of each
 *          batch can be different.
 * @param[in] src
 *   Input. Pointer to source image data on the MLU.
 *          The number of source images must be equal to batch_size.
 * @param[in] dst_desc
 *   Input. Description (width, height, stride, pixel_fmt, color_space, depth) of the destination images.
 *          Members width, height, and stride for each destination images can be different, but other members must be the same.
 *          - The member depth supports CNCV_DEPTH_8U,
 *            when pixel_fmt is CNCV_PIX_FMT_NV12 or CNCV_PIX_FMT_NV21.
 *          - The member depth supports CNCV_DEPTH_8U, CNCV_DEPTH_16F and CNCV_DEPTH_32F,
 *            when pixel_fmt is CNCV_PIX_FMT_RGBA, CNCV_PIX_FMT_BGRA,
 *                         CNCV_PIX_FMT_ARGB, CNCV_PIX_FMT_ABGR,
 *                         CNCV_PIX_FMT_RGB, CNCV_PIX_FMT_BGR,
 *                         or CNCV_PIX_FMT_GRAY.
 * @param[in] dst_rois
 *   Input. Pointer to the CPU space for storing ROIs on destination images.
 *          The number of ROIs must be equal to batch_size.
 * @param[out] dst
 *   Output. Pointer to the MLU space for storing output data.
 *           The number of destination images must be equal to batch_size.
 * @param[in] workspace_size
 *   Input. Size of the extra space for storing source ROIs and destination ROIs on the MLU.
 * @param[in] workspace
 *   Input. Pointer to the extra space for storing source ROIs and destination ROIs on the MLU.
 * @retval CNCV_STATUS_SUCCESS
 *   The function ends normally.
 * @note
 * - None.
 * @par Requirements
 * - None.
 * @par Example
 * - None.
 *
 */
cncvStatus_t cncvCrop(cncvHandle_t handle,
                      const uint32_t batch_size,
                      const cncvImageDescriptor *src_desc,
                      const cncvRect *src_rois,
                      void **src,
                      const cncvImageDescriptor *dst_desc,
                      const cncvRect *dst_rois,
                      void **dst,
                      const size_t workspace_size,
                      void *workspace);

/*********************************************************************
 * Transforms
********************************************************************/
/*!
 * @brief
 *
 * This function gets the size of workspace for the cncvResizeGray function.
 *
 * @param[in] batch_size
 *   Input. The number of images in batch.
 * @param[in] psrc_descs
 *   Input. Pointer to description (width, height, stride, pixel_fmt, color_space, depth) of the source images.
 *          Members width, height, and stride for each source images can be different, but other members must be the same.
 *          The number of source image descriptions must be equal to batch_size.
 *          The member depth in current structure supports CNCV_DEPTH_8U.
 *          The member pixel_fmt in current structure supports CNCV_PIX_FMT_GRAY.
 * @param[in] src_rois
 *   Input. Pointer to ROI (x, y, w, h) of source images.
 *          The number of source image ROIs must be equal to batch_size.
 * @param[in] pdst_descs
 *   Input. Pointer to description (width, height, stride, pixel_fmt, color_space, depth) of the destination images.
 *          Members width, height, and stride for each destination images can be different, but other members must be the same.
 *          The number of destination image descriptions must be equal to batch_size.
 *          The member depth in current structure supports CNCV_DEPTH_8U.
 *          The member pixel_fmt in current structure supports CNCV_PIX_FMT_GRAY.
 * @param[in] dst_rois
 *   Input. Pointer to ROI (x, y, w, h) of destination images.
 *          The number of destination image ROIs must be equal to batch_size.
 * @param[out] size
 *   Output. The value of workspace size.
 * @retval CNCV_STATUS_SUCCESS
 *   The function ends normally.
 * @note
 * - None.
 * @par Requirements
 * - None.
 * @par Example
 * - None.
 *
 */
cncvStatus_t cncvGetResizeGrayWorkspaceSize(const uint32_t batch_size,
                                               const cncvImageDescriptor *psrc_descs,
                                               const cncvRect *src_rois,
                                               const cncvImageDescriptor *pdst_descs,
                                               const cncvRect *dst_rois,
                                               size_t *workspace_size);


/*!
 * @brief
 *
 * This function resizes gray2gray images.
 *
 * @param[in] handle
     Input. Pointer to the CNCV context.
 * @param[in] batch_size
 *   Input. The number of images in batch.
 * @param[in] psrc_descs
 *   Input. Pointer to description (width, height, stride, pixel_fmt, color_space, depth) of the source images.
 *          Members width, height, and stride for each source images can be different, but other members must be the same.
 *          The number of source image descriptions must be equal to batch_size.
 *          The member depth in current structure supports CNCV_DEPTH_8U.
 *          The member color_space is not used in this operator.
 *          The member pixel_fmt in current structure supports CNCV_PIX_FMT_GRAY.
 * @param[in] src_rois
 *   Input. Pointer to ROI (x, y, w, h) of source images.
 *          The number of source image ROIs must be equal to batch_size.
 * @param[in] src
 *   Input. Pointer to MLU space for storing src image data.
 *          The number of source image src data must be equal to batch_size.
 * @param[in] pdst_descs
 *   Input. Pointer to description (width, height, stride, pixel_fmt, color_space, depth) of the destination images.
 *          Members width, height, and stride for each destination images can be different, but other members must be the same.
 *          The number of source image descriptions must be equal to batch_size.
 *          The member depth in current structure supports CNCV_DEPTH_8U.
 *          The member color_space is not used in this operator.
 *          The member pixel_fmt in current structure supports CNCV_PIX_FMT_GRAY.
 * @param[in] dst_rois
 *   Input. Pointer to ROI (x, y, w, h) of destination images.
 *          The number of destination image ROIs must be equal to batch_size.
 * @param[out] dst
 *   Output. Pointer to the MLU space for storing output data.
 *           The number of destination images must be equal to batch_size.
 * @param[in] workspace_size
 *   Input. Size of the extra space to resize and convert on the MLU.
 * @param[in] workspace
 *   Input. Pointer to the extra space to resize and convert on the MLU.
 * @param[in] interpolation
 *   Input. Interpolation method. It supports CNCV_INTER_BILINEAR.
 * @retval CNCV_STATUS_SUCCESS
 *   The function ends normally.
 * @note
 * - None.
 * @par Requirements
 * - None.
 * @par Example
 * - None.
 *
 */
cncvStatus_t cncvResizeGray(cncvHandle_t handle,
                            const uint32_t batch_size,
                            const cncvImageDescriptor *psrc_descs,
                            const cncvRect *src_rois,
                            void **src,
                            const cncvImageDescriptor *pdst_descs,
                            const cncvRect *dst_rois,
                            void **dst,
                            const size_t workspace_size,
                            void *workspace,
                            cncvInterpolation interpolation);

/*!
 * @brief
 *
 * This function gets the size of workspace for the ::cncvResizeRgbx function.
 *
 * @param[in] batch_size
 *   Input. The number of images in batch.
 * @param[out] workspacesize
 *   Output. The value of workspace size.
 * @retval CNCV_STATUS_SUCCESS
 *   The function ends normally.
 * @note
 * - None.
 * @par Requirements
 * - None.
 * @par Example
 * - None.
 *
 */
cncvStatus_t cncvGetResizeRgbxWorkspaceSize(const uint32_t batch_size,
                                            size_t *workspace_size);

/*!
 * @brief
 *
 * This function resizes RGBX images.
 *
 * @param[in] handle
 *   Input. Pointer to the CNCV context.
 * @param[in] batch_size
 *   Input. The number of images in batch.
 * @param[in] src_desc
 *   Input. Description (width, height, stride, pixel_fmt, color_space, depth) of the source images.
 *          The member pixel_fmt now supports
 *                         CNCV_PIX_FMT_RGBA, CNCV_PIX_FMT_BGRA,
 *                         CNCV_PIX_FMT_ARGB, CNCV_PIX_FMT_ABGR,
 *                         CNCV_PIX_FMT_RGB, and CNCV_PIX_FMT_BGR.
 *          The member depth now supports CNCV_DEPTH_8U.
 * @param[in] src
 *   Input. Pointer to source image data on the MLU.
 *          The number of source images must be equal to batch_size.
 * @param[in] src_rois
 *   Input. Pointer to the CPU space for storing ROIs on source images.
 *          The number of ROIs must be equal to batch_size.
 * @param[in] dst_desc
 *   Input. Description (width, height, stride, pixel_fmt, color_space, depth) of the destination images.
 *          The member pixel_fmt now supports
 *                         CNCV_PIX_FMT_RGBA, CNCV_PIX_FMT_BGRA,
 *                         CNCV_PIX_FMT_ARGB, CNCV_PIX_FMT_ABGR,
 *                         CNCV_PIX_FMT_RGB, and CNCV_PIX_FMT_BGR.
 *          The member depth now supports CNCV_DEPTH_8U.
 * @param[in] dst_rois
 *   Input. Pointer to the CPU space for storing ROIs on destination images.
 *          The number of ROIs must be equal to batch_size. Different
 *          destination ROIs info must be equal, otherwise it will return
 *          CNCV_STATUS_NOT_SUPPORTED.
 * @param[out] dst
 *   Output. Pointer to the MLU space for storing output data.
 *           The number of the destination images must be equal to batch_size.
 * @param[in] workspace_size
 *   Input. Size of the extra space for storing source ROIs and destination ROIs on the MLU.
 * @param[in] workspace
 *   Input. Pointer to the extra space for storing source ROIs and destination ROIs on the MLU.
 * @param[in] interpolation
 *   Input. Interpolation method. The interpolation method now supports CNCV_INTER_BILINEAR.
 * @retval CNCV_STATUS_SUCCESS
 *   The function ends normally.
 * @retval CNCV_STATUS_NOT_SUPPORTED
 *   The pointer of dst_rois is not NULL.
 * @note
 * - None.
 * @par Requirements
 * - None.
 * @par Example
 * - None.
 *
 */
cncvStatus_t cncvResizeRgbx(cncvHandle_t handle,
                            uint32_t batch_size,
                            const cncvImageDescriptor src_desc,
                            const cncvRect *src_rois,
                            void **src,
                            const cncvImageDescriptor dst_desc,
                            const cncvRect *dst_rois,
                            void **dst,
                            const size_t workspace_size,
                            void *workspace,
                            cncvInterpolation interpolation);

/*!
 * @brief
 *
 * @deprecated. This API will be deleted in future version. cncvGetResizeYuvWorkspaceSize is recommended to use.
 *
 * This function gets the size of workspace for the ::cncvResizeYuv420sp function.
 *
 * @param[in] batch_size
 *   Input. The number of images in batch.
 * @param[out] workspacesize
 *   Output. The value of workspace size.
 * @retval CNCV_STATUS_SUCCESS
 *   The function ends normally.
 * @note
 * - None.
 * @par Requirements
 * - None.
 * @par Example
 * - None.
 *
 */
cncvStatus_t cncvGetResizeYuv420spWorkspaceSize(const uint32_t batch_size,
                                                size_t *workspace_size);
/*!
 * @brief
 *
 * This function gets the size of workspace for the ::cncvResizeYuv function.
 *
 * @param[in] batch_size
 *   Input. The number of images in batch.
 * @param[in] src_desc
 *   Input. Pointer to description (width, height, stride, pixel_fmt, color_space, depth) of the source images.
 *          Members width, height, and stride for each source images can be different, but other members must be the same.
 *          The number of source image descriptions must be equal to batch_size.
 *          The member depth in current structure supports CNCV_DEPTH_8U.
 *          The member pixel_fmt in current structure supports CNCV_PIX_FMT_NV12 and CNCV_PIX_FMT_NV21.
 * @param[in] src_rois
 *   Input. Pointer to ROI (x, y, w, h) of source images.
 *          The number of source image ROIs must be equal to batch_size.
 * @param[in] dst_desc
 *   Input. Pointer to description (width, height, stride, pixel_fmt, color_space, depth) of the source images.
 *          Members width, height, and stride for each destination images can be different, but other members must be the same.
 *          The number of destination image descriptions must be equal to batch_size.
 *          The member depth in current structure supports CNCV_DEPTH_8U.
 *          The member pixel_fmt in current structure supports CNCV_PIX_FMT_NV12 and CNCV_PIX_FMT_NV21.
 * @param[in] dst_rois
 *   Input. Pointer to ROI (x, y, w, h) of destination images.
 *          The number of destination image ROIs must be equal to batch_size.
 * @param[out] workspacesize
 *   Output. The value of workspace size.
 * @retval CNCV_STATUS_SUCCESS
 *   The function ends normally.
 * @note
 * - None.
 * @par Requirements
 * - None.
 * @par Example
 * - None.
 *
 */
cncvStatus_t cncvGetResizeYuvWorkspaceSize(const uint32_t batch_size,
                                           const cncvImageDescriptor *src_desc,
                                           const cncvRect *src_rois,
                                           const cncvImageDescriptor *dst_desc,
                                           const cncvRect *dst_rois,
                                           size_t *workspace_size);
/*!
 * @brief
 *
 * @deprecated. This API will be deleted in future version. cncvResizeYuv is recommended to use.
 *
 * This function resizes YUV image data.
 *
 * @param[in] handle
 *   Input. Pointer to the CNCV context.
 * @param[in] batch_size
 *   Input. The number of images in batch.
 * @param[in] src_desc
 *   Input. Description (width, height, stride, pixel_fmt, color_space, depth) of the source images.
 *          The description of each source images must be the same.
 *          The member pixel_fmt supports CNCV_PIX_FMT_NV12 and CNCV_PIX_FMT_NV21.
 *          The member depth supports CNCV_DEPTH_8U.
 * @param[in] src_rois
 *   Input. Pointer to the CPU space for storing ROIs on source images.
 *          The number of ROIs must be equal to batch_size. The ROIs of each
 *          batch can be different.
 * @param[in] src_y
 *   Input. Pointer to source image data of plane Y on the MLU.
 *          The number of source images must be equal to batch_size.
 * @param[in] src_uv
 *   Input. Pointer to source image data of plane UV on the MLU.
 *          The number of source images must be equal to batch_size.
 * @param[in] dst_desc
 *   Input. Description (width, height, stride, pixel_fmt, color_space, depth) of the destination images.
 *          The member pixel_fmt supports CNCV_PIX_FMT_NV12 and CNCV_PIX_FMT_NV21.
 *          The member depth supports CNCV_DEPTH_8U.
 * @param[out] dst_y
 *   Output. Pointer to the MLU space for storing output data of plane Y.
 *           The number of destination images must be equal to batch_size.
 * @param[out] dst_uv
 *   Output. Pointer to the MLU space for storing output data of plane UV.
 *           The number of destination images must be equal to batch_size.
 * @param[in] dst_rois
 *   Input. Pointer to the CPU space for storing ROIs on destination images.
 *          The number of ROIs must be equal to batch_size.
 * @param[in] workspace_size
 *   Input. Size of the extra space for storing source ROIs and destination ROIs on the MLU.
 * @param[in] workspace
 *   Input. Pointer to the extra space for storing source ROIs and destination ROIs on the MLU.
 * @param[in] interpolation
 *   Input. Interpolation method. The interpolation method now supports CNCV_INTER_BILINEAR.
 * @retval CNCV_STATUS_SUCCESS
 *   The function ends normally.
 * @note
 * - None.
 * @par Requirements
 * - None.
 * @par Example
 * - None.
 *
 */
cncvStatus_t cncvResizeYuv420sp(cncvHandle_t handle,
                                const uint32_t batch_size,
                                const cncvImageDescriptor src_desc,
                                const cncvRect  *src_rois,
                                void **src_y,
                                void **src_uv,
                                const cncvImageDescriptor dst_desc,
                                void **dst_y,
                                void **dst_uv,
                                const cncvRect *dst_rois,
                                const size_t workspace_size,
                                void *workspace,
                                cncvInterpolation interpolation);

/*!
 * @brief
 *
 * This function resizes YUV image data.
 *
 * @param[in] handle
 *   Input. Pointer to the CNCV context.
 * @param[in] batch_size
 *   Input. The number of images in batch.
 * @param[in] src_desc
 *   Input. Description (width, height, stride, pixel_fmt, color_space, depth) of the source images.
 *          Members width, height, and stride for each source images can be different, but other members must be the same.
 *          The member pixel_fmt supports CNCV_PIX_FMT_NV12 and CNCV_PIX_FMT_NV21.
 *          The member depth supports CNCV_DEPTH_8U.
 * @param[in] src_rois
 *   Input. Pointer to the CPU space for storing ROIs on source images.
 *          The number of ROIs must be equal to batch_size. The ROIs of each
 *          batch can be different.
 * @param[in] src
 *   Input. Pointer to source image data of plane Y and UV on the MLU.
 *          The number of source images must be equal to batch_size.
 * @param[in] dst_desc
 *   Input. Description (width, height, stride, pixel_fmt, color_space, depth) of the destination images.
 *          Members width, height, and stride for each destination images can be different, but other members must be the same.
 *          The member pixel_fmt supports CNCV_PIX_FMT_NV12 and CNCV_PIX_FMT_NV21.
 *          The member depth supports CNCV_DEPTH_8U.
 * @param[out] dst
 *   Output. Pointer to the MLU space for storing output data of plane Y and UV.
 *           The number of destination images must be equal to batch_size.
 * @param[in] dst_rois
 *   Input. Pointer to the CPU space for storing ROIs on destination images.
 *          The number of ROIs must be equal to batch_size.
 * @param[in] workspace_size
 *   Input. Size of the extra space for storing source ROIs and destination ROIs on the MLU.
 * @param[in] workspace
 *   Input. Pointer to the extra space for storing source ROIs and destination ROIs on the MLU.
 * @param[in] interpolation
 *   Input. Interpolation method. The interpolation method now supports CNCV_INTER_BILINEAR.
 * @retval CNCV_STATUS_SUCCESS
 *   The function ends normally.
 * @note
 * - None.
 * @par Requirements
 * - None.
 * @par Example
 * - None.
 *
 */
cncvStatus_t cncvResizeYuv(cncvHandle_t handle,
                           const uint32_t batch_size,
                           const cncvImageDescriptor *src_desc,
                           const cncvRect *src_rois,
                           void **src,
                           const cncvImageDescriptor *dst_desc,
                           void **dst,
                           const cncvRect *dst_rois,
                           const size_t workspace_size,
                           void *workspace,
                           cncvInterpolation interpolation);

/*!
 * @brief
 *
 * This function applies an affine transformation to an image by using the
 * specified matrix dst(x, y) = src(M11*x + M12*y + M13, M21*x + M22*y + M23).
 *
 * @param[in] handle
 *   Input. Pointer to the CNCV context.
 * @param[in] batch_size
 *   Input. The number of images in batch.
 * @param[in] mat
 *   Input. Pointer to the transformation matrices on the MLU. Each matrix has six
 *   elements. The number of matrices must be equal to batch_size.
 * @param[in] src_desc
 *   Input. Description (width, height, stride, color_space, depth, pixel_fmt) of the source image.
 *   It has the following limitations:
 *   - 2 <= width <= 8192, 2 <= height <= 8192.
 *   - The member depth now supports CNCV_DEPTH_8U.
 *   - The member pixel_fmt now supports CNCV_PIX_FMT_RGB,  CNCV_PIX_FMT_BGR,
 *                                       CNCV_PIX_FMT_RGBA, CNCV_PIX_FMT_BGRA,
 *                                       CNCV_PIX_FMT_ARGB, CNCV_PIX_FMT_ABGR,
 *                                       CNCV_PIX_FMT_GRAY, and CNCV_PIX_FMT_LAB.
 * @param[in] src
 *   Input. Pointer to the source image data on the MLU.
 * @param[in] dst_desc
 *   Input. Description (width, height, stride, pixel_fmt, color_space, depth) of the destination images.
 *   It has the following limitations:
 *   - 0 < height <= 8192, 0 < width <= 8192 and width * channel_num <= 12288.
 *   - The member depth now supports CNCV_DEPTH_8U.
 *   - The member pixel_fmt now supports CNCV_PIX_FMT_RGB, CNCV_PIX_FMT_BGR,
 *                                       CNCV_PIX_FMT_RGBA, CNCV_PIX_FMT_BGRA,
 *                                       CNCV_PIX_FMT_ARGB, CNCV_PIX_FMT_ABGR,
 *                                       CNCV_PIX_FMT_GRAY, and CNCV_PIX_FMT_LAB.
 * @param[out] dst
 *   Output. Pointer to the destination image data on the MLU.
 * @retval CNCV_STATUS_SUCCESS
 *   The function ends normally.
 * @note
 * - None.
 * @par Requirements
 * - None.
 * @par Example
 * - None.
 *
 */
cncvStatus_t cncvWarpAffine(cncvHandle_t handle,
                            const uint32_t batch_size,
                            void **mat,
                            const cncvImageDescriptor src_desc,
                            void **src,
                            const cncvImageDescriptor dst_desc,
                            void **dst);

/*********************************************************************
 * Morphological Operations
********************************************************************/
/*!
 * @brief
 *
 * This function processes gray images with the Dilate or Erode operation.
 *
 * @param[in] handle
 *   Input. Pointer to the CNCV context.
 * @param[in] kx
 *   Input. The width size of kernel. It must be > 0 && <= 25.
 * @param[in] ky
 *   Input. The height size of kernel. It must be > 0 && <= 25.
 * @param[in] op_type
 *   Input. The compute type of operation.
 *          It supports CNCV_DILATE_OP, CNCV_ERODE_OP.
 * @param[in] kernel_hape
 *   Input. The shape of kernel.
 *          It supports CNCV_DILATE_ERODE_RECT,
 *                       CNCV_DILATE_ERODE_CROSS,
 *                       CNCV_DILATE_ERODE_ELLIPSE.
 * @param[in] src_desc
 *   Input. Description (width, height, stride, depth) of input image data.
 *          The member depth now supports CNCV_DEPTH_8U and CNCV_DEPTH_32F.
 * @param[in] src
 *   Input. Pointer to source image data on the MLU.
 * @param[in] dst_desc
 *   Input. Description (width, height, stride, depth) of output image data.
 *          The member depth now supports CNCV_DEPTH_8U and CNCV_DEPTH_32F.
 *          It must be the same as src_desc.
 * @param[out] dst
 *   Output. Pointer to the MLU space for storing output data.
 * @retval CNCV_STATUS_SUCCESS
 *   The function ends normally.
 * @note
 * - None.
 * @par Requirements
 * - None.
 * @par Example
 * - None.
 *
 */
cncvStatus_t cncvDilateErode_Gray(cncvHandle_t handle,
                                  const uint32_t kx,
                                  const uint32_t ky,
                                  const cncvDilateErodeType_t op_type,
                                  const cncvDilateErodeShape_t kernel_shape,
                                  const cncvImageDescriptor src_desc,
                                  const void *src,
                                  const cncvImageDescriptor dst_desc,
                                  void *dst);


/*********************************************************************
 * Threshold and Compare Operations
********************************************************************/
/*!
 * @brief
 *
 * This function converts gray images into binary images with the Threshold operation.
 *
 * @param[in] handle
 *   Input. Pointer to the CNCV context.
 * @param[in] thresh
 *   Input. The thresh parameter of threshold.
 * @param[in] maxval
 *   Input. The maxval parameter of threshold.
 * @param[in] type
 *   Input. The type parameter of threshold.
 * @param[in] src_desc
 *   Input. Description (width, height, stride, depth, pixel_fmt) of input image data.
 * @param[in] src
 *   Input. Pointer to source image data on the MLU.
 * @param[in] dst_desc
 *   Input. Description (width, height, stride, depth, pixel_fmt) of output image data.
 * @param[out] dst
 *   Output. Pointer to the MLU space for storing output data.
 * @retval CNCV_STATUS_SUCCESS
 *   The function ends normally.
 * @note
 * - None.
 * @par Requirements
 * - None.
 * @par Example
 * - None.
 *
 */
cncvStatus_t cncvThreshold_Gray(cncvHandle_t handle,
                                const float thresh,
                                const float maxval,
                                const cncvThreshType type,
                                const cncvImageDescriptor src_desc,
                                const void *src,
                                const cncvImageDescriptor dst_desc,
                                void *dst);


/*********************************************************************
* Fusion Operations
********************************************************************/
/*!
 * @brief
 *
 * This function gets the size of workspace for the cncvResizeConvert function.
 *
 * @param[in] batch_size
 *   Input. The number of images in batch.
 * @param[in] psrc_descs
 *   Input. Pointer to description (width, height, stride, pixel_fmt, color_space, depth) of the source images.
 *          Members width, height, and stride for each source images can be different, but other members must be the same.
 *          The number of source image descriptions must be equal to batch_size.
 *          The member depth in current structure supports CNCV_DEPTH_8U.
 *          The member color_space in current structure supports CNCV_COLOR_SPACE_BT_601 and CNCV_COLOR_SPACE_BT_709.
 *          The member pixel_fmt in current structure supports CNCV_PIX_FMT_NV12, CNCV_PIX_FMT_NV21, and CNCV_PIX_FMT_GRAY.
 * @param[in] src_rois
 *   Input. Pointer to ROI (x, y, w, h) of source images.
 *          The number of source image ROIs must be equal to batch_size.
 * @param[in] pdst_descs
 *   Input. Pointer to description (width, height, stride, pixel_fmt, color_space, depth) of the destination images.
 *          Members width, height, and stride for each destination images can be different, but other members must be the same.
 *          The number of destination image descriptions must be equal to batch_size.
 *          The member depth in current structure supports CNCV_DEPTH_8U.
 *          The member color_space in current structure supports CNCV_COLOR_SPACE_BT_601 and CNCV_COLOR_SPACE_BT_709.
 *          The member pixel_fmt in current structure supports CNCV_PIX_FMT_RGBA, CNCV_PIX_FMT_BGRA, CNCV_PIX_FMT_ARGB, CNCV_PIX_FMT_ABGR,
 *          CNCV_PIX_FMT_RGB, CNCV_PIX_FMT_BGR, and CNCV_PIX_FMT_GRAY.
 * @param[in] dst_rois
 *   Input. Pointer to ROI (x, y, w, h) of destination images.
 *          The number of destination image ROIs must be equal to batch_size.
 * @param[out] size
 *   Output. The value of workspace size.
 * @retval CNCV_STATUS_SUCCESS
 *   The function ends normally.
 * @note
 * - None.
 * @par Requirements
 * - None.
 * @par Example
 * - None.
 *
 */
cncvStatus_t cncvGetResizeConvertWorkspaceSize(const uint32_t batch_size,
                                               const cncvImageDescriptor *psrc_descs,
                                               const cncvRect *src_rois,
                                               const cncvImageDescriptor *pdst_descs,
                                               const cncvRect *dst_rois,
                                               size_t *workspace_size);

/*!
 * @brief
 *
 * @deprecated. This API will be deleted in future version. cncvResizeConvert_V2 is recommended to use.
 *
 * This function converts YUV images to RGBX images and then resizes
 * them. You can also use this function to resize gray2gray images.
 *
 * @param[in] handle
     Input. Pointer to the CNCV context.
 * @param[in] batch_size
 *   Input. The number of images in batch.
 * @param[in] psrc_descs
 *   Input. Pointer to description (width, height, stride, pixel_fmt, color_space, depth) of the source images.
 *          Members width, height, and stride for each source images can be different, but other members must be the same.
 *          The number of source image descriptions must be equal to batch_size.
 *          The member depth in current structure supports CNCV_DEPTH_8U.
 *          The member color_space in current structure supports CNCV_COLOR_SPACE_BT_601 and CNCV_COLOR_SPACE_BT_709.
 *          The member pixel_fmt in current structure supports CNCV_PIX_FMT_NV12, CNCV_PIX_FMT_NV21,
 *          and CNCV_PIX_FMT_GRAY.
 * @param[in] src_rois
 *   Input. Pointer to ROI (x, y, w, h) of source images.
 *          The number of source image ROIs must be equal to batch_size.
 * @param[in] src_y
 *   Input. Pointer to source image y data on the MLU.
 *          The number of source image y data must be equal to batch_size.
 * @param[in] src_uv
 *   Input. Pointer to source image uv data on the MLU.
 *          The number of source image uv data must be equal to batch_size.
 * @param[in] pdst_descs
 *   Input. Pointer to description (width, height, stride, pixel_fmt, color_space, depth) of the destination images.
 *          Members width, height, and stride for each destination images can be different, but other members must be the same.
 *          The number of destination image descriptions must be equal to batch_size.
 *          The member depth in current structure supports CNCV_DEPTH_8U.
 *          The member color_space in current structure supports CNCV_COLOR_SPACE_BT_601 and CNCV_COLOR_SPACE_BT_709.
 *          The member pixel_fmt in current structure supports
 *                         CNCV_PIX_FMT_RGBA, CNCV_PIX_FMT_BGRA,
 *                         CNCV_PIX_FMT_ARGB, CNCV_PIX_FMT_ABGR,
 *                         CNCV_PIX_FMT_RGB, CNCV_PIX_FMT_BGR,
 *                         and CNCV_PIX_FMT_GRAY.
 * @param[in] dst_rois
 *   Input. Pointer to ROI (x, y, w, h) of destination images.
 *          The number of destination image ROIs must be equal to batch_size.
 * @param[out] dst
 *   Output. Pointer to the MLU space for storing output data.
 *           The number of destination images must be equal to batch_size.
 * @param[in] workspace_size
 *   Input. Size of the extra space to resize and convert on the MLU.
 * @param[in] workspace
 *   Input. Pointer to the extra space to resize and convert on the MLU.
 * @param[in] interpolation
 *   Input. Interpolation method. It supports CNCV_INTER_BILINEAR.
 * @retval CNCV_STATUS_SUCCESS
 *   The function ends normally.
 * @note
 * - None.
 * @par Requirements
 * - None.
 * @par Example
 * - None.
 *
 */
cncvStatus_t cncvResizeConvert(cncvHandle_t handle,
                               const uint32_t batch_size,
                               const cncvImageDescriptor *psrc_descs,
                               const cncvRect *src_rois,
                               void **src_y,
                               void **src_uv,
                               const cncvImageDescriptor *pdst_descs,
                               const cncvRect *dst_rois,
                               void **dst,
                               const size_t workspace_size,
                               void *workspace,
                               cncvInterpolation interpolation);

/*!
 * @brief
 *
 * This function converts YUV images to RGBX images and then resizes
 * them. You can also use this function to resize gray2gray images.
 *
 * @param[in] handle
     Input. Pointer to the CNCV context.
 * @param[in] batch_size
 *   Input. The number of images in batch.
 * @param[in] psrc_descs
 *   Input. Pointer to description (width, height, stride, pixel_fmt, color_space, depth) of the source images.
 *          Members width, height, and stride for each source images can be different, but other members must be the same.
 *          The number of source image descriptions must be equal to batch_size.
 *          The member depth in current structure supports CNCV_DEPTH_8U.
 *          The member color_space in current structure supports CNCV_COLOR_SPACE_BT_601 and CNCV_COLOR_SPACE_BT_709.
 *          The member pixel_fmt in current structure supports CNCV_PIX_FMT_NV12, CNCV_PIX_FMT_NV21,
 *          and CNCV_PIX_FMT_GRAY.
 * @param[in] src_rois
 *   Input. Pointer to ROI (x, y, w, h) of source images.
 *          The number of source image ROIs must be equal to batch_size.
 * @param[in] src
 *   Input. Pointer to source image data of plane Y and UV data on the MLU.
 *          The number of source images must be equal to batch_size.
 * @param[in] pdst_descs
 *   Input. Pointer to description (width, height, stride, pixel_fmt, color_space, depth) of the destination images.
 *          Members width, height, and stride for each destination images can be different, but other members must be the same.
 *          The number of destination image descriptions must be equal to batch_size.
 *          The member depth in current structure supports CNCV_DEPTH_8U.
 *          The member color_space in current structure supports CNCV_COLOR_SPACE_BT_601 and CNCV_COLOR_SPACE_BT_709.
 *          The member pixel_fmt in current structure supports
 *                         CNCV_PIX_FMT_RGBA, CNCV_PIX_FMT_BGRA,
 *                         CNCV_PIX_FMT_ARGB, CNCV_PIX_FMT_ABGR,
 *                         CNCV_PIX_FMT_RGB, CNCV_PIX_FMT_BGR,
 *                         and CNCV_PIX_FMT_GRAY.
 * @param[in] dst_rois
 *   Input. Pointer to ROI (x, y, w, h) of destination images.
 *          The number of destination image ROIs must be equal to batch_size.
 * @param[out] dst
 *   Output. Pointer to the MLU space for storing output data.
 *           The number of destination images must be equal to batch_size.
 * @param[in] workspace_size
 *   Input. Size of the extra space to resize and convert on the MLU.
 * @param[in] workspace
 *   Input. Pointer to the extra space to resize and convert on the MLU.
 * @param[in] interpolation
 *   Input. Interpolation method. It supports CNCV_INTER_BILINEAR.
 * @retval CNCV_STATUS_SUCCESS
 *   The function ends normally.
 * @note
 * - None.
 * @par Requirements
 * - None.
 * @par Example
 * - None.
 *
 */
cncvStatus_t cncvResizeConvert_V2(cncvHandle_t handle,
                                  const uint32_t batch_size,
                                  const cncvImageDescriptor *psrc_descs,
                                  const cncvRect *src_rois,
                                  void **src,
                                  const cncvImageDescriptor *pdst_descs,
                                  const cncvRect *dst_rois,
                                  void **dst,
                                  const size_t workspace_size,
                                  void *workspace,
                                  cncvInterpolation interpolation);

#if defined(__cplusplus)
}
#endif

#endif  // CNCV_INCLUDE_CNCV_H_
