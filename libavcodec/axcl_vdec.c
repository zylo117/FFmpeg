/*
 * Axera CL Image Video Process System - Video Decoder
 *
 * Copyright (c) 2025 Zylo117
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */
#include <pthread.h>
#include <time.h>
#include <unistd.h>
#include <sys/time.h>

#include "avcodec.h"
#include "codec_internal.h"
#include "internal.h"
#include "decode.h"
#include "hwconfig.h"
#include "libavutil/buffer.h"
#include "libavutil/common.h"
#include "libavutil/frame.h"
#include "libavutil/hwcontext.h"
#include "libavutil/hwcontext_drm.h"
#include "libavutil/imgutils.h"
#include "libavutil/log.h"

#include "axcl/axcl_base.h"
#include "axcl/axcl.h"
#include "axcl/ax_buffer_tool.h"

#if CONFIG_LIBRGA
#include <rga/rga.h>
#include <rga/RgaApi.h>
#endif

// HACK: Older BSP kernel use NA12 for NV15.
#ifndef DRM_FORMAT_NV15 // fourcc_code('N', 'V', '1', '5')
#define DRM_FORMAT_NV15 fourcc_code('N', 'A', '1', '2')
#endif

#define FPS_UPDATE_INTERVAL     120
#define ALIGN_UP(x, align) (((x) + ((align) - 1)) & ~((align) - 1))
#define AX_SHIFT_LEFT_ALIGN(a) (1 << (a))
#define VDEC_STRIDE_ALIGN AX_SHIFT_LEFT_ALIGN(8) /* VDEC stride align 256 */
# define AXCL_SEND_STREAM_TIMEOUT (-1)

typedef struct {
    AX_BOOL enable;
    AX_VDEC_CHN_ATTR_T attr;
} SAMPLE_VDEC_CHN_ATTR;

typedef struct {
    AX_VDEC_MODE_E decoded_mode;
    AX_VDEC_OUTPUT_ORDER_E output_order;
    AX_VDEC_DISPLAY_MODE_E display_mode;
    AX_VDEC_GRP_ATTR_T grp_attr;
    SAMPLE_VDEC_CHN_ATTR chn_attr[AX_VDEC_MAX_CHN_NUM];
} SAMPLE_VDEC_ATTR;

typedef struct {
    AX_U32 u32ChnId;
    AX_U32 u32PicWidth;
    AX_U32 u32PicHeight;
} SAMPLE_VDEC_CHN_INFO;

typedef struct {
    MppCtx ctx;
    MppApi *mpi;
    MppBufferGroup frame_group;

    int8_t eos;
    int8_t draining;

    AVPacket packet;
    AVBufferRef *frames_ref;
    AVBufferRef *device_ref;

    char print_fps;

    uint64_t last_fps_time;
    uint64_t frames;

    char sync;
} AXCLDecoder;

typedef struct {
    AVClass *av_class;
    AXCLDecoder *decoder;           ///< RefStruct reference
    AX_VDEC_GRP grp;
    SAMPLE_VDEC_CHN_INFO chn_info;
    int32_t device_id;
    axclrtContext context;
} AXCLDecodeContext;

typedef struct {
    MppFrame frame;
    AVBufferRef *decoder_ref;
} RKMPPFrameContext;

struct stream_info {
    struct video_info {
        AX_PAYLOAD_TYPE_E payload;
        uint32_t width;
        uint32_t height;
        uint32_t fps;
    } video;

    struct audio_info {
        AX_PAYLOAD_TYPE_E payload;
        uint32_t bps;
        uint32_t sample_rate;
        uint32_t sample_bits;
    } audio;
};

static MppCodingType rkmpp_get_codingtype(AVCodecContext *avctx)
{
    switch (avctx->codec_id) {
        case AV_CODEC_ID_H263:          return MPP_VIDEO_CodingH263;
        case AV_CODEC_ID_H264:          return MPP_VIDEO_CodingAVC;
        case AV_CODEC_ID_HEVC:          return MPP_VIDEO_CodingHEVC;
        case AV_CODEC_ID_AV1:           return MPP_VIDEO_CodingAV1;
        case AV_CODEC_ID_VP8:           return MPP_VIDEO_CodingVP8;
        case AV_CODEC_ID_VP9:           return MPP_VIDEO_CodingVP9;
        case AV_CODEC_ID_MPEG1VIDEO:    /* fallthrough */
        case AV_CODEC_ID_MPEG2VIDEO:    return MPP_VIDEO_CodingMPEG2;
        case AV_CODEC_ID_MPEG4:         return MPP_VIDEO_CodingMPEG4;
        default:                        return MPP_VIDEO_CodingUnused;
    }
}

static uint32_t rkmpp_get_frameformat(MppFrameFormat mppformat)
{
    switch (mppformat & MPP_FRAME_FMT_MASK) {
        case MPP_FMT_YUV420SP:          return DRM_FORMAT_NV12;
        case MPP_FMT_YUV420SP_10BIT:    return DRM_FORMAT_NV15;
        case MPP_FMT_YUV422SP:          return DRM_FORMAT_NV16;
        default:                        return 0;
    }
}

static uint32_t rkmpp_get_avformat(MppFrameFormat mppformat)
{
    switch (mppformat & MPP_FRAME_FMT_MASK) {
        case MPP_FMT_YUV420SP:          return AV_PIX_FMT_NV12;
        case MPP_FMT_YUV420SP_10BIT:    return AV_PIX_FMT_NONE;
        case MPP_FMT_YUV422SP:          return AV_PIX_FMT_NV16;
        default:                        return 0;
    }
}

#if CONFIG_LIBRGA
static uint32_t rkmpp_get_rgaformat(MppFrameFormat mppformat)
{
    switch (mppformat & MPP_FRAME_FMT_MASK) {
    case MPP_FMT_YUV420SP:          return RK_FORMAT_YCbCr_420_SP;
    case MPP_FMT_YUV420SP_10BIT:    return RK_FORMAT_YCbCr_420_SP_10B;
    case MPP_FMT_YUV422SP:          return RK_FORMAT_YCbCr_422_SP;
    default:                        return RK_FORMAT_UNKNOWN;
    }
}
#endif

static int axcl_close_decoder(AVCodecContext *avctx)
{
    RKMPPDecodeContext *rk_context = avctx->priv_data;
    RKMPPDecoder *decoder = (RKMPPDecoder *)rk_context->decoder_ref->data;

    av_packet_unref(&decoder->packet);

    av_buffer_unref(&rk_context->decoder_ref);
    return 0;
}

static void rkmpp_release_decoder(void *opaque, uint8_t *data)
{
    RKMPPDecoder *decoder = (RKMPPDecoder *)data;

    if (decoder->mpi) {
        decoder->mpi->reset(decoder->ctx);
        mpp_destroy(decoder->ctx);
        decoder->ctx = NULL;
    }

    if (decoder->frame_group) {
        mpp_buffer_group_put(decoder->frame_group);
        decoder->frame_group = NULL;
    }

    av_buffer_unref(&decoder->frames_ref);
    av_buffer_unref(&decoder->device_ref);

    av_free(decoder);
}

static int rkmpp_prepare_decoder(AVCodecContext *avctx)
{
    RKMPPDecodeContext *rk_context = avctx->priv_data;
    RKMPPDecoder *decoder = (RKMPPDecoder *)rk_context->decoder_ref->data;
    MppPacket packet;
    int ret;

    // HACK: somehow MPP cannot handle extra data for AV1
    if (avctx->extradata_size && avctx->codec_id != AV_CODEC_ID_AV1) {
        ret = mpp_packet_init(&packet, avctx->extradata, avctx->extradata_size);
        if (ret < 0)
            return AVERROR_UNKNOWN;
        ret = decoder->mpi->decode_put_packet(decoder->ctx, packet);
        mpp_packet_deinit(&packet);
        if (ret < 0)
            return AVERROR_UNKNOWN;
    }

    // wait for decode result after feeding any packets
    if (getenv("FFMPEG_RKMPP_SYNC"))
        decoder->sync = 1;
    return 0;
}

static AX_S32 sample_vdec_init(void) {
    AX_VDEC_MOD_ATTR_T attr;
    memset(&attr, 0, sizeof(attr));
    attr.u32MaxGroupCount = 32;
    attr.enDecModule = AX_ENABLE_BOTH_VDEC_JDEC;
    return AXCL_VDEC_Init(&attr);
}

static AX_S32 sample_vdec_deinit(void) {
    return AXCL_VDEC_Deinit();
}


static AX_S32 sample_vdec_get_frame(AX_VDEC_GRP grp, AX_VDEC_CHN chn, AX_VIDEO_FRAME_INFO_T *frame, AX_S32 timeout) {
    return AXCL_VDEC_GetChnFrame(grp, chn, frame, timeout);
}

static AX_S32 sample_vdec_release_frame(AX_VDEC_GRP grp, AX_VDEC_CHN chn, const AX_VIDEO_FRAME_INFO_T *frame) {
    return AXCL_VDEC_ReleaseChnFrame(grp, chn, frame);
}

static AX_S32 sample_vdec_send_stream(AX_VDEC_GRP grp, const AX_VDEC_STREAM_T *stream, AX_S32 timeout) {
    return AXCL_VDEC_SendStream(grp, stream, timeout);
}

static void on_receive_demux_stream_data(AVCodecContext *avctx, AX_VDEC_GRP grp, const struct stream_data *data) {
    AX_VDEC_STREAM_T stream;
    memset(&stream, 0, sizeof(stream));
    stream.u64PTS = data->video.pts;
    stream.u32StreamPackLen = data->video.size;
    stream.pu8Addr = data->video.data;
    stream.bEndOfFrame = AX_TRUE;
    if (0 == data->video.size) {
        stream.bEndOfStream = AX_TRUE;
    }

    AX_S32 ret = sample_vdec_send_stream(grp, &stream, -1);
    if (0 != ret) {
        av_log(avctx, AV_LOG_ERROR, "[decoder %2d] send stream (id: %ld, size: %u) fail, ret = 0x%x\n", grp, data->video.seq_num, data->video.size, ret);
    }
}

static int sample_get_decoded_image_thread(AVCodecContext *avctx, AVFrame *frame) {
    AXCLDecodeContext *axcl_context = avctx->priv_data;
    AX_VDEC_GRP grp = axcl_context->grp;
    SAMPLE_VDEC_CHN_INFO chn_info = axcl_context->chn_info;
    int32_t device_id = axcl_context->device_id;
    AXCLDecoder *decoder = axcl_context->decoder;

    axclError ret;

    /* step01: create thread context */
    axclrtContext context;
    ret = axclrtCreateContext(&context, device_id);
    if (AXCL_SUCC != ret) {
        return ret;
    }

    const AX_VDEC_CHN chn = chn_info.u32ChnId;

    const size_t size = ALIGN_UP(chn_info.u32PicWidth, VDEC_STRIDE_ALIGN) * chn_info.u32PicHeight * 3 / 2;
//    void* cma_allocator = dma_buffer_create();

    AX_VIDEO_FRAME_INFO_T axcl_frame;
    memset(&axcl_frame, 0, sizeof(axcl_frame));

    /* step02: get decoded image */
    ret = sample_vdec_get_frame(grp, chn, &axcl_frame, -1);
    if (0 != ret) {
        if (AX_ERR_VDEC_UNEXIST == ret) {
            av_log(avctx, AV_LOG_INFO, "[decoder %2d] grp is destroyed\n", grp);
            return AX_ERR_VDEC_UNEXIST;
        } else if (AX_ERR_VDEC_STRM_ERROR == ret) {
            av_log(avctx, AV_LOG_WARNING, "[decoder %2d] stream is undecodeable\n", grp);
            return AX_ERR_VDEC_STRM_ERROR;
        } else if (AX_ERR_VDEC_FLOW_END == ret) {
            av_log(avctx, AV_LOG_WARNING, "[decoder %2d] flow end\n", grp);
            return AX_ERR_VDEC_FLOW_END;
        } else {
            av_log(avctx, AV_LOG_ERROR, "[decoder %2d] get frame fail, ret = 0x%x\n", grp, ret);
            return ret;
        }
    }

    // TODO: axcl frame to ffmpeg frame

    /* TODO: */
    if (0 == ret) {
        /* step03: release decoded image */
        sample_vdec_release_frame(grp, chn, &axcl_frame);
    }

    /* step04: destroy thread context */
    axclrtDestroyContext(context);
//    cma_allocator.free();

    av_log(avctx, AV_LOG_INFO, "[decoder %2d] decode thread ---\n", grp);
}

static AX_U32 sample_vdec_calc_blk_size(AX_U32 width, AX_U32 height, AX_PAYLOAD_TYPE_E payload, AX_FRAME_COMPRESS_INFO_T *fbc,
                                 AX_IMG_FORMAT_E pix_fmt) {
    return AX_VDEC_GetPicBufferSize(width, height, pix_fmt, fbc, payload);
}

static SAMPLE_VDEC_ATTR sample_get_vdec_attr_from_stream_info(AVCodecContext *avctx, const struct stream_info *info) {
    av_log(avctx, AV_LOG_INFO, "stream info: %dx%d payload %d fps %d", info->video.width, info->video.height, info->video.payload, info->video.fps);

    SAMPLE_VDEC_ATTR vdec_attr;
    memset(&vdec_attr, 0, sizeof(vdec_attr));

    vdec_attr.decoded_mode = VIDEO_DEC_MODE_IPB;
    vdec_attr.output_order = AX_VDEC_OUTPUT_ORDER_DEC;
    vdec_attr.display_mode = AX_VDEC_DISPLAY_MODE_PLAYBACK;

    vdec_attr.grp_attr.enCodecType = info->video.payload;
    vdec_attr.grp_attr.enInputMode = AX_VDEC_INPUT_MODE_FRAME;
    vdec_attr.grp_attr.u32MaxPicWidth = ALIGN_UP(info->video.width, 16);
    vdec_attr.grp_attr.u32MaxPicHeight = ALIGN_UP(info->video.height, 16);
    vdec_attr.grp_attr.u32StreamBufSize = vdec_attr.grp_attr.u32MaxPicWidth * vdec_attr.grp_attr.u32MaxPicHeight * 2;
    vdec_attr.grp_attr.bSdkAutoFramePool = AX_TRUE;

    vdec_attr.chn_attr[0].enable = AX_TRUE;
    vdec_attr.chn_attr[1].enable = AX_FALSE;
    vdec_attr.chn_attr[2].enable = AX_FALSE;
    vdec_attr.chn_attr[0].attr.u32PicWidth = info->video.width;
    vdec_attr.chn_attr[0].attr.u32PicHeight = info->video.height;
    vdec_attr.chn_attr[0].attr.u32FrameStride = ALIGN_UP(vdec_attr.chn_attr[0].attr.u32PicWidth, VDEC_STRIDE_ALIGN);
    vdec_attr.chn_attr[0].attr.u32OutputFifoDepth = 3;
    vdec_attr.chn_attr[0].attr.u32FrameBufCnt = 8;
    vdec_attr.chn_attr[0].attr.stCompressInfo.enCompressMode = AX_COMPRESS_MODE_NONE;
    vdec_attr.chn_attr[0].attr.stCompressInfo.u32CompressLevel = 0;
    vdec_attr.chn_attr[0].attr.enOutputMode = AX_VDEC_OUTPUT_ORIGINAL;
    vdec_attr.chn_attr[0].attr.enImgFormat = AX_FORMAT_YUV420_SEMIPLANAR;
    vdec_attr.chn_attr[0].attr.u32FrameBufSize = sample_vdec_calc_blk_size(vdec_attr.chn_attr[0].attr.u32PicWidth,
                                                                           vdec_attr.chn_attr[0].attr.u32PicHeight,
                                                                           info->video.payload,
                                                                           &vdec_attr.chn_attr[0].attr.stCompressInfo,
                                                                           vdec_attr.chn_attr[0].attr.enImgFormat);

    return vdec_attr;
}

static AX_S32 sample_vdec_create_grp(AX_VDEC_GRP grp, const SAMPLE_VDEC_ATTR *attr) {
    axclError ret;
    ret = AXCL_VDEC_CreateGrp(grp, &attr->grp_attr);
    if (AXCL_SUCC != ret) {
        return ret;
    }

    do {
        AX_VDEC_GRP_PARAM_T param;
        memset(&param, 0, sizeof(param));
        param.stVdecVideoParam.enVdecMode = attr->decoded_mode;
        param.stVdecVideoParam.enOutputOrder = attr->output_order;
        ret = AXCL_VDEC_SetGrpParam(grp, &param);
        if (AXCL_SUCC != ret) {
            break;
        }

        ret = AXCL_VDEC_SetDisplayMode(grp, attr->display_mode);
        if (AXCL_SUCC != ret) {
            break;
        }

        for (AX_VDEC_CHN chn = 0; chn < AX_VDEC_MAX_CHN_NUM; ++chn) {
            if (attr->chn_attr[chn].enable) {
                ret = AXCL_VDEC_SetChnAttr(grp, chn, &attr->chn_attr[chn].attr);
                if (AXCL_SUCC != ret) {
                    break;
                }

                ret = AXCL_VDEC_EnableChn(grp, chn);
                if (AXCL_SUCC != ret) {
                    break;
                }
            }
        }

        if (AXCL_SUCC != ret) {
            break;
        }

        return 0;

    } while (0);

    AXCL_VDEC_DestroyGrp(grp);
    return ret;
}

static AX_S32 sample_vdec_destory_grp(AX_VDEC_GRP grp) {
    axclError ret;
    for (AX_VDEC_CHN chn = 0; chn < AX_VDEC_MAX_CHN_NUM; ++chn) {
        ret = AXCL_VDEC_DisableChn(grp, chn);
        if (AXCL_SUCC != ret) {
            return ret;
        }
    }

    ret = AXCL_VDEC_DestroyGrp(grp);
    if (AXCL_SUCC != ret) {
        return ret;
    }

    return 0;
}

static AX_S32 sample_vdec_start(AX_VDEC_GRP grp, const SAMPLE_VDEC_ATTR *attr) {
    axclError ret;
    ret = sample_vdec_create_grp(grp, attr);
    if (0 != ret) {
        return ret;
    }

    AX_VDEC_RECV_PIC_PARAM_T param;
    memset(&param, 0, sizeof(param));
    param.s32RecvPicNum = -1;
    ret = AXCL_VDEC_StartRecvStream(grp, &param);
    if (AXCL_SUCC != ret) {
        sample_vdec_destory_grp(grp);
        return ret;
    }

    return 0;
}

static AX_S32 sample_vdec_stop(AX_VDEC_GRP grp) {
    axclError ret;
    ret = AXCL_VDEC_StopRecvStream(grp);
    if (AXCL_SUCC != ret) {
        return ret;
    }

    int32_t retry = 0;
    while (++retry <= 10) {
        ret = AXCL_VDEC_ResetGrp(grp);
        if (AXCL_SUCC != ret) {
            usleep(40 * 1000);
            continue;
        } else {
            break;
        }
    }

    return sample_vdec_destory_grp(grp);
}

// step1
static int axcl_init_decoder(AVCodecContext *avctx)
{
    // constants
    const char *json = "./axcl.json";
    int32_t device_id = 0;
    int32_t count = 1;
    int32_t chn_id = 0;

    axclError ret;

    AXCLDecodeContext *axcl_context = avctx->priv_data;

    /* step01: axcl initialize */
    av_log(avctx, AV_LOG_INFO, "json: %s\n", json);
    ret = axclInit(json);
    if (AXCL_SUCC != ret) {
        av_log(avctx, AV_LOG_ERROR, "axcl init fail, ret = 0x%x\n", ret);
        return ret;
    }

    if (device_id <= 0) {
        axclrtDeviceList lst;
        ret = axclrtGetDeviceList(&lst);
        if (AXCL_SUCC != ret || 0 == lst.num) {
            av_log(avctx, AV_LOG_ERROR, "no device is connected\n");
            axclFinalize();
            return ret;
        }

        device_id = lst.devices[0];
        av_log(avctx, AV_LOG_INFO, "device id: %d\n", device_id);
    }

    /* step02: active device */
    av_log(avctx, AV_LOG_INFO, "active device %d\n", device_id);
    ret = axclrtSetDevice(device_id);
    if (AXCL_SUCC != ret) {
        av_log(avctx, AV_LOG_ERROR, "active device, ret = 0x%x\n", ret);
        axclFinalize();
        return ret;
    }

    // get default context
    axclrtContext context;
    axclrtGetCurrentContext(&context);
    axcl_context->context = context;

    /* step04: init sys module */
    av_log(avctx, AV_LOG_INFO, "init sys\n");
    ret = AXCL_SYS_Init();
    if (0 != ret) {
        av_log(avctx, AV_LOG_ERROR, "init sys, ret = 0x%x\n", ret);
        axclrtResetDevice(device_id);
        axclFinalize();
        return ret;
    }

    /* step05: init vdec module */
    av_log(avctx, AV_LOG_INFO, "init vdec\n");
    ret = sample_vdec_init();
    if (0 != ret) {
        av_log(avctx, AV_LOG_ERROR, "init vdec, ret = 0x%x\n", ret);
        AXCL_SYS_Deinit();
        axclrtResetDevice(device_id);
        axclFinalize();
        return ret;
    }

    /* step06: start vdec */
//    axcl::threadx decode_threads[MAX_STREAM_COUNT];
//    axcl::event decode_eof_events[MAX_STREAM_COUNT];
    for (int32_t i = 0; i < count; ++i) {
        const AX_VDEC_GRP grp = i;
        av_log(avctx, AV_LOG_INFO, "start decoder %d\n", grp);

        struct stream_info this_stream_info;
        this_stream_info.video.width = avctx->width;
        this_stream_info.video.height = avctx->height;
        this_stream_info.video.payload =
                avctx->codec_id == AV_CODEC_ID_H264?PT_H264:
                avctx->codec_id == AV_CODEC_ID_HEVC?PT_H265:
                avctx->codec_id == AV_CODEC_ID_MJPEG?PT_MJPEG:PT_BUTT;
        SAMPLE_VDEC_ATTR attr = sample_get_vdec_attr_from_stream_info(avctx, &this_stream_info);

//        if (chn_id) {
//            sample_vdec_set_attr(&attr, chn_id, w, h);
//        }

        ret = sample_vdec_start(grp, &attr);
        if ( 0 != ret) {
            av_log(avctx, AV_LOG_ERROR, "start vdec %d fail, ret = 0x%x\n", grp, ret);

            for (int32_t j = 0; j < i; ++j) {
                sample_vdec_stop(j);
            }

            sample_vdec_deinit();
            AXCL_SYS_Deinit();
            axclrtResetDevice(device_id);
            axclFinalize();
        }

        /* step07: start decoded get thread */
        char name[32];
        SAMPLE_VDEC_CHN_INFO chn_info;
        memset(&chn_info, 0, sizeof(chn_info));
        chn_info.u32ChnId = chn_id;
        for (int32_t j = 0; j < AX_VDEC_MAX_CHN_NUM; ++j) {
            if (attr.chn_attr[j].enable) {
                chn_info.u32PicWidth = attr.chn_attr[j].attr.u32PicWidth;
                chn_info.u32PicHeight = attr.chn_attr[j].attr.u32PicHeight;
                break;
            }
        }
        sprintf(name, "decode%d", grp);

        av_log(avctx, AV_LOG_INFO, "[decoder %2d] decode thread +++\n", grp);

        axcl_context->grp = grp;
        axcl_context->chn_info = chn_info;
        axcl_context->device_id = device_id;
//
//        decode_threads[i].start(name, sample_get_decoded_image_thread, grp, device_id, &decode_eof_events[i], chn_info, dump);
//
//        /* step08: start to demux video */
//        SAMPLE_LOG_I("start demuxer %d", i);
//        ffmpeg_start_demuxer(demuxers[i]);
    }

    return ret;
}

static void rkmpp_release_frame(void *opaque, uint8_t *data)
{
    AVDRMFrameDescriptor *desc = (AVDRMFrameDescriptor *)data;
    AVBufferRef *framecontextref = (AVBufferRef *)opaque;
    RKMPPFrameContext *framecontext = (RKMPPFrameContext *)framecontextref->data;

    mpp_frame_deinit(&framecontext->frame);
    av_buffer_unref(&framecontext->decoder_ref);
    av_buffer_unref(&framecontextref);

    av_free(desc);
}

static int rkmpp_convert_frame(AVCodecContext *avctx, AVFrame *frame,
                               MppFrame mppframe, MppBuffer buffer)
{
    char *src = mpp_buffer_get_ptr(buffer);
    char *dst_y = frame->data[0];
    char *dst_u = frame->data[1];
    char *dst_v = frame->data[2];
#if CONFIG_LIBRGA
    RgaSURF_FORMAT format = rkmpp_get_rgaformat(mpp_frame_get_fmt(mppframe));
#endif
    int width = mpp_frame_get_width(mppframe);
    int height = mpp_frame_get_height(mppframe);
    int hstride = mpp_frame_get_hor_stride(mppframe);
    int vstride = mpp_frame_get_ver_stride(mppframe);
    int y_pitch = frame->linesize[0];
    int u_pitch = frame->linesize[1];
    int v_pitch = frame->linesize[2];
    int i, j;

#if CONFIG_LIBRGA
    rga_info_t src_info = {0};
    rga_info_t dst_info = {0};
    int dst_height = (dst_u - dst_y) / y_pitch;

    static int rga_supported = 1;
    static int rga_inited = 0;

    if (!rga_supported)
        goto bail;

    if (!rga_inited) {
        if (c_RkRgaInit() < 0) {
            rga_supported = 0;
            av_log(avctx, AV_LOG_WARNING, "RGA not available\n");
            goto bail;
        }
        rga_inited = 1;
    }

    if (format == RK_FORMAT_UNKNOWN)
        goto bail;

    if (u_pitch != y_pitch / 2 || v_pitch != y_pitch / 2 ||
        dst_u != dst_y + y_pitch * dst_height ||
        dst_v != dst_u + u_pitch * dst_height / 2)
        goto bail;

    src_info.fd = mpp_buffer_get_fd(buffer);
    src_info.mmuFlag = 1;
    rga_set_rect(&src_info.rect, 0, 0, width, height, hstride, vstride,
                 format);

    dst_info.virAddr = dst_y;
    dst_info.mmuFlag = 1;
    rga_set_rect(&dst_info.rect, 0, 0, frame->width, frame->height,
                 y_pitch, dst_height, RK_FORMAT_YCbCr_420_P);

    if (c_RkRgaBlit(&src_info, &dst_info, NULL) < 0)
        goto bail;

    return 0;

bail:
#endif
    if (mpp_frame_get_fmt(mppframe) != MPP_FMT_YUV420SP) {
        av_log(avctx, AV_LOG_WARNING, "Unable to convert\n");
        return -1;
    }

    av_log(avctx, AV_LOG_WARNING, "Doing slow software conversion\n");

    for (i = 0; i < frame->height; i++)
        memcpy(dst_y + i * y_pitch, src + i * hstride, frame->width);

    src += hstride * vstride;

    for (i = 0; i < frame->height / 2; i++) {
        for (j = 0; j < frame->width; j++) {
            dst_u[j] = src[2 * j + 0];
            dst_v[j] = src[2 * j + 1];
        }
        dst_u += u_pitch;
        dst_v += v_pitch;
        src += hstride;
    }

    return 0;
}

static void rkmpp_update_fps(AVCodecContext *avctx)
{
    RKMPPDecodeContext *rk_context = avctx->priv_data;
    RKMPPDecoder *decoder = (RKMPPDecoder *)rk_context->decoder_ref->data;
    struct timeval tv;
    uint64_t curr_time;
    float fps;

    if (!decoder->print_fps)
        return;

    if (!decoder->last_fps_time) {
        gettimeofday(&tv, NULL);
        decoder->last_fps_time = tv.tv_sec * 1000 + tv.tv_usec / 1000;
    }

    if (++decoder->frames % FPS_UPDATE_INTERVAL)
        return;

    gettimeofday(&tv, NULL);
    curr_time = tv.tv_sec * 1000 + tv.tv_usec / 1000;

    fps = 1000.0f * FPS_UPDATE_INTERVAL / (curr_time - decoder->last_fps_time);
    decoder->last_fps_time = curr_time;

    av_log(avctx, AV_LOG_INFO,
           "[FFMPEG RKMPP] FPS: %6.1f || Frames: %" PRIu64 "\n",
           fps, decoder->frames);
}

static int axcl_get_frame(AVCodecContext *avctx, AVFrame *frame)
{
    AXCLDecodeContext *axcl_context = avctx->priv_data;
    AX_VDEC_GRP grp = axcl_context->grp;
    SAMPLE_VDEC_CHN_INFO chn_info = axcl_context->chn_info;
    int32_t device_id = axcl_context->device_id;
    AXCLDecoder *decoder = axcl_context->decoder;
    AVPacket *packet = &decoder->packet;

    RKMPPFrameContext *framecontext = NULL;
    AVBufferRef *framecontextref = NULL;
    int ret;
    MppFrame mppframe = NULL;
    MppBuffer buffer = NULL;
    AVDRMFrameDescriptor *desc = NULL;
    AVDRMLayerDescriptor *layer = NULL;
    int mode;
    MppFrameFormat mppformat;
    uint32_t drmformat;

    // should not provide any frame after EOS
    if (decoder->eos)
        return AVERROR_EOF;

    decoder->mpi->control(decoder->ctx, MPP_SET_OUTPUT_TIMEOUT, (MppParam)&timeout);

    ret = decoder->mpi->decode_get_frame(decoder->ctx, &mppframe);
    if (ret != MPP_OK && ret != MPP_ERR_TIMEOUT) {
        av_log(avctx, AV_LOG_ERROR, "Failed to get frame (code = %d)\n", ret);
        return AVERROR_UNKNOWN;
    }

    if (!mppframe) {
        if (timeout != MPP_TIMEOUT_NON_BLOCK)
            av_log(avctx, AV_LOG_DEBUG, "Timeout getting decoded frame.\n");
        return AVERROR(EAGAIN);
    }

    if (mpp_frame_get_eos(mppframe)) {
        av_log(avctx, AV_LOG_DEBUG, "Received a EOS frame.\n");
        decoder->eos = 1;
        ret = AVERROR_EOF;
        goto fail;
    }

    if (mpp_frame_get_discard(mppframe)) {
        av_log(avctx, AV_LOG_DEBUG, "Received a discard frame.\n");
        ret = AVERROR(EAGAIN);
        goto fail;
    }

    if (mpp_frame_get_errinfo(mppframe)) {
        av_log(avctx, AV_LOG_ERROR, "Received a errinfo frame.\n");
        ret = AVERROR_UNKNOWN;
        goto fail;
    }

    if (mpp_frame_get_info_change(mppframe)) {
        AVHWFramesContext *hwframes;

        av_log(avctx, AV_LOG_INFO, "Decoder noticed an info change (%dx%d), format=%d\n",
               (int)mpp_frame_get_width(mppframe), (int)mpp_frame_get_height(mppframe),
               (int)mpp_frame_get_fmt(mppframe));

        avctx->width = mpp_frame_get_width(mppframe);
        avctx->height = mpp_frame_get_height(mppframe);

        // chromium would align planes' width and height to 32, adding this
        // hack to avoid breaking the plane buffers' contiguous.
        avctx->coded_width = FFALIGN(avctx->width, 64);
        avctx->coded_height = FFALIGN(avctx->height, 64);

        decoder->mpi->control(decoder->ctx, MPP_DEC_SET_INFO_CHANGE_READY, NULL);

        av_buffer_unref(&decoder->frames_ref);

        decoder->frames_ref = av_hwframe_ctx_alloc(decoder->device_ref);
        if (!decoder->frames_ref) {
            ret = AVERROR(ENOMEM);
            goto fail;
        }

        mppformat = mpp_frame_get_fmt(mppframe);

        hwframes = (AVHWFramesContext*)decoder->frames_ref->data;
        hwframes->format    = AV_PIX_FMT_DRM_PRIME;
        hwframes->sw_format = rkmpp_get_avformat(mppformat);
        hwframes->width     = avctx->width;
        hwframes->height    = avctx->height;
        ret = av_hwframe_ctx_init(decoder->frames_ref);
        if (!ret)
            ret = AVERROR(EAGAIN);

        goto fail;
    }

    // here we should have a valid frame
    av_log(avctx, AV_LOG_DEBUG, "Received a frame.\n");

    // now setup the frame buffer info
    buffer = mpp_frame_get_buffer(mppframe);
    if (!buffer) {
        av_log(avctx, AV_LOG_ERROR, "Failed to get the frame buffer, frame is dropped (code = %d)\n", ret);
        ret = AVERROR(EAGAIN);
        goto fail;
    }

    rkmpp_update_fps(avctx);

    if (avctx->pix_fmt != AV_PIX_FMT_DRM_PRIME) {
        ret = ff_get_buffer(avctx, frame, 0);
        if (ret < 0)
            goto out;
    }

    // setup general frame fields
    frame->format           = avctx->pix_fmt;
    frame->width            = mpp_frame_get_width(mppframe);
    frame->height           = mpp_frame_get_height(mppframe);
    frame->pts              = mpp_frame_get_pts(mppframe);
    frame->reordered_opaque = frame->pts;
    frame->color_range      = mpp_frame_get_color_range(mppframe);
    frame->color_primaries  = mpp_frame_get_color_primaries(mppframe);
    frame->color_trc        = mpp_frame_get_color_trc(mppframe);
    frame->colorspace       = mpp_frame_get_colorspace(mppframe);

    mode = mpp_frame_get_mode(mppframe);
    frame->interlaced_frame = ((mode & MPP_FRAME_FLAG_FIELD_ORDER_MASK) == MPP_FRAME_FLAG_DEINTERLACED);
    frame->top_field_first  = ((mode & MPP_FRAME_FLAG_FIELD_ORDER_MASK) == MPP_FRAME_FLAG_TOP_FIRST);

    if (avctx->pix_fmt != AV_PIX_FMT_DRM_PRIME) {
        ret = rkmpp_convert_frame(avctx, frame, mppframe, buffer);
        goto out;
    }

    mppformat = mpp_frame_get_fmt(mppframe);
    drmformat = rkmpp_get_frameformat(mppformat);

    desc = av_mallocz(sizeof(AVDRMFrameDescriptor));
    if (!desc) {
        ret = AVERROR(ENOMEM);
        goto fail;
    }

    desc->nb_objects = 1;
    desc->objects[0].fd = mpp_buffer_get_fd(buffer);
    desc->objects[0].size = mpp_buffer_get_size(buffer);

    desc->nb_layers = 1;
    layer = &desc->layers[0];
    layer->format = drmformat;
    layer->nb_planes = 2;

    layer->planes[0].object_index = 0;
    layer->planes[0].offset = 0;
    layer->planes[0].pitch = mpp_frame_get_hor_stride(mppframe);

    layer->planes[1].object_index = 0;
    layer->planes[1].offset = layer->planes[0].pitch * mpp_frame_get_ver_stride(mppframe);
    layer->planes[1].pitch = layer->planes[0].pitch;

    // we also allocate a struct in buf[0] that will allow to hold additionnal information
    // for releasing properly MPP frames and decoder
    framecontextref = av_buffer_allocz(sizeof(*framecontext));
    if (!framecontextref) {
        ret = AVERROR(ENOMEM);
        goto fail;
    }

    // MPP decoder needs to be closed only when all frames have been released.
    framecontext = (RKMPPFrameContext *)framecontextref->data;
    framecontext->decoder_ref = av_buffer_ref(rk_context->decoder_ref);
    framecontext->frame = mppframe;

    frame->data[0]  = (uint8_t *)desc;
    frame->buf[0]   = av_buffer_create((uint8_t *)desc, sizeof(*desc), rkmpp_release_frame,
                                       framecontextref, AV_BUFFER_FLAG_READONLY);

    if (!frame->buf[0]) {
        ret = AVERROR(ENOMEM);
        goto fail;
    }

    frame->hw_frames_ctx = av_buffer_ref(decoder->frames_ref);
    if (!frame->hw_frames_ctx) {
        ret = AVERROR(ENOMEM);
        goto fail;
    }

    return 0;

    out:
    fail:
    if (mppframe)
        mpp_frame_deinit(&mppframe);

    if (framecontext)
        av_buffer_unref(&framecontext->decoder_ref);

    if (framecontextref)
        av_buffer_unref(&framecontextref);

    if (desc)
        av_free(desc);

    return ret;
}

static int axcl_send_packet(AVCodecContext *avctx, AVPacket *packet, AX_S32 timeout)
{
    AXCLDecodeContext *axcl_context = avctx->priv_data;
    AX_VDEC_GRP grp = axcl_context->grp;
    AXCLDecoder *decoder = axcl_context->decoder;

    AX_VDEC_STREAM_T stream;
    memset(&stream, 0, sizeof(stream));
    stream.u64PTS = packet->pts;
    stream.u32StreamPackLen = packet->size;
    stream.pu8Addr = packet->data;
    stream.bEndOfFrame = AX_TRUE;
    if (0 == packet->size) {
        stream.bEndOfStream = AX_TRUE;
    }

    axclError ret;

    // avoid sending new data after EOS
    if (decoder->draining)
        return AVERROR_EOF;

    ret = sample_vdec_send_stream(grp, &stream, timeout);
    uint64_t seq_num = 0;
    if (0 != ret) {
        av_log(avctx, AV_LOG_ERROR, "[decoder %2d] send stream (id: %lu, size: %u) fail, ret = 0x%x\n", grp, seq_num, packet->size, ret);
        return AVERROR_UNKNOWN;
    }

    av_log(avctx, AV_LOG_DEBUG, "Wrote %d bytes to decoder\n", packet->size);
    return 0;
}

static int rkmpp_send_eos(AVCodecContext *avctx)
{
    RKMPPDecodeContext *rk_context = avctx->priv_data;
    RKMPPDecoder *decoder = (RKMPPDecoder *)rk_context->decoder_ref->data;
    MppPacket mpkt;
    int ret;

    ret = mpp_packet_init(&mpkt, NULL, 0);
    if (ret != MPP_OK) {
        av_log(avctx, AV_LOG_ERROR, "Failed to init EOS packet (code = %d)\n", ret);
        return AVERROR_UNKNOWN;
    }

    mpp_packet_set_eos(mpkt);

    do {
        ret = decoder->mpi->decode_put_packet(decoder->ctx, mpkt);
    } while (ret != MPP_OK);
    mpp_packet_deinit(&mpkt);

    decoder->draining = 1;

    return 0;
}

static int axcl_receive_frame(AVCodecContext *avctx, AVFrame *frame)
{
    AXCLDecodeContext *axcl_context = avctx->priv_data;
    AX_VDEC_GRP grp = axcl_context->grp;
    SAMPLE_VDEC_CHN_INFO chn_info = axcl_context->chn_info;
    int32_t device_id = axcl_context->device_id;
    axclrtContext context = axcl_context->context;
    AXCLDecoder *decoder = axcl_context->decoder;
    AVPacket *packet = &decoder->packet;
    axclError ret;

    /* step01: set thread context */
    ret = axclrtSetCurrentContext(context);
    if (AXCL_SUCC != ret) {
        return ret;
    }

    AX_VDEC_STREAM_T stream;
    memset(&stream, 0, sizeof(stream));

    // no more frames after EOS
    if (decoder->eos)
        return AVERROR_EOF;

    // draining remain frames
    if (decoder->draining)
        return axcl_get_frame(avctx, frame);

    while (1) {
        if (!packet->size) {
            ret = ff_decode_get_packet(avctx, packet);
            if (ret == AVERROR_EOF) {
                av_log(avctx, AV_LOG_DEBUG, "End of stream.\n");
                // send EOS and start draining
                stream.bEndOfFrame = AX_TRUE;
                decoder->draining = 1;
                return axcl_get_frame(avctx, frame);
            } else if (ret == AVERROR(EAGAIN)) {
                // not blocking so that we can feed new data ASAP
                return axcl_get_frame(avctx, frame);
            } else if (ret < 0) {
                av_log(avctx, AV_LOG_ERROR, "Failed to get packet (code = %d)\n", ret);
                return ret;
            }
        } else {
            // send pending data to decoder
            ret = axcl_send_packet(avctx, packet, AXCL_SEND_STREAM_TIMEOUT);
            if (ret < 0) {
                av_log(avctx, AV_LOG_ERROR, "Failed to send data (code = %d)\n", ret);
                return ret;
            } else {
                av_packet_unref(packet);
                packet->size = 0;

                // blocked waiting for decode result
                if (decoder->sync)
                    return axcl_get_frame(avctx, frame);
            }
        }
    }
}

static void axcl_flush(AVCodecContext *avctx)
{
    RKMPPDecodeContext *rk_context = avctx->priv_data;
    RKMPPDecoder *decoder = (RKMPPDecoder *)rk_context->decoder_ref->data;

    av_log(avctx, AV_LOG_DEBUG, "Flush.\n");

    decoder->mpi->reset(decoder->ctx);

    rkmpp_prepare_decoder(avctx);

    decoder->eos = 0;
    decoder->draining = 0;
    decoder->last_fps_time = decoder->frames = 0;

    av_packet_unref(&decoder->packet);
}

static const AVCodecHWConfigInternal *const axcl_hw_configs[] = {
        HW_CONFIG_INTERNAL(YUV420P),
        NULL
};

#define AXCL_DEC_CLASS(NAME) \
    static const AVClass axcl_##NAME##_dec_class = { \
        .class_name = "axcl_" #NAME "_dec", \
        .version    = LIBAVUTIL_VERSION_INT, \
    };

#define AXCL_DEC(NAME, ID, BSFS) \
    AXCL_DEC_CLASS(NAME) \
    const FFCodec ff_##NAME##_axcl_decoder = { \
        .p.name         = #NAME "_axcl", \
        .p.long_name    = NULL_IF_CONFIG_SMALL(#NAME " (axcl)"), \
        .p.type         = AVMEDIA_TYPE_VIDEO, \
        .p.id           = ID, \
        .priv_data_size = sizeof(AXCLDecodeContext), \
        .init           = axcl_init_decoder, \
        .close          = axcl_close_decoder, \
        FF_CODEC_RECEIVE_FRAME_CB(axcl_receive_frame), \
        .flush          = axcl_flush, \
        .p.priv_class   = &axcl_##NAME##_dec_class, \
        .p.capabilities = AV_CODEC_CAP_DELAY | AV_CODEC_CAP_AVOID_PROBING | AV_CODEC_CAP_HARDWARE, \
        .p.pix_fmts     = (const enum AVPixelFormat[]) { AV_PIX_FMT_YUV420P, \
                                                         AV_PIX_FMT_NONE}, \
        .hw_configs     = axcl_hw_configs, \
        .bsfs           = BSFS, \
        .p.wrapper_name = "axcl", \
        .caps_internal  = FF_CODEC_CAP_NOT_INIT_THREADSAFE, \
    };

AXCL_DEC(h264,  AV_CODEC_ID_H264,          "h264_mp4toannexb")
AXCL_DEC(hevc,  AV_CODEC_ID_HEVC,          "hevc_mp4toannexb")
