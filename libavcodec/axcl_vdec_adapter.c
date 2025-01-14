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
#include "libavutil/mem.h"
#include "libavutil/refstruct.h"

#include "axcl/ax_base_type.h"
#include "axcl/axcl_base.h"
#include "axcl/axcl.h"
#include "axcl/ax_buffer_tool.h"
#include "axcl_dma_buffer.h"

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
    AX_VDEC_STREAM_T stream;

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
    AX_VIDEO_FRAME_INFO_T axcl_frame;
    int32_t device_id;
    axclrtContext context;
    void* cma_allocator;
} AXCLDecodeContext;

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

static void axcl_release_decoder(AVRefStructOpaque opaque, void *data)
{
    AXCLDecoder *decoder = (AXCLDecoder *)data;

    av_buffer_unref(&decoder->frames_ref);
    av_buffer_unref(&decoder->device_ref);

    av_free(decoder);
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

    AXCLDecodeContext *axcl_context  = avctx->priv_data;
    struct dma_buffer cma_allocator = {
            .m_fd = -1,  // 初始化文件描述符为-1
            .m_mem = {
                    .size = 0,
                    .vir = NULL,
                    .phy = 0,
            }
    };
    dma_buffer_init(&cma_allocator);
    axcl_context->cma_allocator = &cma_allocator;

    AX_VIDEO_FRAME_INFO_T axcl_frame = axcl_context->axcl_frame;
    memset(&axcl_frame, 0, sizeof(axcl_frame));

    // create a decoder and a ref to it
    AXCLDecoder *decoder = NULL;
    decoder = av_refstruct_alloc_ext(sizeof(*decoder), 0,
                                     NULL, axcl_release_decoder);
    if (!decoder) {
        ret = AVERROR(ENOMEM);
        av_refstruct_unref(decoder);
    }
    axcl_context->decoder = decoder;

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

        const size_t size = ALIGN_UP(chn_info.u32PicWidth, VDEC_STRIDE_ALIGN) * chn_info.u32PicHeight * 3 / 2;
        if (size <= 0) {
            axclrtDestroyContext(context);
            av_log(avctx, AV_LOG_ERROR, "[decoder %2d] decode nv12 frame size = %ld\n", grp, size);
            return AV_LOG_ERROR;
        }

        if (!dma_buffer_alloc(&cma_allocator, size)) {
            av_log(avctx, AV_LOG_ERROR, "[decoder %2d] alloc cma mem size %ld fail\n", grp, size);
            axclrtDestroyContext(context);
            return AV_LOG_ERROR;
        }
//
//        decode_threads[i].start(name, sample_get_decoded_image_thread, grp, device_id, &decode_eof_events[i], chn_info, dump);
//
//        /* step08: start to demux video */
//        SAMPLE_LOG_I("start demuxer %d", i);
//        ffmpeg_start_demuxer(demuxers[i]);
    }

    return ret;
}

static int axcl_close_decoder(AVCodecContext *avctx)
{
    AXCLDecodeContext *axcl_context = avctx->priv_data;
    AXCLDecoder *decoder = (AXCLDecoder *)axcl_context->decoder;

    dma_buffer_free(axcl_context->cma_allocator);

    av_packet_unref(&decoder->packet);

    int32_t count = 1;
    /* step09: stop vdec and demuxer */
    for (int32_t i = 0; i < count; ++i) {
        /**
         * bugfix:
         * Stop vdec first; otherwise `sample_vdec_send_stream(-1)` may hang.
        */
        sample_vdec_stop(axcl_context->grp);
    }

    /* step10: deinit vdec module */
    av_log(avctx, AV_LOG_INFO, "deinit vdec\n");
    sample_vdec_deinit();

    /* step11: deinit sys module */
    av_log(avctx, AV_LOG_INFO, "deinit sys\n");
    AXCL_SYS_Deinit();

    /* step12: deinit axcl */
    av_log(avctx, AV_LOG_INFO, "axcl deinit\n");
    axclrtResetDevice(axcl_context->device_id);
    axclFinalize();

    return 0;
}

static int axcl_convert_frame(AVCodecContext *avctx, AVFrame *frame)
{
    AXCLDecodeContext *axcl_context = avctx->priv_data;
    AX_VDEC_GRP grp = axcl_context->grp;
    SAMPLE_VDEC_CHN_INFO chn_info = axcl_context->chn_info;
    AX_VIDEO_FRAME_INFO_T *axcl_frame = &axcl_context->axcl_frame;
    void* cma_allocator = axcl_context->cma_allocator;
    int32_t device_id = axcl_context->device_id;
    AXCLDecoder *decoder = axcl_context->decoder;
    AX_VDEC_STREAM_T *stream = &decoder->stream;
    AVPacket *packet = &decoder->packet;

    axclError ret;

    const size_t size = ALIGN_UP(chn_info.u32PicWidth, VDEC_STRIDE_ALIGN) * chn_info.u32PicHeight * 3 / 2;

    ret = axclrtMemcpy((void *) dma_buffer_get(cma_allocator)->phy,
                       (const void *) axcl_frame->stVFrame.u64PhyAddr[0], size,
                       AXCL_MEMCPY_DEVICE_TO_HOST_PHY);
    if (AXCL_SUCC != ret) {
        av_log(avctx, AV_LOG_ERROR, "copy rawframe from device to cpu, ret = 0x%x\n", ret);
        return ret;
    }

    char *src = dma_buffer_get(cma_allocator)->vir;
    char *dst_y = frame->data[0];
    char *dst_u = frame->data[1];
    char *dst_v = frame->data[2];
    int width = axcl_frame->stVFrame.u32Width;
    int height = axcl_frame->stVFrame.u32Height;
    int hstride = ALIGN_UP(width, VDEC_STRIDE_ALIGN);
    int vstride = height;
    int y_pitch = frame->linesize[0];
    int u_pitch = frame->linesize[1];
    int v_pitch = frame->linesize[2];
    int i, j;

    // 打印src的前10个字节
//    for (int i = 0; i < 10; i++) {
//        av_log(avctx, AV_LOG_WARNING, "src[%d] = %d\n", i, src[i]);
//    }

    // 创建文件/tmp/fuck.yuv，并将src的yuv数据写入到本地文件/tmp/fuck.yuv
//    FILE *fp = fopen("/tmp/fuck.yuv", "wb");
//    fwrite(src, 1, size, fp);

    if (axcl_frame->stVFrame.enImgFormat != AX_FORMAT_YUV420_SEMIPLANAR) {
        av_log(avctx, AV_LOG_WARNING, "Unable to convert\n");
        return -1;
    }

    av_log(avctx, AV_LOG_WARNING, "Doing slow software conversion\n");

    for (i = 0; i < frame->height; i++)
        memcpy(dst_y + i * y_pitch, src + i * hstride, frame->width);

    src += hstride * vstride;

    for (i = 0; i < frame->height / 2; i++) {
        for (j = 0; j < frame->width / 2; j++) {
            dst_u[j] = src[2 * j + 0];
            dst_v[j] = src[2 * j + 1];
        }
        dst_u += u_pitch;
        dst_v += v_pitch;
        src += hstride;
    }

    return 0;
}

static int axcl_get_frame(AVCodecContext *avctx, AVFrame *frame)
{
    AXCLDecodeContext *axcl_context = avctx->priv_data;
    AX_VDEC_GRP grp = axcl_context->grp;
    SAMPLE_VDEC_CHN_INFO chn_info = axcl_context->chn_info;
    AX_VIDEO_FRAME_INFO_T *axcl_frame = &axcl_context->axcl_frame;
    int32_t device_id = axcl_context->device_id;
    AXCLDecoder *decoder = axcl_context->decoder;
    AX_VDEC_STREAM_T *stream = &decoder->stream;
    AVPacket *packet = &decoder->packet;

    axclError ret;

    // should not provide any frame after EOS
    if (decoder->eos)
        return AVERROR_EOF;

    // here we should have a valid frame
    av_log(avctx, AV_LOG_DEBUG, "Received a frame.\n");

    if (stream->bEndOfStream) {
        av_log(avctx, AV_LOG_DEBUG, "Received a EOS frame.\n");
        decoder->eos = 1;
        ret = AVERROR_EOF;
        return ret;
    }

    const AX_VDEC_CHN chn = chn_info.u32ChnId;

    const size_t size = ALIGN_UP(chn_info.u32PicWidth, VDEC_STRIDE_ALIGN) * chn_info.u32PicHeight * 3 / 2;

    /* step02: get decoded image */
    ret = sample_vdec_get_frame(grp, chn, axcl_frame, 100);
    if (0 != ret) {
        if (AX_ERR_VDEC_UNEXIST == ret) {
            av_log(avctx, AV_LOG_ERROR, "[decoder %2d] grp is destroyed\n", grp);
            goto fail;
        } else if (AX_ERR_VDEC_STRM_ERROR == ret) {
            av_log(avctx, AV_LOG_ERROR, "[decoder %2d] stream is undecodeable\n", grp);
            goto fail;
        } else if (AX_ERR_VDEC_FLOW_END == ret) {
            av_log(avctx, AV_LOG_ERROR, "[decoder %2d] flow end\n", grp);
            goto fail;
        } else {
            av_log(avctx, AV_LOG_ERROR, "[decoder %2d] get frame fail, ret = 0x%x\n", grp, ret);
            goto fail;
        }
    }

    // TODO: axcl frame to ffmpeg frame
    avctx->width = axcl_frame->stVFrame.u32Width;
    avctx->height = axcl_frame->stVFrame.u32Height;
    avctx->pix_fmt = AV_PIX_FMT_YUV420P;  // todo: ax 和 ffmpeg对应匹配
    av_log(avctx, AV_LOG_INFO, "Decoder noticed an info change (%dx%d), format=%d\n",
           avctx->width, avctx->height, avctx->pix_fmt);

    // chromium would align planes' width and height to 32, adding this
    // hack to avoid breaking the plane buffers' contiguous.
    avctx->coded_width = FFALIGN(avctx->width, 64);
    avctx->coded_height = FFALIGN(avctx->height, 64);

    ret = ff_get_buffer(avctx, frame, 0);
    if (ret < 0)
        goto out;

    // setup general frame fields
    frame->format           = avctx->pix_fmt;
    frame->width            = axcl_frame->stVFrame.u32Width;
    frame->height           = axcl_frame->stVFrame.u32Height;
    frame->pts              = axcl_frame->stVFrame.u64PTS;

    ret = axcl_convert_frame(avctx, frame);

    /* TODO: */
    if (0 == ret) {
        /* step03: release decoded image */
        ret = sample_vdec_release_frame(grp, chn, axcl_frame);
        if (0 != ret){
            av_log(avctx, AV_LOG_ERROR, "[decoder %2d] release frame fail, ret = 0x%x\n", grp, ret);
            goto fail;
        }
    }

    return 0;

    out:
    fail:
        ret = sample_vdec_release_frame(grp, chn, axcl_frame);
        if (0 != ret){
            av_log(avctx, AV_LOG_ERROR, "[decoder %2d] goto fail: release frame fail, ret = 0x%x\n", grp, ret);
        }
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

static int axcl_receive_frame(AVCodecContext *avctx, AVFrame *frame)
{
    AXCLDecodeContext *axcl_context = avctx->priv_data;
    AX_VDEC_GRP grp = axcl_context->grp;
    SAMPLE_VDEC_CHN_INFO chn_info = axcl_context->chn_info;
    int32_t device_id = axcl_context->device_id;
    axclrtContext context = axcl_context->context;
    AXCLDecoder *decoder = axcl_context->decoder;
    AX_VDEC_STREAM_T *stream = &decoder->stream;
    AVPacket *packet = &decoder->packet;
    axclError ret;

    ret = axclrtSetCurrentContext(axcl_context->context);
    if (AXCL_SUCC != ret) {
        return ret;
    }
    memset(stream, 0, sizeof(&stream));

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
                stream->bEndOfStream = AX_TRUE;
                decoder->draining    = 1;
                return axcl_get_frame(avctx, frame);
//                return ret;  // todo: dont know how to decode the last frame without forever blocking
//            } else if (ret == AVERROR(EAGAIN)) {
//                av_log(avctx, AV_LOG_ERROR, "No packet.\n");
//                return AVERROR(EAGAIN);
            } else if (ret < 0) {
                av_log(avctx, AV_LOG_DEBUG, "Failed to get packet (code = %d)\n", ret);
                return ret;
            } else {
                av_log(avctx, AV_LOG_DEBUG, "Got a packet of size %d\n", packet->size);
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
                return axcl_get_frame(avctx, frame);
            }
        }
    }
}

static void axcl_flush(AVCodecContext *avctx)
{
    AXCLDecodeContext *axcl_context = avctx->priv_data;
    AXCLDecoder *decoder = (AXCLDecoder *)axcl_context->decoder;

    av_log(avctx, AV_LOG_DEBUG, "Flush.\n");

    decoder->eos = 0;
    decoder->draining = 0;
    decoder->last_fps_time = decoder->frames = 0;

    av_packet_unref(&decoder->packet);
}

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
        .p.capabilities = AV_CODEC_CAP_DELAY | AV_CODEC_CAP_AVOID_PROBING, \
        .p.pix_fmts     = (const enum AVPixelFormat[]) { AV_PIX_FMT_YUV420P, \
                                                         AV_PIX_FMT_NONE}, \
        .bsfs           = BSFS, \
        .p.wrapper_name = "axcl", \
        .caps_internal  = FF_CODEC_CAP_NOT_INIT_THREADSAFE, \
    };

AXCL_DEC(h264,  AV_CODEC_ID_H264,          "h264_mp4toannexb")
AXCL_DEC(hevc,  AV_CODEC_ID_HEVC,          "hevc_mp4toannexb")
