/**
 ********************************************************************
 * @file    dji_camera_stream_decoder.cpp
 * @brief
 *
 * @copyright (c) 2021 DJI. All rights reserved.
 *
 * All information contained herein is, and remains, the property of DJI.
 * The intellectual and technical concepts contained herein are proprietary
 * to DJI and may be covered by U.S. and foreign patents, patents in process,
 * and protected by trade secret or copyright law.  Dissemination of this
 * information, including but not limited to data and other proprietary
 * material(s) incorporated within the information, in any form, is strictly
 * prohibited without the express written consent of DJI.
 *
 * If you receive this source code without DJI’s authorization, you may not
 * further disseminate the information, and you must immediately remove the
 * source code and notify DJI of its removal. DJI reserves the right to pursue
 * legal actions against you for any loss(es) or damage(s) caused by your
 * failure to do so.
 *
 *********************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "umd_psdk_wrapper/dji_camera_stream_decoder.hpp"
#include "unistd.h"
#include "pthread.h"
#include "dji_logger.h"
#include <iostream> 

/* Private constants ---------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private values -------------------------------------------------------------*/

/* Private functions declaration ---------------------------------------------*/

/* Exported functions definition ---------------------------------------------*/
DJICameraStreamDecoder::DJICameraStreamDecoder()
    : initSuccess(false),
      cbThreadIsRunning(false),
      cbThreadStatus(-1),
      cb(nullptr),
      cbUserParam(nullptr),
      pCodecCtx(nullptr),
      pCodec(nullptr),
      pCodecParserCtx(nullptr),
      pSwsCtx(nullptr),
      pFrameYUV(nullptr),
      pFrameRGB(nullptr),
      rgbBuf(nullptr),
      bufSize(0)
{
    pthread_mutex_init(&decodemutex, nullptr);
}

DJICameraStreamDecoder::~DJICameraStreamDecoder()
{
    pthread_mutex_destroy(&decodemutex);
    if(cb)
    {
        registerCallback(nullptr, nullptr);
    }

    cleanup();
}

bool DJICameraStreamDecoder::init()
{
    pthread_mutex_lock(&decodemutex);

    if (true == initSuccess) {
        USER_LOG_INFO("Decoder already initialized.\n");
        return true;
    }

    avcodec_register_all();
    pCodecCtx = avcodec_alloc_context3(nullptr);
    if (!pCodecCtx) {
        return false;
    }

    pCodecCtx->thread_count = 4;
    pCodec = avcodec_find_decoder(AV_CODEC_ID_H264);
    if (!pCodec || avcodec_open2(pCodecCtx, pCodec, nullptr) < 0) {
        return false;
    }

    pCodecParserCtx = av_parser_init(AV_CODEC_ID_H264);
    if (!pCodecParserCtx) {
        return false;
    }

    pFrameYUV = av_frame_alloc();
    if (!pFrameYUV) {
        return false;
    }

    pFrameRGB = av_frame_alloc();
    if (!pFrameRGB) {
        return false;
    }

    pSwsCtx = nullptr;

    pCodecCtx->flags2 |= AV_CODEC_FLAG2_SHOW_ALL;
    initSuccess = true;

    pthread_mutex_unlock(&decodemutex);

    return true;
}

void DJICameraStreamDecoder::cleanup()
{
    pthread_mutex_lock(&decodemutex);

    initSuccess = false;
    if (nullptr != pSwsCtx) {
        sws_freeContext(pSwsCtx);
        pSwsCtx = nullptr;
    }

    if (nullptr != pFrameYUV) {
        av_free(pFrameYUV);
        pFrameYUV = nullptr;
    }

    if (nullptr != pCodecParserCtx) {
        av_parser_close(pCodecParserCtx);
        pCodecParserCtx = nullptr;
    }

    if (nullptr != pCodec) {
        avcodec_close(pCodecCtx);
        pCodec = nullptr;
    }

    if (nullptr != pCodecCtx) {
        av_free(pCodecCtx);
        pCodecCtx = nullptr;
    }

    if (nullptr != rgbBuf) {
        av_free(rgbBuf);
        rgbBuf = nullptr;
    }

    if (nullptr != pFrameRGB) {
        av_free(pFrameRGB);
        pFrameRGB = nullptr;
    }

    pthread_mutex_unlock(&decodemutex);
}

void *DJICameraStreamDecoder::callbackThreadEntry(void *p)
{
    std::cout << "16" << std::endl;
    //DSTATUS_PRIVATE("****** Decoder Callback Thread Start ******\n");
    usleep(50 * 1000);
    static_cast<DJICameraStreamDecoder *>(p)->callbackThreadFunc();
    return nullptr;
}

void DJICameraStreamDecoder::callbackThreadFunc()
{
    std::cout << "15" << std::endl;
    while (cbThreadIsRunning) {
        CameraRGBImage copyOfImage;
        if (!decodedImageHandler.getNewImageWithLock(copyOfImage, 1000)) {
            //DDEBUG_PRIVATE("Decoder Callback Thread: Get image time out\n");
            continue;
        }

        if (cb) {
            std::cout << "If cb in callbackThreadFunc" << std::endl;
            (*cb)(copyOfImage, cbUserParam);
        }
    }
}

void DJICameraStreamDecoder::decodeBuffer(const uint8_t *buf, int bufLen)
{
    std::cout << "14" << std::endl;
    const uint8_t *pData = buf;
    int remainingLen = bufLen;
    int processedLen = 0;

    AVPacket pkt;
    av_init_packet(&pkt);
    pthread_mutex_lock(&decodemutex);
    while (remainingLen > 0) {
        if (!pCodecParserCtx || !pCodecCtx) {
            //DSTATUS("Invalid decoder ctx.");
            break;
        }
        processedLen = av_parser_parse2(pCodecParserCtx, pCodecCtx,
                                        &pkt.data, &pkt.size,
                                        pData, remainingLen,
                                        AV_NOPTS_VALUE, AV_NOPTS_VALUE, AV_NOPTS_VALUE);
        remainingLen -= processedLen;
        pData += processedLen;

        if (pkt.size > 0) {
            int gotPicture = 0;
            avcodec_decode_video2(pCodecCtx, pFrameYUV, &gotPicture, &pkt);

            if (!gotPicture) {
                ////DSTATUS_PRIVATE("Got Frame, but no picture\n");
                continue;
            } else {
                int w = pFrameYUV->width;
                int h = pFrameYUV->height;
                ////DSTATUS_PRIVATE("Got picture! size=%dx%d\n", w, h);

                if (nullptr == pSwsCtx) {
                    pSwsCtx = sws_getContext(w, h, pCodecCtx->pix_fmt,
                                             w, h, AV_PIX_FMT_RGB24,
                                             4, nullptr, nullptr, nullptr);
                    std::cout << "Filling pSwsCtx for entering writeNewImageWithLock" << std::endl;
                }

                if (nullptr == rgbBuf) {
                    bufSize = avpicture_get_size(AV_PIX_FMT_RGB24, w, h);
                    rgbBuf = (uint8_t *) av_malloc(bufSize);
                    avpicture_fill((AVPicture *) pFrameRGB, rgbBuf, AV_PIX_FMT_RGB24, w, h);
                    std::cout << "Filling rgbBuf for entering writeNewImageWithLock" << std::endl;
                }

                if (nullptr != pSwsCtx && nullptr != rgbBuf) {
                    std::cout << "pSwsCtx and rgbBuf != nullptr" << std::endl;
                    sws_scale(pSwsCtx,
                              (uint8_t const *const *) pFrameYUV->data, pFrameYUV->linesize, 0, pFrameYUV->height,
                              pFrameRGB->data, pFrameRGB->linesize);

                    pFrameRGB->height = h;
                    pFrameRGB->width = w;
                    std::cout << "Gonna call writeNewImageWithLock" << std::endl;
                    decodedImageHandler.writeNewImageWithLock(pFrameRGB->data[0], bufSize, w, h);
                }
            }
        }
    }
    pthread_mutex_unlock(&decodemutex);
    av_free_packet(&pkt);
}

bool DJICameraStreamDecoder::registerCallback(CameraImageCallback f, void *param)
{
    std::cout << "registerCallback 0" << std::endl;
    cb = f;
    cbUserParam = param;

    /* When users register a non-nullptr callback, we will start the callback thread. */
    if (nullptr != cb) {
        std::cout << "registerCallback 1" << std::endl;
        if (!cbThreadIsRunning) {
            std::cout << "registerCallback 2" << std::endl;
            cbThreadStatus = pthread_create(&callbackThread, nullptr, callbackThreadEntry, this);
            if (0 == cbThreadStatus) {
                std::cout << "registerCallback 3" << std::endl;
                //DSTATUS_PRIVATE("User callback thread created successfully!\n");
                cbThreadIsRunning = true;
                return true;
            } else {
                std::cout << "registerCallback 4" << std::endl;
                //DERROR_PRIVATE("User called thread creation failed!\n");
                cbThreadIsRunning = false;
                return false;
            }
        } else {
            std::cout << "registerCallback 5" << std::endl;
            //DERROR_PRIVATE("Callback thread already running!\n");
            return true;
        }
    } else {
        std::cout << "registerCallback 6" << std::endl;
        if (cbThreadStatus == 0) {
            std::cout << "registerCallback 7" << std::endl;
            cbThreadIsRunning = false;
            pthread_join(callbackThread, nullptr);
            cbThreadStatus = -1;
        }
        return true;
    }
}

/* Private functions definition-----------------------------------------------*/

/****************** (C) COPYRIGHT DJI Innovations *****END OF FILE****/
