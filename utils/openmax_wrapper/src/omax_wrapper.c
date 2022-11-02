/*
 * Copyright (c) 2016-2022 The Khronos Group Inc.
 * Copyright 2022, QNX Software Systems.
 * Copyright (C) 2022 Texas Instruments Incorporated - http://www.ti.com/
 * 
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject
 * to the following conditions: 
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
 *
 */

#include "omax_wrapper_priv.h"

#define ALIGN64(X)  (((X)+63) &~63)
#define ALIGN16(X)  (((X)+15) &~15)

app_omax_wrapper_obj_t g_app_omax_wrapper_obj = { 0 };
pthread_cond_t g_cond = PTHREAD_COND_INITIALIZER;

static const char *QNX_ENC_COMP_NAME = "OMX.qnx.video.encoder";

static const char* OmxErrorTypeToStr(OMX_ERRORTYPE  err)
{
  const char *str = "Unknown error";
  switch(err)
  {
    case OMX_ErrorNone:                               str = "OMX_ErrorNone";                               break;
    case OMX_ErrorInsufficientResources:              str = "OMX_ErrorInsufficientResources";              break;
    case OMX_ErrorUndefined:                          str = "OMX_ErrorUndefined";                          break;
    case OMX_ErrorInvalidComponentName:               str = "OMX_ErrorInvalidComponentName";               break;
    case OMX_ErrorComponentNotFound:                  str = "OMX_ErrorComponentNotFound";                  break;
    case OMX_ErrorInvalidComponent:                   str = "OMX_ErrorInvalidComponent";                   break;
    case OMX_ErrorBadParameter:                       str = "OMX_ErrorBadParameter";                       break;
    case OMX_ErrorNotImplemented:                     str = "OMX_ErrorNotImplemented";                     break;
    case OMX_ErrorUnderflow:                          str = "OMX_ErrorUnderflow";                          break;
    case OMX_ErrorOverflow:                           str = "OMX_ErrorOverflow";                           break;
    case OMX_ErrorHardware:                           str = "OMX_ErrorHardware";                           break;
    case OMX_ErrorInvalidState:                       str = "OMX_ErrorInvalidState";                       break;
    case OMX_ErrorStreamCorrupt:                      str = "OMX_ErrorStreamCorrupt";                      break;
    case OMX_ErrorPortsNotCompatible:                 str = "OMX_ErrorPortsNotCompatible";                 break;
    case OMX_ErrorResourcesLost:                      str = "OMX_ErrorResourcesLost";                      break;
    case OMX_ErrorNoMore:                             str = "OMX_ErrorNoMore";                             break;
    case OMX_ErrorVersionMismatch:                    str = "OMX_ErrorVersionMismatch";                    break;
    case OMX_ErrorNotReady:                           str = "OMX_ErrorNotReady";                           break;
    case OMX_ErrorTimeout:                            str = "OMX_ErrorTimeout";                            break;
    case OMX_ErrorSameState:                          str = "OMX_ErrorSameState";                          break;
    case OMX_ErrorResourcesPreempted:                 str = "OMX_ErrorResourcesPreempted";                 break;
    case OMX_ErrorPortUnresponsiveDuringAllocation:   str = "OMX_ErrorPortUnresponsiveDuringAllocation";   break;
    case OMX_ErrorPortUnresponsiveDuringDeallocation: str = "OMX_ErrorPortUnresponsiveDuringDeallocation"; break;
    case OMX_ErrorPortUnresponsiveDuringStop:         str = "OMX_ErrorPortUnresponsiveDuringStop";         break;
    case OMX_ErrorIncorrectStateTransition:           str = "OMX_ErrorIncorrectStateTransition";           break;
    case OMX_ErrorIncorrectStateOperation:            str = "OMX_ErrorIncorrectStateOperation";            break;
    case OMX_ErrorUnsupportedSetting:                 str = "OMX_ErrorUnsupportedSetting";                 break;
    case OMX_ErrorUnsupportedIndex:                   str = "OMX_ErrorUnsupportedIndex";                   break;
    case OMX_ErrorBadPortIndex:                       str = "OMX_ErrorBadPortIndex";                       break;
    case OMX_ErrorPortUnpopulated:                    str = "OMX_ErrorPortUnpopulated";                    break;
    case OMX_ErrorComponentSuspended:                 str = "OMX_ErrorComponentSuspended";                 break;
    case OMX_ErrorDynamicResourcesUnavailable:        str = "OMX_ErrorDynamicResourcesUnavailable";        break;
    case OMX_ErrorMbErrorsInFrame:                    str = "OMX_ErrorMbErrorsInFrame";                    break;
    case OMX_ErrorFormatNotDetected:                  str = "OMX_ErrorFormatNotDetected";                  break;
    case OMX_ErrorContentPipeOpenFailed:              str = "OMX_ErrorContentPipeOpenFailed";              break;
    case OMX_ErrorContentPipeCreationFailed:          str = "OMX_ErrorContentPipeCreationFailed";          break;
    case OMX_ErrorSeperateTablesUsed:                 str = "OMX_ErrorSeperateTablesUsed";                 break;
    case OMX_ErrorTunnelingUnsupported:               str = "OMX_ErrorTunnelingUnsupported";               break;
    default: WRAPPER_PRINTF("\nOmxilEnc:%s Unlnown OMX_ERRORTYPE=0x%x", __func__, err);      break;
  }
  return str;
}

static void timedwait(OmxilVideoEncDec_t *encH, const char *caller)
{
    // !!! the mutex is already being held by the caller
    struct timespec to;
    clock_gettime(CLOCK_MONOTONIC, &to);
    nsec2timespec(&to, timespec2nsec(&to) + (uint64_t)TIMEOUT_WAIT);
    int r = pthread_cond_timedwait(&encH->cond, &encH->mutex, &to);
    switch (r)
    {
        case EOK:
            break;
        case ETIMEDOUT:
            WRAPPER_ERROR("\nOmxilEnc=> %s timed-out", caller);
            encH->compError = OMX_ErrorTimeout;
            break;
        default:
            WRAPPER_ERROR("\nOmxilEnc=> %s undefined error: %d", caller, r);
            encH->compError = OMX_ErrorUndefined;
    }
}

static OMX_ERRORTYPE waitForCommandComplete(OmxilVideoEncDec_t *encH)
{
    pthread_mutex_lock(&encH->mutex);
    while(!encH->cmdComplete && encH->compError == OMX_ErrorNone)
    {
        timedwait(encH, __func__);
    }
    OMX_ERRORTYPE err = encH->compError;
    pthread_mutex_unlock(&encH->mutex);
    return err;
}

static OMX_BUFFERHEADERTYPE* OMAX_qPeek(OmxilVideoEncDec_t *encH, omxil_bool isInputQ)
{
    if (isInputQ == omxil_true_e)
    {
        return encH->inputBufHdrList[encH->qInputBufHdr[encH->qInputBufHdrFirstIdx]];
    }
    else
    {
        return encH->outputBufHdrList[encH->qOutputBufHdr[encH->qOutputBufHdrFirstIdx]];
    }
}

static void OMAX_qPush(OmxilVideoEncDec_t *encH, omxil_bool isInputQ, int32_t pBufHdrIdx)
{
    if (isInputQ == omxil_true_e)
    {
        encH->qInputBufHdrLastIdx++;
        encH->qInputBufHdr[encH->qInputBufHdrLastIdx] = pBufHdrIdx;
        if (encH->qInputBufHdrFirstIdx == -1)
        {
            encH->qInputBufHdrFirstIdx = encH->qInputBufHdrLastIdx;
        }
    }
    else
    {
        encH->qOutputBufHdrLastIdx++;
        encH->qOutputBufHdr[encH->qOutputBufHdrLastIdx] = pBufHdrIdx;
        if (encH->qOutputBufHdrFirstIdx == -1)
        {
            encH->qOutputBufHdrFirstIdx = encH->qOutputBufHdrLastIdx;
        }
    }
}

static OMX_BUFFERHEADERTYPE* OMAX_qPop(OmxilVideoEncDec_t *encH, omxil_bool isInputQ)
{
    OMX_BUFFERHEADERTYPE* pBufHdr = NULL;

    if (isInputQ == omxil_true_e)
    {
        if (encH->qInputBufHdrFirstIdx > -1)
        {
            pBufHdr = encH->inputBufHdrList[encH->qInputBufHdr[encH->qInputBufHdrFirstIdx]];
            encH->qInputBufHdr[encH->qInputBufHdrFirstIdx] = -1;
            encH->qInputBufHdrFirstIdx++;

            if (encH->qInputBufHdrFirstIdx > encH->qInputBufHdrLastIdx)
            {
                encH->qInputBufHdrLastIdx = -1;
                encH->qInputBufHdrFirstIdx = -1;
            }
        }
    }
    else
    {
        if (encH->qOutputBufHdrFirstIdx > -1)
        {
            pBufHdr = encH->inputBufHdrList[encH->qOutputBufHdr[encH->qOutputBufHdrFirstIdx]];
            encH->qOutputBufHdr[encH->qOutputBufHdrFirstIdx] = -1;
            encH->qOutputBufHdrFirstIdx++;

            if (encH->qOutputBufHdrFirstIdx > encH->qOutputBufHdrLastIdx)
            {
                encH->qOutputBufHdrLastIdx = -1;
                encH->qOutputBufHdrFirstIdx = -1;
            }
        }
    }

    return pBufHdr;
}

static OMX_ERRORTYPE EventHandler(
    OMX_HANDLETYPE hComponent,
    OMX_PTR pAppData,
    OMX_EVENTTYPE eEvent,
    OMX_U32 nData1,
    OMX_U32 nData2,
    OMX_PTR pEventData)
{
    OmxilVideoEncDec_t *encH = (OmxilVideoEncDec_t *) pAppData;
    OMX_ERRORTYPE omxErr = OMX_ErrorNone;

    if(encH == NULL)
    {
        WRAPPER_ERROR("\nOmxilEnc=> EventHandler Fatal error encH is NULL");
        return OMX_ErrorUndefined;
    }

    if(encH->compHandle == NULL)
    {
        WRAPPER_ERROR("\nOmxilEnc=> EventHandler: compHandle is NULL");
        return OMX_ErrorUndefined;
    }

    switch(eEvent)
    {
        case OMX_EventError:
        {
            if(hComponent == encH->compHandle)
            {
                if (OMX_ErrorStreamCorrupt == (OMX_ERRORTYPE) nData1)
                {
                    WRAPPER_PRINTF("\nOmxilEnc:%s corrupted stream detected; continuing...", __func__);
                }
                else
                {
                    pthread_mutex_lock(&encH->mutex);
                    encH->compError = (OMX_ERRORTYPE) nData1;
                    WRAPPER_ERROR("\nOmxilEnc:%s err=0x%x:'%s' encH=%p", __func__, encH->compError, OmxErrorTypeToStr(encH->compError), encH);
                    pthread_cond_broadcast(&encH->cond);
                    pthread_mutex_unlock(&encH->mutex);
                }
            }
            break;
        }

        case OMX_EventCmdComplete:
        {
            switch((OMX_COMMANDTYPE) nData1)
            {
                case OMX_CommandStateSet:
                {
                    // In this case, nData2 is the arrived at state
                    if(hComponent == encH->compHandle)
                    {
                        WRAPPER_PRINTF("\nOmxilEnc=> Reached compState: %d, encH=%p", (OMX_STATETYPE) nData2, encH);
                        pthread_mutex_lock(&encH->mutex);
                        encH->cmdComplete = omxil_true_e;
                        pthread_cond_signal(&encH->cond);
                        pthread_mutex_unlock(&encH->mutex);
                    }
                    break;
                }

                case OMX_CommandFlush:
                    if(hComponent == encH->compHandle)
                    {
                        pthread_mutex_lock(&encH->mutex);
                        if(nData2 == encH->inPortIndex)
                        {
                            encH->inPortFlushed = omxil_true_e;
                        }
                        if(nData2 == encH->outPortIndex)
                        {
                            encH->outPortFlushed = omxil_true_e;
                        }
                        if(encH->outPortFlushed && encH->inPortFlushed)
                        {
                            encH->cmdComplete = omxil_true_e;
                            pthread_cond_signal(&encH->cond);
                        }
                        pthread_mutex_unlock(&encH->mutex);
                    }
                    break;

                case OMX_CommandPortDisable:
                case OMX_CommandPortEnable:
                    if(hComponent == encH->compHandle)
                    {
                        pthread_mutex_lock(&encH->mutex);
                        encH->cmdComplete = omxil_true_e;
                        pthread_cond_signal(&encH->cond);
                        pthread_mutex_unlock(&encH->mutex);
                    }
                    break;
                default:
                    // do nothing
                    break;
            }
            break;
        }

        case OMX_EventBufferFlag:
        {
            if(nData2 & OMX_BUFFERFLAG_EOS)
            {
                WRAPPER_PRINTF("\nOmxilEnc=> Component detected EOS, encH=%p", encH);
                pthread_mutex_lock(&encH->mutex);
                encH->eos_received = omxil_true_e;
                struct timespec to;
                clock_gettime(CLOCK_MONOTONIC, &to);
                encH->enc_eos_time_ms = (timespec2nsec(&to) / 1000000LL);
                pthread_cond_signal(&encH->cond);
                pthread_mutex_unlock(&encH->mutex);
            }
            break;
        }

        case OMX_EventPortSettingsChanged:
        {
            if(encH && hComponent == encH->compHandle)
            {
                OMX_PARAM_PORTDEFINITIONTYPE portParam;
                OMX_ERRORTYPE omxErr = OMX_ErrorNone;

                WRAPPER_PRINTF("\nOmxilEnc=> port settings changed %u %u, encH=%p, hComponent=%p", nData1, nData2, encH, hComponent);
                pthread_mutex_lock(&encH->mutex);
                SET_OMX_VERSION_SIZE(portParam, sizeof(portParam));
                portParam.nPortIndex = encH->outPortIndex;
                omxErr = OMX_GetParameter(encH->compHandle,
                        OMX_IndexParamPortDefinition,
                        &portParam);
                if(omxErr != OMX_ErrorNone)
                {
                    WRAPPER_ERROR("\nOmxilEnc=> %s:%d OutPort OMX_GetParameter() returned 0x%08x:'%s'", __func__, __LINE__, omxErr, OmxErrorTypeToStr(omxErr));
                }

                WRAPPER_PRINTF("\nOmxilEnc=> encH=%p, nbuffers=%u, ofsize=%u, oheight=%u, owidth=%u, nBufferCountMin=%u",
                            encH,
                            portParam.nBufferCountActual,
                            portParam.nBufferSize,
                            portParam.format.video.nFrameHeight,
                            portParam.format.video.nFrameWidth,
                            portParam.nBufferCountMin);

                pthread_cond_signal(&encH->cond);
                pthread_mutex_unlock(&encH->mutex);
            }
            break;
        }

        default:
            WRAPPER_PRINTF("\nOmxilEnc=> Event %d %u %u encH=%p", eEvent, nData1, nData2, encH);
            break;
    }

    return omxErr;
}

static OMX_ERRORTYPE EmptyBufferDone(
    OMX_HANDLETYPE hComponent,
    OMX_PTR pAppData,
    OMX_BUFFERHEADERTYPE *pBufHdr)
{
    OmxilVideoEncDec_t *encH = (OmxilVideoEncDec_t*) pAppData;
    OMX_ERRORTYPE err = OMX_ErrorNone;
    uint32_t i;

    if(encH == NULL) {
        WRAPPER_ERROR("\nOmxilEnc=> EmptyBufferDone Wrong Venc %p", encH);
        return OMX_ErrorUndefined;
    }

    if(encH->compHandle != hComponent)
    {
        WRAPPER_ERROR("\nOmxilEnc=> EmptyBufferDone Unknown Component %p", hComponent);
        return OMX_ErrorNone;
    }

    pthread_mutex_lock(&encH->mutex);
    WRAPPER_PRINTF("\nOmxilEnc=> %s:%d comp %p, port %u, bufHdr 0x%p, bufPtr 0x%p, size %u", __func__, __LINE__,
                encH->compHandle, encH->inPortIndex, pBufHdr, pBufHdr->pBuffer, pBufHdr->nAllocLen);
    for (i = 0; i < encH->nInputBufs; i++)
    {
        if (pBufHdr == encH->inputBufHdrList[i])
        {
            encH->inBufEmpty[i] = omxil_true_e;
            WRAPPER_PRINTF("\ninBufEmpty[%d] is empty", i);
            break;
        }
    }
    WRAPPER_PRINTF("\ninBufEmpty[%d] is %d", i, encH->inBufEmpty[i]);
    pthread_cond_signal(&encH->cond);
    pthread_mutex_unlock(&encH->mutex);

    return err;
}

static OMX_ERRORTYPE FillBufferDone(
    OMX_HANDLETYPE hComponent,
    OMX_PTR pAppData,
    OMX_BUFFERHEADERTYPE *pBufHdr)
{
    OmxilVideoEncDec_t *encH = (OmxilVideoEncDec_t*) pAppData;
    OMX_ERRORTYPE err = OMX_ErrorNone;

    if(encH == NULL)
    {
        WRAPPER_ERROR("\nOmxilEnc=> FillBufferDone Wrong Venc %p", encH);
        return OMX_ErrorUndefined;
    }

    if(encH->compHandle != hComponent)
    {
        WRAPPER_ERROR("\nOmxilEnc=> FillBufferDone Unknown Component %p", hComponent);
        return OMX_ErrorNone;
    }

    pthread_mutex_lock(&encH->mutex);
    if(pBufHdr->nFilledLen > 0 && encH->out_fd != -1)
    {
        WRAPPER_PRINTF("\nOmxilEnc=> FillBufferDone (nFilledLen=%d, %2x %2x %2x %2x %2x) comp %p, port %u, bufHdr 0x%p, bufPtr 0x%p, size %u", pBufHdr->nFilledLen, pBufHdr->pBuffer[0], pBufHdr->pBuffer[1], pBufHdr->pBuffer[2], pBufHdr->pBuffer[3], pBufHdr->pBuffer[4], encH->compHandle, encH->outPortIndex, pBufHdr, pBufHdr->pBuffer, pBufHdr->nAllocLen);
        if((write(encH->out_fd, pBufHdr->pBuffer, pBufHdr->nFilledLen)) != pBufHdr->nFilledLen)
        {
            WRAPPER_ERROR("\nOmxilEnc=> FillBufferDone error when writing to file");
        }
    }
    else
    {
        WRAPPER_PRINTF("\nOmxilEnc=> FillBufferDone unprocessed data(nFilledLen=%d)", pBufHdr->nFilledLen);
    }

    if(!encH->eos_received && pBufHdr->nFlags & OMX_BUFFERFLAG_EOS)
    {
        encH->eos_received = omxil_true_e;
        struct timespec to;
        clock_gettime(CLOCK_MONOTONIC, &to);
        encH->enc_eos_time_ms = (timespec2nsec(&to) / 1000000LL);
    }

    pBufHdr->nFilledLen = 0;
    for (uint32_t i = 0; i < encH->nOutputBufs; i++)
    {
        if (pBufHdr == encH->outputBufHdrList[i])
        {
            OMAX_qPush(encH, omxil_false_e, i);
            WRAPPER_PRINTF("\nENQUEUE Output Buffer with Idx=%d", i);
            break;
        }
    }
       
    pthread_cond_signal(&encH->cond);
    pthread_mutex_unlock(&encH->mutex);

    return err;
}

static OMX_ERRORTYPE AllocatePortBuffers(OmxilVideoEncDec_t *encH)
{
    OMX_U32 i;
    OMX_ERRORTYPE omxErr = OMX_ErrorNone;
    OMX_U8 *pBuffer;

    //Allocate input port buffers
    WRAPPER_PRINTF("\nOmxilEnc=> AllocatePortBuffers allocating %u input buffers of %u size",
              encH->nInputBufs, encH->inputPortBufSize);

    pthread_mutex_lock(&encH->mutex);
    for(i = 0; i < encH->nInputBufs; i++)
    {
        // Buffers are allocated by MMF and shared with component
        OmxilEncInputBuffer_t *pBuf = encH->input_bufs[i];
        OMX_BUFFERHEADERTYPE *pBufHdr = NULL;
        pBuffer = (OMX_U8 *) (intptr_t) pBuf->addr;

        omxErr = OMX_UseBuffer(encH->compHandle,
                &pBufHdr,
                encH->inPortIndex,
                (OMX_PTR) pBuf->addr,
                pBuf->size,
                pBuffer);

        if( omxErr != OMX_ErrorNone )
        {
            WRAPPER_ERROR("\nOmxilEnc=> ERROR: %s:%d OMX_UseBuffer() returned 0x%08x", __func__, __LINE__, omxErr);
            pthread_mutex_unlock(&encH->mutex);
            return omxErr;
        }
        else
        {
            WRAPPER_PRINTF("\nOmxilEnc=> %s:%d Port Count:%d comp %p, port %u, bufHdr 0x%p, bufPtr 0x%p, size %u", __func__, __LINE__,
                    i, encH->compHandle, encH->inPortIndex, pBufHdr, pBufHdr->pBuffer, pBufHdr->nAllocLen);
            pBufHdr->pAppPrivate = (OMX_PTR)pBuf->addr;
            encH->inputBufHdrList[i] = pBufHdr;
        }
    }
    
    //Allocate output port buffers
    WRAPPER_PRINTF("\nOmxilEnc=> AllocatePortBuffers allocating %u output buffers of %u size",
                encH->nOutputBufs, encH->outputPortBufSize);

    for(i = 0; i < encH->nOutputBufs; i++)
    {
        // Buffers are allocated by MMF and shared with component
        OMX_BUFFERHEADERTYPE *pBufHdr = NULL;
        omxErr = OMX_AllocateBuffer(encH->compHandle,
                &pBufHdr,
                encH->outPortIndex,
                NULL,
                encH->outputPortBufSize);
        if(omxErr != OMX_ErrorNone)
        {
            WRAPPER_ERROR("\nOmxilEnc=> ERROR: %s:%d Port OMX_AllocateBuffer() returned 0x%08x:'%s'", __func__, __LINE__, omxErr, OmxErrorTypeToStr(omxErr));
            pthread_mutex_unlock(&encH->mutex);
            return omxErr;
        }
        else
        {
            WRAPPER_PRINTF("\nOmxilEnc=> %s:%d Port Count:%d Port comp %p, port %u, bufHdr 0x%p, bufPtr 0x%p, size %u", __func__, __LINE__,
                    i, encH->compHandle, encH->outPortIndex, pBufHdr, pBufHdr->pBuffer, encH->outputPortBufSize);
            encH->outputBufHdrList[i] = pBufHdr;
            OMAX_qPush(encH, omxil_false_e, i);
        }
    }
    pthread_mutex_unlock(&encH->mutex);
    return OMX_ErrorNone;
}

static OMX_ERRORTYPE FreeOutPortBuffers(OmxilVideoEncDec_t *encH)
{
    OMX_ERRORTYPE omxErr = OMX_ErrorNone;
    OMX_BUFFERHEADERTYPE *pBufHdr = NULL;
    OMX_U32 nBufFreed = 0;

    pthread_mutex_lock(&encH->mutex);  
    while(nBufFreed < encH->nOutputBufs)
    {
        pBufHdr = OMAX_qPeek(encH, omxil_false_e);
        if(pBufHdr == NULL)
        {
            WRAPPER_PRINTF("\nOmxilEnc=> FreeOutPortBuffers waiting for FillBufferDone... (%u, %u)", nBufFreed, encH->nOutputBufs);
            pthread_cond_wait(&encH->cond, &encH->mutex);
            continue;
        }
        omxErr = OMX_FreeBuffer(encH->compHandle, encH->outPortIndex, pBufHdr);
        if(omxErr != OMX_ErrorNone)
        {
            WRAPPER_ERROR("\nOmxilEnc=> %s:%d OutPort OMX_FreeBuffer() returned 0x%08x:'%s'", __func__, __LINE__, omxErr, OmxErrorTypeToStr(omxErr));
        }
        nBufFreed++;
        OMAX_qPop(encH, omxil_false_e);
    }
    pthread_mutex_unlock(&encH->mutex);
    return omxErr;
}

static OMX_ERRORTYPE FreeInPortBuffers(OmxilVideoEncDec_t *encH)
{
    OMX_U32 i;
    OMX_ERRORTYPE omxErr = OMX_ErrorNone;
    
    pthread_mutex_lock(&encH->mutex);
    for(i = 0; i < encH->nInputBufs; i++)
    {
        while(encH->inBufEmpty[i] == omxil_false_e)
        {
            pthread_cond_wait(&encH->cond, &encH->mutex);
        }

        omxErr = OMX_FreeBuffer(encH->compHandle, encH->inPortIndex, encH->inputBufHdrList[i]);
        if(omxErr != OMX_ErrorNone)
        {
            WRAPPER_ERROR("\nOmxilEnc=> %s:%d InPort OMX_FreeBuffer() returned 0x%08x:'%s'", __func__, __LINE__, omxErr, OmxErrorTypeToStr(omxErr));
        }
    }
    pthread_mutex_unlock(&encH->mutex);

    return omxErr;
}

static OMX_ERRORTYPE MoveToState(OMX_STATETYPE newState, OmxilVideoEncDec_t *encH)
{
    OMX_ERRORTYPE omxErr;
    OMX_STATETYPE currState;

    switch(newState)
    {
        case OMX_StateLoaded:
        case OMX_StateIdle:
        case OMX_StatePause:
        case OMX_StateExecuting:
            break;
        default:
            return OMX_ErrorBadParameter;
    }

    // Make sure the next state is legitimate
    omxErr = OMX_GetState(encH->compHandle, &currState);
    if (omxErr != OMX_ErrorNone)
    {
        WRAPPER_ERROR("\nOmxilEnc=> ERROR: Failed to get current state!!!");
        return omxErr;
    }

    if(currState == OMX_StateInvalid)
    {
        WRAPPER_ERROR("\nOmxilEnc=> ERROR: Transition from Invalid state is not allowed!!!");
        return OMX_ErrorInvalidState;
    }

    if(currState == newState)
    {
        // State for component(s) has already been set
        return OMX_ErrorNone;
    }

    WRAPPER_PRINTF("\nOmxilEnc=> StateTransition target state %d", newState);
    encH->cmdComplete = omxil_false_e;
    omxErr = OMX_SendCommand(encH->compHandle, OMX_CommandStateSet, newState, NULL);
    if(omxErr != OMX_ErrorNone)
    {
        WRAPPER_ERROR("\nOmxilEnc=> ERROR: StateTransition(IDLE) returned 0x%08x", omxErr);
        return omxErr;
    }

    if(currState == OMX_StateLoaded && newState == OMX_StateIdle)
    {
        // Allocate buffers for active ports
        omxErr = AllocatePortBuffers(encH);
        if(omxErr != OMX_ErrorNone)
        {
            WRAPPER_ERROR("\nOmxilEnc=> ERROR: AllocatePortBuffers returned 0x%08x", omxErr);
            return omxErr;
        }
    }
    else if(currState == OMX_StateIdle && newState == OMX_StateLoaded)
    {
        // Free buffers for active ports
        omxErr = FreeInPortBuffers(encH);
        if(omxErr != OMX_ErrorNone)
        {
            WRAPPER_ERROR("\nOmxilEnc=> ERROR: FreeInPortBuffers returned 0x%08x", omxErr);
            return omxErr;
        }

        omxErr = FreeOutPortBuffers(encH);
        if(omxErr != OMX_ErrorNone)
        {
            WRAPPER_ERROR("\nOmxilEnc=> ERROR: FreeOutPortBuffers returned 0x%08x", omxErr);
            return omxErr;
        }
    }

    if((omxErr = waitForCommandComplete(encH)) != OMX_ErrorNone)
    {
        WRAPPER_ERROR("\nOmxilEnc=> ERROR: waitForCommandComplete returned 0x%08x", omxErr);
    }
    return omxErr;
}

static OMX_ERRORTYPE InitEncComp(app_omax_wrapper_obj_t *encH)
{
    OMX_ERRORTYPE omxErr;
    OMX_PORT_PARAM_TYPE portParam;
    OMX_PARAM_PORTDEFINITIONTYPE inPortParam;
    OMX_PARAM_PORTDEFINITIONTYPE outPortParam;

    // Create the OMX component
    OMX_STRING comp_name = (OMX_STRING)QNX_ENC_COMP_NAME;
    // Obtain an instance of video decoder component
    static OMX_CALLBACKTYPE callbacks = { &EventHandler,
                                          &EmptyBufferDone,
                                          &FillBufferDone };

    for (uint8_t ch = 0; ch < encH->params.in_num_channels && omxErr == OMX_ErrorNone; ch++)
    {
        omxErr = OMX_GetHandle(&encH->compHandleArray[ch].compHandle,
                            comp_name,
                            (OMX_PTR) &encH->compHandleArray[ch],
                            &callbacks);
        if(!encH->compHandleArray[ch].compHandle || (omxErr != OMX_ErrorNone))
        {
            WRAPPER_ERROR("\nOmxilEnc=> ERROR: Component(%s) OMX_GetHandle() returned 0x%08x", comp_name, omxErr);
            encH->compHandleArray[ch].compError = omxErr;
            return omxErr;
        }

        // Get component ports info and prepare internal port contexts.
        SET_OMX_VERSION_SIZE(portParam, sizeof(OMX_PORT_PARAM_TYPE));
        portParam.nPorts = 0;
        omxErr = OMX_GetParameter(encH->compHandleArray[ch].compHandle,
                    OMX_IndexParamVideoInit,
                    &portParam);
        if(omxErr != OMX_ErrorNone)
        {
            WRAPPER_ERROR("\nOmxilEnc=> ERROR: Component OMX_GetParameter() returned 0x%08x version %u",
                    omxErr, portParam.nVersion.nVersion);
            encH->compHandleArray[ch].compError = omxErr;
            return omxErr;
        }

        encH->compHandleArray[ch].numOfPorts = portParam.nPorts;
        if(encH->compHandleArray[ch].numOfPorts < 2)
        {
            WRAPPER_ERROR("\nOmxilEnc=> ERROR:  Invalid number of ports %d", encH->compHandleArray[ch].numOfPorts);
            encH->compHandleArray[ch].compError = omxErr;
            return omxErr;
        }
        encH->compHandleArray[ch].inPortIndex = portParam.nStartPortNumber;
        encH->compHandleArray[ch].outPortIndex = portParam.nStartPortNumber + 1;

        //set frame rate
        OMX_CONFIG_FRAMERATETYPE frame_rate_t;
        SET_OMX_VERSION_SIZE(frame_rate_t, sizeof(OMX_CONFIG_FRAMERATETYPE));
        frame_rate_t.nPortIndex = encH->compHandleArray[ch].outPortIndex;
        omxErr = OMX_GetConfig(encH->compHandleArray[ch].compHandle, OMX_IndexConfigVideoFramerate, &frame_rate_t);
        if(omxErr != OMX_ErrorNone)
        {
            WRAPPER_ERROR("\nOmxilEnc=> ERROR: %s:%d Port OMX_GetConfig() returned 0x%08x:'%s'", __func__, __LINE__, omxErr, OmxErrorTypeToStr(omxErr));
            encH->compHandleArray[ch].compError = omxErr;
            return omxErr;
        }
        frame_rate_t.xEncodeFramerate = (OMX_U32)(encH->compHandleArray[ch].frame_rate << 16);

        omxErr = OMX_SetConfig(encH->compHandleArray[ch].compHandle, OMX_IndexConfigVideoFramerate, &frame_rate_t);
        if(omxErr != OMX_ErrorNone)
        {
            WRAPPER_ERROR("\nOmxilEnc=> ERROR: %s:%d Port OMX_SetConfig() returned 0x%08x:'%s'", __func__, __LINE__, omxErr, OmxErrorTypeToStr(omxErr));
            encH->compHandleArray[ch].compError = omxErr;
            return omxErr;
        }

        //set bitrate
        OMX_VIDEO_CONFIG_BITRATETYPE bitrate_t;
        SET_OMX_VERSION_SIZE(bitrate_t, sizeof(OMX_VIDEO_CONFIG_BITRATETYPE));
        bitrate_t.nPortIndex = encH->compHandleArray[ch].outPortIndex;
        omxErr = OMX_GetConfig(encH->compHandleArray[ch].compHandle, OMX_IndexConfigVideoBitrate, &bitrate_t);
        if(omxErr != OMX_ErrorNone)
        {
            WRAPPER_ERROR("\nOmxilEnc=> ERROR: %s:%d Port OMX_GetConfig() returned 0x%08x:'%s'", __func__, __LINE__, omxErr, OmxErrorTypeToStr(omxErr));
            encH->compHandleArray[ch].compError = omxErr;
            return omxErr;
        }
        bitrate_t.nEncodeBitrate = encH->compHandleArray[ch].bitrate;

        omxErr = OMX_SetConfig(encH->compHandleArray[ch].compHandle, OMX_IndexConfigVideoBitrate, &bitrate_t);
        if(omxErr != OMX_ErrorNone)
        {
            WRAPPER_ERROR("\nOmxilEnc=> ERROR: %s:%d Port OMX_SetConfig() returned 0x%08x:'%s'", __func__, __LINE__, omxErr, OmxErrorTypeToStr(omxErr));
            encH->compHandleArray[ch].compError = omxErr;
            return omxErr;
        }

        //set intra period
        OMX_VIDEO_CONFIG_AVCINTRAPERIOD intra_period_t;
        SET_OMX_VERSION_SIZE(intra_period_t, sizeof(OMX_VIDEO_CONFIG_AVCINTRAPERIOD));
        intra_period_t.nPortIndex = encH->compHandleArray[ch].outPortIndex;
        omxErr = OMX_GetConfig(encH->compHandleArray[ch].compHandle, OMX_IndexConfigVideoAVCIntraPeriod, &intra_period_t);
        if(omxErr != OMX_ErrorNone)
        {
            WRAPPER_ERROR("\nOmxilEnc=> ERROR: %s:%d Port OMX_GetConfig() returned 0x%08x:'%s'", __func__, __LINE__, omxErr, OmxErrorTypeToStr(omxErr));
            encH->compHandleArray[ch].compError = omxErr;
            return omxErr;
        }
        intra_period_t.nIDRPeriod = encH->compHandleArray[ch].idr_period;

        omxErr = OMX_SetConfig(encH->compHandleArray[ch].compHandle, OMX_IndexConfigVideoAVCIntraPeriod, &intra_period_t);
        if(omxErr != OMX_ErrorNone)
        {
            WRAPPER_ERROR("\nOmxilEnc=> ERROR: %s:%d Port OMX_SetConfig() returned 0x%08x:'%s'", __func__, __LINE__, omxErr, OmxErrorTypeToStr(omxErr));
            encH->compHandleArray[ch].compError = omxErr;
            return omxErr;
        }

        //set bitrate control
        OMX_VIDEO_PARAM_BITRATETYPE bitrate_ctl_t;
        SET_OMX_VERSION_SIZE(bitrate_ctl_t, sizeof(OMX_VIDEO_PARAM_BITRATETYPE));
        bitrate_ctl_t.nPortIndex = encH->compHandleArray[ch].outPortIndex;
        omxErr = OMX_GetParameter(encH->compHandleArray[ch].compHandle,
                                OMX_IndexParamVideoBitrate,
                                &bitrate_ctl_t);
        if(omxErr != OMX_ErrorNone)
        {
            WRAPPER_ERROR("\nOmxilEnc=> ERROR: %s:%d Port OMX_GetParameter() returned 0x%08x:'%s'", __func__, __LINE__, omxErr, OmxErrorTypeToStr(omxErr));
            encH->compHandleArray[ch].compError = omxErr;
            return omxErr;
        }
        bitrate_ctl_t.eControlRate = (OMX_VIDEO_CONTROLRATETYPE)encH->compHandleArray[ch].rcmode;
        bitrate_ctl_t.nTargetBitrate = 10000000;

        omxErr = OMX_SetParameter(encH->compHandleArray[ch].compHandle,
                                OMX_IndexParamVideoBitrate,
                                &bitrate_ctl_t);
        if(omxErr != OMX_ErrorNone)
        {
            WRAPPER_ERROR("\nOmxilEnc=> ERROR: %s:%d Port OMX_SetParameter() returned 0x%08x:'%s'", __func__, __LINE__, omxErr, OmxErrorTypeToStr(omxErr));
            encH->compHandleArray[ch].compError = omxErr;
            return omxErr;
        }

        //Configure input port
        pthread_mutex_lock(&encH->compHandleArray[ch].mutex);
        SET_OMX_VERSION_SIZE(inPortParam, sizeof(OMX_PARAM_PORTDEFINITIONTYPE));
        inPortParam.nPortIndex = encH->compHandleArray[ch].inPortIndex;
        omxErr = OMX_GetParameter(encH->compHandleArray[ch].compHandle,
                                OMX_IndexParamPortDefinition,
                                &inPortParam);
        if(omxErr != OMX_ErrorNone)
        {
            WRAPPER_ERROR("\nOmxilEnc=> ERROR: %s:%d Port OMX_GetParameter() returned 0x%08x:'%s'", __func__, __LINE__, omxErr, OmxErrorTypeToStr(omxErr));
            encH->compHandleArray[ch].compError = omxErr;
            return omxErr;
        }

        // Allocate input buffer for the worst case to avoid input buffer reconfiguration
        inPortParam.format.video.nFrameWidth   = encH->compHandleArray[ch].src_width;
        inPortParam.format.video.nFrameHeight  = encH->compHandleArray[ch].src_height;
        inPortParam.format.video.nStride = encH->compHandleArray[ch].src_stride;
        inPortParam.format.video.xFramerate    = (OMX_U32)(encH->compHandleArray[ch].frame_rate << 16); // FrameRate in Q16 format
        inPortParam.format.video.eColorFormat  = (OMX_COLOR_FORMATTYPE)OMXQ_COLOR_FormatNV12;
        inPortParam.nBufferCountActual = encH->compHandleArray[ch].input_buf_num;

        WRAPPER_PRINTF("\nOmxilEnc=> Port comp %p, port %u, nFrameWidth: %d, nFrameHeight: %d, nStride: %d, xFramerate: %d, eColorFormat: %d", encH->compHandleArray[ch].compHandle, inPortParam.nPortIndex, inPortParam.format.video.nFrameWidth, inPortParam.format.video.nFrameHeight, inPortParam.format.video.nStride, inPortParam.format.video.xFramerate, inPortParam.format.video.eColorFormat);
        omxErr = OMX_SetParameter(encH->compHandleArray[ch].compHandle,
                                OMX_IndexParamPortDefinition,
                                &inPortParam);
        if(omxErr != OMX_ErrorNone)
        {
            WRAPPER_ERROR("\nOmxilEnc=> ERROR: %s:%d Port OMX_SetParameter() returned 0x%08x:'%s'", __func__, __LINE__, omxErr, OmxErrorTypeToStr(omxErr));
            encH->compHandleArray[ch].compError = omxErr;
            return omxErr;
        }

        omxErr = OMX_GetParameter(encH->compHandleArray[ch].compHandle,
                                OMX_IndexParamPortDefinition,
                                &inPortParam);
        if(omxErr != OMX_ErrorNone)
        {
            WRAPPER_ERROR("\nOmxilEnc=> ERROR: %s:%d Port OMX_GetParameter() returned 0x%08x:'%s'", __func__, __LINE__, omxErr, OmxErrorTypeToStr(omxErr));
            encH->compHandleArray[ch].compError = omxErr;
            return omxErr;
        }
        WRAPPER_PRINTF("\nOmxilEnc=> Port comp %p, port %u, nFrameWidth: %d, nFrameHeight: %d, nStride: %d, xFramerate: %d, eColorFormat: %d", encH->compHandleArray[ch].compHandle, inPortParam.nPortIndex, inPortParam.format.video.nFrameWidth, inPortParam.format.video.nFrameHeight, inPortParam.format.video.nStride, inPortParam.format.video.xFramerate, inPortParam.format.video.eColorFormat);
        encH->compHandleArray[ch].nInputBufs = inPortParam.nBufferCountActual;
        encH->compHandleArray[ch].inputPortBufSize = inPortParam.nBufferSize;

        //Configure output port
        SET_OMX_VERSION_SIZE(outPortParam, sizeof(OMX_PARAM_PORTDEFINITIONTYPE));
        outPortParam.nPortIndex = encH->compHandleArray[ch].outPortIndex;
        omxErr = OMX_GetParameter(encH->compHandleArray[ch].compHandle,
                                OMX_IndexParamPortDefinition,
                                &outPortParam);
        if(omxErr != OMX_ErrorNone)
        {
            WRAPPER_ERROR("\nOmxilEnc=> ERROR: %s:%d Port OMX_GetParameter() returned 0x%08x:'%s'", __func__, __LINE__, omxErr, OmxErrorTypeToStr(omxErr));
            encH->compHandleArray[ch].compError = omxErr;
            return omxErr;
        }

        outPortParam.format.video.eCompressionFormat = OMX_VIDEO_CodingAVC;
        outPortParam.format.video.nFrameWidth   = encH->compHandleArray[ch].src_width;
        outPortParam.format.video.nFrameHeight  = encH->compHandleArray[ch].src_height;
        outPortParam.nBufferCountActual = encH->compHandleArray[ch].input_buf_num;
        outPortParam.nBufferSize = encH->compHandleArray[ch].frame_size;
        WRAPPER_PRINTF("\nOmxilEnc=> Port comp %p, port %u, eCompressionFormat: %d, nFrameWidth: %d, nFrameHeight: %d", encH->compHandleArray[ch].compHandle, outPortParam.nPortIndex, outPortParam.format.video.eCompressionFormat, outPortParam.format.video.nFrameWidth, outPortParam.format.video.nFrameHeight);
        omxErr = OMX_SetParameter(encH->compHandleArray[ch].compHandle,
                                OMX_IndexParamPortDefinition,
                                &outPortParam);
        if(omxErr != OMX_ErrorNone)
        {
            WRAPPER_ERROR("\nOmxilEnc=> ERROR: %s:%d Port OMX_SetParameter() returned 0x%08x:'%s'", __func__, __LINE__, omxErr, OmxErrorTypeToStr(omxErr));
            encH->compHandleArray[ch].compError = omxErr;
            return omxErr;
        }

        omxErr = OMX_GetParameter(encH->compHandleArray[ch].compHandle,
                                OMX_IndexParamPortDefinition,
                                &outPortParam);
        if(omxErr != OMX_ErrorNone)
        {
            WRAPPER_ERROR("\nOmxilEnc=> ERROR: %s:%d Port OMX_SetParameter() returned 0x%08x:'%s'", __func__, __LINE__, omxErr, OmxErrorTypeToStr(omxErr));
            encH->compHandleArray[ch].compError = omxErr;
            return omxErr;
        }
        WRAPPER_PRINTF("\nOmxilEnc=> Port comp %p, port %u, eCompressionFormat: %d, nFrameWidth: %d, nFrameHeight: %d", encH->compHandleArray[ch].compHandle, outPortParam.nPortIndex, outPortParam.format.video.eCompressionFormat, outPortParam.format.video.nFrameWidth, outPortParam.format.video.nFrameHeight);
        encH->compHandleArray[ch].outputPortBufSize = outPortParam.nBufferSize;
        encH->compHandleArray[ch].nOutputBufs = outPortParam.nBufferCountActual;

        //set avc type
        OMX_VIDEO_PARAM_AVCTYPE avc;
        SET_OMX_VERSION_SIZE(avc, sizeof(OMX_VIDEO_PARAM_AVCTYPE));
        avc.nPortIndex = encH->compHandleArray[ch].outPortIndex;
        omxErr = OMX_GetParameter(encH->compHandleArray[ch].compHandle,
                                OMX_IndexParamVideoAvc,
                                &avc);
        if(omxErr != OMX_ErrorNone)
        {
            WRAPPER_ERROR("\nOmxilEnc=> ERROR: %s:%d Port OMX_GetParameter() returned 0x%08x:'%s'", __func__, __LINE__, omxErr, OmxErrorTypeToStr(omxErr));
            encH->compHandleArray[ch].compError = omxErr;
            return omxErr;
        }
        avc.eProfile = OMX_VIDEO_AVCProfileHigh;
        avc.eLevel = OMX_VIDEO_AVCLevel1;

        omxErr = OMX_SetParameter(encH->compHandleArray[ch].compHandle,
                                OMX_IndexParamVideoAvc,
                                &avc);
        if(omxErr != OMX_ErrorNone)
        {
            WRAPPER_ERROR("\nOmxilEnc=> ERROR: %s:%d Port OMX_SetParameter() returned 0x%08x:'%s'", __func__, __LINE__, omxErr, OmxErrorTypeToStr(omxErr));
            encH->compHandleArray[ch].compError = omxErr;
            return omxErr;
        }

        pthread_mutex_unlock(&encH->compHandleArray[ch].mutex);

        encH->compHandleArray[ch].compError = OMX_ErrorNone;
        omxErr = OMX_ErrorNone;
    }
    
    WRAPPER_PRINTF("\nOmxilEnc=>%s video created successfully", __func__);
    return omxErr;
}

static OMX_ERRORTYPE StartVenc(OmxilVideoEncDec_t *encH)
{
    OMX_ERRORTYPE omxErr;

    omxErr = MoveToState(OMX_StateIdle, encH);
    if(omxErr != OMX_ErrorNone)
    {
        WRAPPER_ERROR("\nOmxilEnc=> ERROR: Transition LOADED->IDLE failed 0x%x", omxErr);
        return omxErr;
    }

    omxErr = MoveToState(OMX_StateExecuting, encH);
    if(omxErr != OMX_ErrorNone)
    {
        WRAPPER_ERROR("\nOmxilEnc=> ERROR: Transition IDLE->EXECUTING failed 0x%x", omxErr);
        return omxErr;
    }

    WRAPPER_PRINTF("\nOmxilEnc=> Transition to EXECUTING done");
    return OMX_ErrorNone;
}

static OMX_ERRORTYPE StopVenc(OmxilVideoEncDec_t *encH)
{
    OMX_ERRORTYPE omxErr = OMX_ErrorNone;

    omxErr = MoveToState(OMX_StateIdle, encH);
    if(omxErr != OMX_ErrorNone)
    {
        WRAPPER_ERROR("\nOmxilEnc=> ERROR: StopVenc Transition EXECUTING->IDLE failed 0x%x", omxErr);
        return omxErr;
    }

    omxErr = MoveToState(OMX_StateLoaded, encH);
    if(omxErr != OMX_ErrorNone)
    {
        WRAPPER_ERROR("\nOmxilEnc=> ERROR: StopVenc Transition IDLE->LOADED failed 0x%x", omxErr);
        return omxErr;
    }

    return omxErr;
}

static void CloseVenc(OmxilVideoEncDec_t *encH)
{
    OMX_ERRORTYPE omxErr;
    OMX_STATETYPE currState;

    WRAPPER_PRINTF("\nOmxilEnc=> CloseVenc called encH= %p",(void *) encH);

    if(encH == NULL)
    {
        WRAPPER_ERROR("\nOmxilEnc=> CloseVenc encH is NULL !!!");
        return;
    }

    //wait util state change completed
    omxErr = OMX_GetState(encH->compHandle, &currState);
    if (omxErr == OMX_ErrorNone)
    {
        if(currState != OMX_StateInvalid && currState != OMX_StateLoaded)
        {
            StopVenc(encH);
        }
    }

    // Freeing component
    WRAPPER_PRINTF("\nOmxilEnc=> Freeing component %p", encH->compHandle);
    omxErr = OMX_FreeHandle(encH->compHandle);
    if(omxErr != OMX_ErrorNone)
    {
        WRAPPER_ERROR("\nOmxilEnc=> Component OMX_FreeHandle() returned 0x%08x", omxErr);
    }

    return;
}

static OMX_ERRORTYPE FlushVenc(OmxilVideoEncDec_t *encH)
{
    OMX_ERRORTYPE omxErr;
    OMX_STATETYPE currState;
    OMX_ERRORTYPE err = OMX_ErrorNone;

    WRAPPER_PRINTF("\nOmxilEnc=> FlushVenc In(%p)",encH);
    omxErr = OMX_GetState(encH->compHandle, &currState);
    if (omxErr != OMX_ErrorNone)
    {
        WRAPPER_ERROR("\nOmxilEnc=> ERROR: FlushVenc Failed to get current state!!!");
        return omxErr;
    }

    if(currState == OMX_StateInvalid)
    {
        WRAPPER_ERROR("\nOmxilEnc=> ERROR: flushing from INVALID state is not allowed!");
        return omxErr;
    }

    if(currState == OMX_StateLoaded || currState == OMX_StateIdle)
    {
        WRAPPER_PRINTF("\nOmxilEnc=> skip flushing due to compState=%d", currState);
        return OMX_ErrorNone;
    }

    encH->cmdComplete = omxil_false_e;
    encH->inPortFlushed = omxil_false_e;
    encH->outPortFlushed = omxil_false_e;
    omxErr = OMX_SendCommand(encH->compHandle, OMX_CommandFlush, OMX_ALL, NULL);
    if(omxErr != OMX_ErrorNone)
    {
        WRAPPER_ERROR("\nOmxilEnc=> ERROR: Send OMX_CommandFlush returned 0x%08x", omxErr);
        return omxErr;
    }
    if((omxErr = waitForCommandComplete(encH)) != OMX_ErrorNone)
    {
        WRAPPER_ERROR("\nOmxilEnc=> ERROR: waitForCommandComplete returned 0x%08x", omxErr);
        return omxErr;
    }
    WRAPPER_PRINTF("\nOmxilEnc=> FlushVenc complete");

    return err;
}

static void *encoder_file_push_thread(void *args)
{
    pthread_setname_np(pthread_self(), "encoder_file_push_thread");
    WRAPPER_PRINTF("\nencoder_file_push_thread(%d) has started \n", gettid());

    OmxilVideoEncDec_t *encH = (OmxilVideoEncDec_t *)args;
    OMX_ERRORTYPE omxErr;

    WRAPPER_PRINTF("\nOmxilEnc=>%s start push input frames", __func__);
    struct timespec to;
    clock_gettime(CLOCK_MONOTONIC, &to);
    encH->enc_start_time_ms = (timespec2nsec(&to) / 1000000LL);
    for (;;)
    {
        pthread_mutex_lock(&encH->mutex);
        if (encH->compError != OMX_ErrorNone || encH->eos_received)
        {
            WRAPPER_PRINTF("\nencoder_player_push_thread bailed or eos(%d) at %d\n",encH->eos_received, __LINE__);
            pthread_cond_signal(&g_cond);
            pthread_mutex_unlock(&encH->mutex);
            break;
        }

        while(!(encH->qOutputBufHdrFirstIdx == -1))
        {
            OMX_BUFFERHEADERTYPE *oBufHdr = OMAX_qPeek(encH, omxil_false_e);

            WRAPPER_PRINTF("\nOmxilEnc=> OMX_FillThisBuffer: %s:%d comp %p, port %u, bufHdr 0x%p, bufPtr 0x%p, size %u", __func__, __LINE__,
                encH->compHandle, encH->outPortIndex, oBufHdr, oBufHdr->pBuffer, oBufHdr->nAllocLen);
            
            memset(oBufHdr->pBuffer, 0, oBufHdr->nAllocLen);
            omxErr = OMX_FillThisBuffer(encH->compHandle, oBufHdr);
            if(omxErr != OMX_ErrorNone)
            {
                WRAPPER_ERROR("\nOmxilEnc=> ERROR: OMX_FillThisBuffer: return omxErr=%08x", omxErr);
                break;
            }
            OMAX_qPop(encH, omxil_false_e);
        }
        pthread_mutex_unlock(&encH->mutex);
    }
    WRAPPER_PRINTF("\nencoder_file_push_thread has stopped");
    return NULL;
}

int32_t appOMXInit(app_codec_wrapper_params_t* params)
{
    int32_t status = 0;
    app_omax_wrapper_obj_t* p_omax_pipe_obj = &g_app_omax_wrapper_obj;

    p_omax_pipe_obj->params = *params;
    
    if (strcmp(params->in_format, "NV12") || strcmp(params->out_format, "NV12"))
    {
        WRAPPER_ERROR("\nomax_wrapper: Error: Only NV12 format supported!");
        status = -1;
    }

    if (status == 0)
    {
        if (params->appEncode == 1)
        {
            status = procmgr_ability(0,
            PROCMGR_AOP_ALLOW | PROCMGR_ADN_NONROOT | PROCMGR_AID_KEYDATA,
            PROCMGR_AOP_ALLOW | PROCMGR_ADN_NONROOT | PROCMGR_AID_IO,
            PROCMGR_AOP_ALLOW | PROCMGR_ADN_NONROOT | PROCMGR_AID_MEM_PHYS,
            PROCMGR_AOP_ALLOW | PROCMGR_ADN_NONROOT | PROCMGR_AID_PRIORITY,
            PROCMGR_AOP_DENY  | PROCMGR_ADN_NONROOT | PROCMGR_AOP_LOCK      | PROCMGR_AID_EOL);
        
            if (status != 0)
            {
                WRAPPER_ERROR("\nUnable to gain procmgr abilities for nonroot operation.");
                return status;
            }

            ThreadCtl(_NTO_TCTL_IO, 0);

            for (uint8_t ch = 0; ch < p_omax_pipe_obj->params.in_num_channels && status==0; ch++)
            {
                p_omax_pipe_obj->compHandleArray[ch].bitrate = 1024000;
                p_omax_pipe_obj->compHandleArray[ch].idr_period = 30;
                p_omax_pipe_obj->compHandleArray[ch].rcmode = (int32_t)OMX_Video_ControlRateConstant;
            }
        }
    }

    //thread lock for input/output port
    for (uint8_t ch = 0; ch < p_omax_pipe_obj->params.in_num_channels && status==0; ch++)
    {
        if(pthread_mutex_init(&p_omax_pipe_obj->compHandleArray[ch].mutex, NULL) != EOK)
        {
            WRAPPER_ERROR("\nOmxilEnc=>%s failure, mutex init err ", __func__);
        }
    }

    pthread_condattr_t attr;
    if (pthread_condattr_init(&attr) != EOK)
    {
        WRAPPER_ERROR("\nOmxilEnc=>%s failure, condattr init err", __func__);
        status = -1;
    }
    if (pthread_condattr_setclock(&attr, CLOCK_MONOTONIC) != EOK)
    {
        WRAPPER_ERROR("\nOmxilEnc=>%s failure, condattr_setclock err ", __func__);
        pthread_condattr_destroy(&attr);
        status = -1;
    }

    for (uint8_t ch = 0; ch < p_omax_pipe_obj->params.in_num_channels && status==0; ch++)
    {
        if(pthread_cond_init(&p_omax_pipe_obj->compHandleArray[ch].cond, &attr) != EOK)
        {
            WRAPPER_ERROR("\nOmxilEnc=>%s failure, cond init err ", __func__);
            pthread_condattr_destroy(&attr);
            status = -1;
        }
    }
    pthread_condattr_destroy(&attr);

    p_omax_pipe_obj->push_count = -1;
    p_omax_pipe_obj->pull_count = -1;
    for (uint8_t ch = 0; ch < p_omax_pipe_obj->params.in_num_channels && status==0; ch++)
    {
        p_omax_pipe_obj->compHandleArray[ch].in_fd = -1;
        p_omax_pipe_obj->compHandleArray[ch].out_fd = -1;
        p_omax_pipe_obj->compHandleArray[ch].qInputBufHdrFirstIdx = -1;
        p_omax_pipe_obj->compHandleArray[ch].qInputBufHdrLastIdx = -1;
        p_omax_pipe_obj->compHandleArray[ch].qOutputBufHdrFirstIdx = -1;
        p_omax_pipe_obj->compHandleArray[ch].qOutputBufHdrLastIdx = -1;
    }

    return status;
}

int32_t appOMXEncInit(void* data_ptr[CODEC_MAX_BUFFER_DEPTH][CODEC_MAX_NUM_CHANNELS][CODEC_MAX_NUM_PLANES])
{
    int32_t status = 0;
    OMX_ERRORTYPE omxErr;
    app_omax_wrapper_obj_t* p_omax_pipe_obj = &g_app_omax_wrapper_obj;

    p_omax_pipe_obj->push_count = 0;
    
    for (uint8_t ch = 0; ch < p_omax_pipe_obj->params.in_num_channels && status==0; ch++)
    {
        p_omax_pipe_obj->compHandleArray[ch].input_buf_num = p_omax_pipe_obj->params.in_buffer_depth;
        p_omax_pipe_obj->compHandleArray[ch].channelIdx = ch;
        p_omax_pipe_obj->compHandleArray[ch].src_height = p_omax_pipe_obj->params.in_height;
        p_omax_pipe_obj->compHandleArray[ch].src_width = p_omax_pipe_obj->params.in_width;
        p_omax_pipe_obj->compHandleArray[ch].aligned_height = ALIGN16(p_omax_pipe_obj->compHandleArray[ch].src_height);
        p_omax_pipe_obj->compHandleArray[ch].src_stride = ALIGN64(p_omax_pipe_obj->compHandleArray[ch].src_width);
        p_omax_pipe_obj->compHandleArray[ch].frame_size = p_omax_pipe_obj->compHandleArray[ch].src_width * p_omax_pipe_obj->compHandleArray[ch].src_height * 3 / 2; //default yuv420 format.
        p_omax_pipe_obj->compHandleArray[ch].frame_rate = 30;
        snprintf(p_omax_pipe_obj->compHandleArray[ch].out_path, OMAX_MAX_FILE_PATH, "output_video_%d.264", ch);
        
        if(p_omax_pipe_obj->params.appEncode == 1 && p_omax_pipe_obj->params.appDecode == 0)
        {
            p_omax_pipe_obj->compHandleArray[ch].out_fd = open(p_omax_pipe_obj->compHandleArray[ch].out_path, O_WRONLY | O_CREAT);
            if(p_omax_pipe_obj->compHandleArray[ch].out_fd == -1)
            {
                WRAPPER_ERROR("\nError: unable to open output file (%s)", p_omax_pipe_obj->compHandleArray[ch].out_path);
                return OMX_ErrorInsufficientResources;
            }
        }

        WRAPPER_PRINTF("\nInput stride is %d", p_omax_pipe_obj->compHandleArray[ch].src_stride);

        for (uint8_t idx = 0; idx < p_omax_pipe_obj->params.in_buffer_depth && status==0; idx++)
        {
            void *pointer = data_ptr[idx][ch][0];
            off64_t offset = 0;

            p_omax_pipe_obj->compHandleArray[ch].input_bufs[idx] = (OmxilEncInputBuffer_t *)malloc(sizeof(OmxilEncInputBuffer_t));
            
            /* JB: HACK: use special area of memory to keep buffers in low mem region */
            status = mem_offset64(pointer, NOFD, 1, &offset, NULL);
            if (offset == 0)
            {
                WRAPPER_ERROR("\n%s:%d: Error: failed mem_offset. errno=%d", __FUNCTION__, __LINE__, errno);
                return OMX_ErrorBadParameter;
            }

            p_omax_pipe_obj->compHandleArray[ch].input_bufs[idx]->addr = pointer;
            p_omax_pipe_obj->compHandleArray[ch].input_bufs[idx]->size = p_omax_pipe_obj->compHandleArray[ch].frame_size;
            p_omax_pipe_obj->compHandleArray[ch].input_bufs[idx]->offset = offset;
            WRAPPER_PRINTF("\nGet frame size for input_bufs[%d][%d]: %d, %p, %lx", idx, ch, p_omax_pipe_obj->compHandleArray[ch].frame_size, pointer, offset);
            p_omax_pipe_obj->compHandleArray[ch].input_bufs[idx]->offset = offset;
        }
    }

    // Initialize OpenMAX IL
    omxErr = OMX_Init();
    if(omxErr != OMX_ErrorNone)
    {
        WRAPPER_ERROR("\nOmxilEnc=> ERROR: Component OMX_Init() returned 0x%08x:'%s'", omxErr, OmxErrorTypeToStr(omxErr));
        return omxErr;
    }
    
    if((InitEncComp(p_omax_pipe_obj)) != OMX_ErrorNone)
    {
        WRAPPER_ERROR("\nFailed to create encoder:");
        return OMX_ErrorInsufficientResources;
    }

    if (status != OMX_ErrorNone)
    {
        WRAPPER_ERROR("\nInitialization of OpenMAX IL failed!");
    }

    return status;
}

int32_t appOMXDecInit(void* (*data_ptr)[CODEC_MAX_NUM_CHANNELS][CODEC_MAX_NUM_PLANES])
{
    int32_t status = 0;
    app_omax_wrapper_obj_t* p_omax_pipe_obj = &g_app_omax_wrapper_obj;

    if (status == 0)
    {
        p_omax_pipe_obj->pull_count = 0;
    }

    if (status != OMX_ErrorNone)
    {
        WRAPPER_ERROR("\nInitialization of OpenMAX IL failed!");
    }

    return status;
}

int32_t appOMXStart()
{
    int32_t status = 0;
    app_omax_wrapper_obj_t* p_omax_pipe_obj = &g_app_omax_wrapper_obj;
    
    for (uint8_t ch = 0; ch < p_omax_pipe_obj->params.in_num_channels; ch++)
    {
        status = StartVenc(&p_omax_pipe_obj->compHandleArray[ch]);

        /* We're not going to register a buffer callback for our encoder,
        * having a seperate thread to push buffers is faster.
        * We'll pull buffers from the decoder output queue and push it into the
        * encoder input queue.
        */
        pthread_attr_t attr;
        pthread_attr_init(&attr);
        pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
        pthread_create(&p_omax_pipe_obj->compHandleArray[ch].encoder_push, &attr, encoder_file_push_thread, (void *)&p_omax_pipe_obj->compHandleArray[ch]);
        pthread_attr_destroy(&attr);
    }

    return status;
}

int32_t appOMXEnqAppEnc(uint8_t idx)
{
    int32_t status = 0;
    app_omax_wrapper_obj_t* p_omax_pipe_obj = &g_app_omax_wrapper_obj;

    if(p_omax_pipe_obj->params.appEncode == 0)
    {
        WRAPPER_ERROR("\nOMAX Pipeline not initialised correctly: appEncode = 0 !");
        return -1;
    }

    p_omax_pipe_obj->push_count++;

    WRAPPER_PRINTF("\nOMAX Enqueue idx=%d", idx);

    for (uint8_t ch = 0; ch < p_omax_pipe_obj->params.in_num_channels; ch++)
    {
        pthread_mutex_lock(&(p_omax_pipe_obj->compHandleArray[ch].mutex));

        WRAPPER_PRINTF("\nOmxilEnc=> %s:%d comp %p, port %u, bufHdr 0x%p, bufPtr 0x%p, size %u", __func__, __LINE__,
                p_omax_pipe_obj->compHandleArray[ch].compHandle, p_omax_pipe_obj->compHandleArray[ch].inPortIndex, p_omax_pipe_obj->compHandleArray[ch].inputBufHdrList[idx], p_omax_pipe_obj->compHandleArray[ch].inputBufHdrList[idx]->pBuffer, p_omax_pipe_obj->compHandleArray[ch].inputBufHdrList[idx]->nAllocLen);

        OMAX_qPush(&(p_omax_pipe_obj->compHandleArray[ch]), omxil_true_e, idx);
        p_omax_pipe_obj->compHandleArray[ch].inBufEmpty[idx] = omxil_false_e;

        OMX_BUFFERHEADERTYPE *buffer = NULL;
        if(!p_omax_pipe_obj->compHandleArray[ch].eos_sent)
        {    
            buffer = OMAX_qPop(&(p_omax_pipe_obj->compHandleArray[ch]), omxil_true_e);
            buffer->nFilledLen = buffer->nAllocLen;
            p_omax_pipe_obj->compHandleArray[ch].input_frame_cnt++;
        }
        if (buffer != NULL)
        {
            WRAPPER_PRINTF("\nOmxilEnc=> OMX_EmptyThisBuffer: %s:%d comp %p, port %u, bufHdr 0x%p, bufPtr 0x%p, size %u", __func__, __LINE__,
                p_omax_pipe_obj->compHandleArray[ch].compHandle, p_omax_pipe_obj->compHandleArray[ch].inPortIndex, buffer, buffer->pBuffer, buffer->nAllocLen);
            status = OMX_EmptyThisBuffer(p_omax_pipe_obj->compHandleArray[ch].compHandle, buffer);
            if(status != OMX_ErrorNone) 
            {
                WRAPPER_ERROR("\nOmxilEnc=> ERROR: OMX_EmptyThisBuffer: return omxErr=%08x", status);
                pthread_mutex_unlock(&(p_omax_pipe_obj->compHandleArray[ch].mutex));
                break;
            }
        }
        pthread_mutex_unlock(&(p_omax_pipe_obj->compHandleArray[ch].mutex));
    }

    return status;
}

int32_t appOMXDeqAppEnc(uint8_t idx)
{
    app_omax_wrapper_obj_t* p_omax_pipe_obj = &g_app_omax_wrapper_obj;
    OMX_ERRORTYPE omxErr = OMX_ErrorNone;
    omxil_bool inBufEmptyArray[p_omax_pipe_obj->params.in_num_channels];
    omxil_bool allChannelsEmpty = omxil_false_e;
    uint8_t chanIdx = 0;

    if(p_omax_pipe_obj->params.appEncode == 0)
    {
        WRAPPER_ERROR("\nOMAX Pipeline not initialised correctly: appEncode = 0 !");
        return -1;
    }

    WRAPPER_PRINTF("\nOMAX Dequeue idx=%d", idx);

    for (uint8_t ch = 0; ch < p_omax_pipe_obj->params.in_num_channels; ch++)
    {
        inBufEmptyArray[ch] = p_omax_pipe_obj->compHandleArray[ch].inBufEmpty[idx];
    }
    
    while(allChannelsEmpty == omxil_false_e)
    {
        if(chanIdx == p_omax_pipe_obj->params.in_num_channels)
        {
            chanIdx = 0;
        }

        inBufEmptyArray[chanIdx] = p_omax_pipe_obj->compHandleArray[chanIdx].inBufEmpty[idx];
        chanIdx++;
        for (uint8_t ch = 0; ch < p_omax_pipe_obj->params.in_num_channels; ch++)
        {
            if(inBufEmptyArray[ch] == omxil_false_e)
            {
                allChannelsEmpty = omxil_false_e;
                break;
            }
            else
            {
                allChannelsEmpty = omxil_true_e;
            }
        }
    }

    for (uint8_t ch = 0; ch < p_omax_pipe_obj->params.in_num_channels; ch++)
    {
        OMAX_qPop(&(p_omax_pipe_obj->compHandleArray[ch]), omxil_true_e);
    }

    return omxErr;
}

int32_t appOMXEnqEosAppEnc()
{
    int32_t status = 0;
    app_omax_wrapper_obj_t* p_omax_pipe_obj = &g_app_omax_wrapper_obj;

    if(p_omax_pipe_obj->params.appEncode == 0)
    {
        WRAPPER_ERROR("\nOMAX Pipeline not initialised correctly: appEncode = 0 !");
        return -1;
    }

    for (uint8_t ch = 0; ch < p_omax_pipe_obj->params.in_num_channels; ch++)
    {
        pthread_mutex_lock(&(p_omax_pipe_obj->compHandleArray[ch].mutex));

        WRAPPER_PRINTF("\nOmxilEnc=> %s:%d comp %p, port %u, bufHdr 0x%p, bufPtr 0x%p, size %u", __func__, __LINE__,
                p_omax_pipe_obj->compHandleArray[ch].compHandle, p_omax_pipe_obj->compHandleArray[ch].inPortIndex, p_omax_pipe_obj->compHandleArray[ch].inputBufHdrList[0], p_omax_pipe_obj->compHandleArray[ch].inputBufHdrList[0]->pBuffer, p_omax_pipe_obj->compHandleArray[ch].inputBufHdrList[0]->nAllocLen);

        OMAX_qPush(&(p_omax_pipe_obj->compHandleArray[ch]), omxil_true_e, 0);
        p_omax_pipe_obj->compHandleArray[ch].inBufEmpty[0] = omxil_false_e;

        OMX_BUFFERHEADERTYPE *buffer = NULL;
        if(!p_omax_pipe_obj->compHandleArray[ch].eos_sent)
        {    
            buffer = OMAX_qPop(&(p_omax_pipe_obj->compHandleArray[ch]), omxil_true_e);
            buffer->nFilledLen = buffer->nAllocLen;
            p_omax_pipe_obj->compHandleArray[ch].input_frame_cnt++;
        }
        if (buffer != NULL)
        {
            WRAPPER_PRINTF("\nencoder_file_push_thread send eos");
            buffer->nFlags |= OMX_BUFFERFLAG_EOS;
            p_omax_pipe_obj->compHandleArray[ch].eos_sent = omxil_true_e;

            status = OMX_EmptyThisBuffer(p_omax_pipe_obj->compHandleArray[ch].compHandle, buffer);
            if(status != OMX_ErrorNone) 
            {
                WRAPPER_ERROR("\nOmxilEnc=> ERROR: OMX_EmptyThisBuffer: return omxErr=%08x", status);
                pthread_mutex_unlock(&(p_omax_pipe_obj->compHandleArray[ch].mutex));
                break;
            }
            pthread_cond_wait(&(p_omax_pipe_obj->compHandleArray[ch].cond), &(p_omax_pipe_obj->compHandleArray[ch].mutex));
        }
        pthread_mutex_unlock(&(p_omax_pipe_obj->compHandleArray[ch].mutex));
    }

    return status;
}

int32_t appOMXDeqAppDec(uint8_t idx)
{
    int32_t status = 0;
    app_omax_wrapper_obj_t* p_omax_pipe_obj = &g_app_omax_wrapper_obj;

    if(p_omax_pipe_obj->params.appDecode == 0)
    {
        WRAPPER_ERROR("\nOMAX Pipeline not initialised correctly: appDecode = 0 !");
        return -1;
    }

    return status;
}

int32_t appOMXEnqAppDec(uint8_t idx)
{
    int32_t status = 0;
    app_omax_wrapper_obj_t* p_omax_pipe_obj = &g_app_omax_wrapper_obj;

    if(p_omax_pipe_obj->params.appDecode == 0)
    {
        WRAPPER_ERROR("\nOMAX Pipeline not initialised correctly: appDecode = 0 !");
        return -1;
    }

    return status;
}

int32_t appOMXStop()
{
    int32_t status = 0;
    app_omax_wrapper_obj_t* p_omax_pipe_obj = &g_app_omax_wrapper_obj;

    for (uint8_t ch = 0; ch < p_omax_pipe_obj->params.in_num_channels; ch++)
    {
        int64_t enc_time = p_omax_pipe_obj->compHandleArray[ch].enc_eos_time_ms - p_omax_pipe_obj->compHandleArray[ch].enc_start_time_ms;

        if(enc_time > 0)
        {
            p_omax_pipe_obj->compHandleArray[ch].input_frame_cnt--;
            float enc_frame_rate = p_omax_pipe_obj->compHandleArray[ch].input_frame_cnt  * 1000.0 / (float)enc_time;
            p_omax_pipe_obj->compHandleArray[ch].enc_time_ms = enc_time;
            p_omax_pipe_obj->compHandleArray[ch].enc_frame_rate = enc_frame_rate;
        }
        else
        {
            p_omax_pipe_obj->compHandleArray[ch].enc_time_ms = -1;
        }

        /* Terminate our buffer push thread before continuing */
        if (pthread_join(p_omax_pipe_obj->compHandleArray[ch].encoder_push, NULL) != 0)
        {
            if (errno != ESRCH)
            {
                WRAPPER_ERROR("\nFailed to terminate encoder_player_push_thread! Error: %s", strerror(errno));
            }
        }
        
        if(p_omax_pipe_obj->compHandleArray[ch].compHandle)
        {
            FlushVenc(&p_omax_pipe_obj->compHandleArray[ch]);
            CloseVenc(&p_omax_pipe_obj->compHandleArray[ch]);
        }

        pthread_mutex_destroy(&p_omax_pipe_obj->compHandleArray[ch].mutex);
        pthread_cond_destroy(&p_omax_pipe_obj->compHandleArray[ch].cond);
        
        if(p_omax_pipe_obj->compHandleArray[ch].out_fd != -1)
        {
            close(p_omax_pipe_obj->compHandleArray[ch].out_fd);
            WRAPPER_PRINTF("\nClosed Output File\n");
        }
        if(p_omax_pipe_obj->compHandleArray[ch].in_fd != -1)
        {
            close(p_omax_pipe_obj->compHandleArray[ch].in_fd);
            WRAPPER_PRINTF("\nClosed Input File\n");
        }   
    }

    return status;
}

void appOMXDeInit()
{
    app_omax_wrapper_obj_t* p_omax_pipe_obj = &g_app_omax_wrapper_obj;

    OMX_Deinit();

    for (uint8_t idx = 0; idx < p_omax_pipe_obj->params.in_buffer_depth; idx++)
    {
        for (uint8_t ch = 0; ch < p_omax_pipe_obj->params.in_num_channels; ch++)
        {
            free(p_omax_pipe_obj->compHandleArray[ch].input_bufs[idx]);
        }
    }
    
    WRAPPER_PRINTF("\nOpenMAX IL enc destroyed, exiting.");
}

void appOMXPrintStats()
{
    app_omax_wrapper_obj_t* p_omax_pipe_obj = &g_app_omax_wrapper_obj;

    printf("\nOMAX_WRAPPER PUSH/PULL COUNTS:\n");
    printf("\n\n");
    printf("\nPush count : %d\n", p_omax_pipe_obj->push_count);
    printf("\nPull count : %d\n", p_omax_pipe_obj->pull_count);
    printf("\n\n");
}
