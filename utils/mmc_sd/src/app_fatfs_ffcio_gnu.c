/*
 * Copyright (c) 2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*
 * ======== ffcio.c ========
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>

#include <ti/fs/fatfs/ff.h>

void appFatFsLock(void);
void appFatFsUnLock(void);

#define MAX_OPEN_FILES  10


static FIL *filTable[MAX_OPEN_FILES];

static int init = 0;

/*
 *  ======== _open ========
 */
int _open(const char *path, int flags, ...)
{
    int         dev_fd = -1;
    BYTE        fflags;
    int         status = 0;

    appFatFsLock();

    if (!init) {
        for (dev_fd = 0; dev_fd < MAX_OPEN_FILES; dev_fd++) {
            filTable[dev_fd] = NULL;
        }
        /* reserve for stdin, stdout, stderr */
        filTable[0] = (FIL *)~0;
        filTable[1] = (FIL *)~0;
        filTable[2] = (FIL *)~0;

        init = 1;
    }

    for (dev_fd = 0; (dev_fd < MAX_OPEN_FILES) && (filTable[dev_fd] != NULL); dev_fd++) {
    }

    if (dev_fd == MAX_OPEN_FILES) {
        /* no available file handles */
        status = (-1);
    }

    if(status == 0)
    {
        filTable[dev_fd] = malloc(sizeof(FIL));

        if (filTable[dev_fd] == NULL) {
            /* allocation failed */
            status = (-1);
        }
    }
    if(status==0)
    {
        switch (flags & 0x3) {
            case O_RDONLY:
                fflags = FA_READ;
                break;

            case O_WRONLY:
                fflags = FA_WRITE;
                break;

            case O_RDWR:
                fflags = FA_READ | FA_WRITE;
                break;

            default:
                status = (-1);
                break;
        }
    }
    if(status==0)
    {
        if (flags & O_TRUNC) {
            fflags |= FA_CREATE_ALWAYS;
        }

        if (flags & O_CREAT) {
            fflags |= FA_OPEN_ALWAYS;
        }

        if (flags & O_APPEND) {
            ;
        }

        if (f_open(filTable[dev_fd], path, fflags) != FR_OK) {
            status = -1;

        }
    }
    if(status==-1)
    {
        if(dev_fd < MAX_OPEN_FILES)
        {
            if(filTable[dev_fd])
                free(filTable[dev_fd]);

            filTable[dev_fd] =NULL;

        }
        dev_fd = -1;
    }
    appFatFsUnLock();


    return (dev_fd);
}

/*
 *  ======== _close ========
 */
int _close(int dev_fd)
{
    appFatFsLock();

    if(dev_fd < MAX_OPEN_FILES && filTable[dev_fd] != NULL)
    {

        f_close(filTable[dev_fd]);

        free(filTable[dev_fd]);

        filTable[dev_fd] = NULL;
    }

    appFatFsUnLock();

    return (0);
}

/*
 *  ======== _read ========
 */
ssize_t _read(int dev_fd, void *buf, size_t count)
{
    FRESULT result;
    unsigned int actual = 0;

    appFatFsLock();

    if(dev_fd < MAX_OPEN_FILES && filTable[dev_fd] != NULL)
    {
        result = f_read(filTable[dev_fd], buf, count, &actual);

        if (result == FR_OK) {
        }
        else {
            actual = (-1);
        }
    }
    appFatFsUnLock();

    return ((int)actual);
}

/*
 *  ======== _write ========
 */
ssize_t _write(int dev_fd, const void *buf, size_t count)
{
    FRESULT result;
    unsigned int actual;
    int status = 0;

    appFatFsLock();

    if(dev_fd < MAX_OPEN_FILES && filTable[dev_fd] != NULL)
    {

        if (dev_fd <= 2) {
            /* do log to terminal here */
            status = -1;
        }

        if(status ==  0)
        {
            result = f_write(filTable[dev_fd], buf, count, &actual);

            if (result == FR_OK) {
                status = ((int)actual);
            }
            else {
                status = (-1);
            }
        }
    }
    appFatFsUnLock();

    return status;
}

/*
 *  ======== _lseek ========
 */
off_t _lseek(int dev_fd, off_t offset, int origin)
{
    FRESULT result;

    appFatFsLock();

    if(dev_fd < MAX_OPEN_FILES && filTable[dev_fd] != NULL)
    {
        /* HACK: FatFS has no "tell" functions, so peek inside FIL object */
        if (origin == SEEK_CUR) {
            offset += filTable[dev_fd]->fptr;
        }

        if (origin == SEEK_END) {
            offset += f_size(filTable[dev_fd]);
        }

        result = f_lseek(filTable[dev_fd], offset);

        if (result != FR_OK) {
            offset = (-1);
        }
    }
    appFatFsUnLock();

    return offset;
}

/*
 *  ======== _unlink ========
 */
int _unlink(const char *path)
{
    FRESULT result;
    int status = 0;

    appFatFsLock();

    result = f_unlink(path);

    if (result != FR_OK) {
        status = (-1);
    }
    else {
        status = (0);
    }

    appFatFsUnLock();

    return status;
}

/*
 *  ======== _rename ========
 */
int _rename(const char *old_name, const char *new_name)
{
    FRESULT result;
    int status = 0;

    appFatFsLock();

    result = f_rename(old_name, new_name);

    if (result != FR_OK) {
        status = (-1);
    }
    else {
        status = (0);
    }

    appFatFsUnLock();

    return status;
}

/*
 *  ======== mkdir ========
 */
int mkdir(const char * path, mode_t mode)
{
    int status = 0;

    appFatFsLock();

    (void)mode;

    if (f_mkdir(path) == FR_OK) {
        status = (0);
    }
    else {
        status = (-1);
    }

    appFatFsUnLock();

    return status;
}

/*
 *  ======== rmdir ========
 */
int rmdir(const char * path)
{
    int status = 0;

    appFatFsLock();

    if (f_unlink(path) == FR_OK) {
        status = (0);
    }
    else {
        status = (-1);
    }

    appFatFsUnLock();

    return status;
}
