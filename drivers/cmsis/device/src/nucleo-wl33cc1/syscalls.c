/*********************************************************************
*                    SEGGER Microcontroller GmbH                     *
*                        The Embedded Experts                        *
**********************************************************************
*                                                                    *
*            (c) 2014 - 2019 SEGGER Microcontroller GmbH             *
*                                                                    *
*           www.segger.com     Support: support@segger.com           *
*                                                                    *
**********************************************************************
*                                                                    *
* All rights reserved.                                               *
*                                                                    *
* Redistribution and use in source and binary forms, with or         *
* without modification, are permitted provided that the following    *
* conditions are met:                                                *
*                                                                    *
* - Redistributions of source code must retain the above copyright   *
*   notice, this list of conditions and the following disclaimer.    *
*                                                                    *
* - Neither the name of SEGGER Microcontroller GmbH                  *
*   nor the names of its contributors may be used to endorse or      *
*   promote products derived from this software without specific     *
*   prior written permission.                                        *
*                                                                    *
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND             *
* CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,        *
* INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF           *
* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE           *
* DISCLAIMED.                                                        *
* IN NO EVENT SHALL SEGGER Microcontroller GmbH BE LIABLE FOR        *
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR           *
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT  *
* OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;    *
* OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF      *
* LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT          *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE  *
* USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH   *
* DAMAGE.                                                            *
*                                                                    *
**********************************************************************

-------------------------- END-OF-HEADER -----------------------------

File    : main.c
Purpose : Low level semihosting example

*/
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/fcntl.h>

/*********************************************************************
*
*       Defines, fixed
*
**********************************************************************
*/

#define SYS_OPEN        0x01
#define SYS_CLOSE       0x02
#define SYS_WRITEC      0x03
#define SYS_WRITE0      0x04
#define SYS_WRITE       0x05
#define SYS_READ        0x06
#define SYS_READC       0x07
#define SYS_ISTTY       0x09
#define SYS_SEEK        0x0A
#define SYS_FLEN        0x0C
#define SYS_GET_CMDLINE 0x15

/*********************************************************************
*
*       Global functions
*
**********************************************************************
*/
/*********************************************************************
*
*       BKPT
*/
int __attribute__((optimize("O0"))) BKPT(int op, void* p1, void* p2) {
  register int r0 asm("r0");
  register int r1 asm("r1") __attribute__((unused));
  register int r2 asm("r2") __attribute__((unused));

  r0 = op;
  r1 = (int) p1;
  r2 = (int) p2;

  asm volatile(
      " bkpt 0xAB \n"
      : "=r"(r0) // out
      :// in
      :// clobber
  );
  return r0;
}

int _open(const char *name, int flags, ...) {
    int FileHandle;
    void* block[3];
    int Len;
    int ShMode;

    ShMode = 0;
    //
    // Convert Flags to Semihosting modes
    //
    if (flags & O_RDWR) {
      ShMode |= 2;
    }
    if ((flags & O_CREAT) || (flags & O_TRUNC) || (flags & O_WRONLY)) {
      ShMode |= 4;
    }
    if (flags & O_APPEND) {
      ShMode &= ~4;
      ShMode |= 8;
   }
    //
    // Get filename length
    //
    //ShMode = 6;  // Set mode to w+
    Len = strlen(name);
    block[0] = (void*)name;
    block[1] = (void*)ShMode;
    block[2] = (void*)Len;
    FileHandle = BKPT(SYS_OPEN, (void*) block, (void*) 0);
    return FileHandle;
}

int _close(int file) {
  int r;
  void* block[1];

  block[0] = (void*)file;
  r = BKPT(SYS_CLOSE, (void*)block, (void*) 0);
  return r;
}

int _read(int file, char *ptr, int len) {
  int NumBytesLeft;
  void* block[3];
  if(len == 0) {
    return 0;  // Early out
  }
  block[0] = (void*)file;
  block[1] = (void*)ptr;
  block[2] = (void*)len;
  NumBytesLeft = BKPT(SYS_READ, (void*) block, (void*) 0);
  return NumBytesLeft;
}

int _write(int file, char *buf, int nbytes) {
  int NumBytesLeft;
  void* block[3];

  block[0] = (void*)file;
  block[1] = (void*)buf;
  block[2] = (void*)nbytes;
  NumBytesLeft = BKPT(SYS_WRITE, (void*) block, (void*) 0);
  return NumBytesLeft;
}

int _lseek(int file, int offset, int whence) {
  int r;
  void* block[2];

  block[0] = (void*)file;
  block[1] = (void*)(whence + offset);
  r = BKPT(SYS_SEEK, (void*) block, (void*) 0);
  return r;
}

int _fstat(int file, struct stat *st) {
  (void)file;
  st->st_mode = S_IFCHR;
  return  0;
}

int _isatty(int file) {
  (void)file;
  return  1;
}
