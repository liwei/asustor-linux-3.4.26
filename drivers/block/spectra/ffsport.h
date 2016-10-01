/*
 * NAND Flash Controller Device Driver
 * Copyright (c) 2008-2010, Intel Corporation and its suppliers.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 */



#ifndef _FFSPORT_
#define _FFSPORT_

#include "spectraswconfig.h"
#include "ffsdefs.h"

//
#if defined __GNUC__
  typedef unsigned char                byte;
  typedef unsigned short int           uint16;      /* 16-bit (always) */
  typedef unsigned long int            uint32;      /* 32-bit (always) */

#ifdef SUPPORT_LARGE_FILESYS
  typedef unsigned long long int       uint64;      /* 64-bit (always) */
  typedef long long int                int64;       /* 64-bit (always) */
#endif

  #define PACKED
  #define PACKED_GNU __attribute__ ((packed))
  #define UNALIGNED

#elif defined __BORLANDC__
  #define _stricmp stricmp
  #define _strnicmp strnicmp

  typedef unsigned __int8              byte;
  typedef unsigned __int16             uint16;      /* 16-bit (always) */
  typedef unsigned __int32             uint32;      /* 32-bit (always) */
#ifdef SUPPORT_LARGE_FILESYS
  typedef unsigned __int64             uint64;      /* 64-bit (always) */
  typedef __int64                      int64;       /* 64-bit (always) */
#endif

  #define PACKED     /* packed by default (unless -a used); no keyword */
  #define PACKED_GNU
  #define UNALIGNED

#elif defined __HIGHC__
  typedef unsigned char                byte;
  typedef unsigned short int           uint16;      /* 16-bit (always) */
  typedef unsigned long int            uint32;      /* 32-bit (always) */
#ifdef SUPPORT_LARGE_FILESYS
  typedef unsigned long long int       uint64;      /* 64-bit (always) */
  typedef long long int                int64;       /* 64-bit (always) */
#endif

  #define PACKED _Packed
  #define PACKED_GNU
  #define UNALIGNED _Unaligned

#elif defined ARMC
  typedef unsigned char                byte;
  typedef unsigned short int           uint16;      /* 16-bit (always) */
  typedef unsigned long int            uint32;      /* 32-bit (always) */
#ifdef SUPPORT_LARGE_FILESYS
  typedef unsigned long long           uint64;      /* 64-bit (always) */
  typedef long long                    int64;       /* 64-bit (always) */
#endif

  #define PACKED _packed
  #define PACKED_GNU
  #define UNALIGNED

#elif defined _MSC_VER
  typedef unsigned __int8              byte;
  typedef unsigned __int16             uint16;      /* 16-bit (always) */
  typedef unsigned __int32             uint32;      /* 32-bit (always) */
  typedef unsigned __int64             uint64;      /* 64-bit (always) */
  typedef __int64                      int64;       /* 64-bit (always) */

  #pragma pack(1)
  #define PACKED
  #define PACKED_GNU
  #define UNALIGNED

#elif defined __MWERKS__
  typedef unsigned char                byte;
  typedef unsigned short               uint16;      /* 16-bit (always) */
  typedef unsigned long                uint32;      /* 32-bit (always) */
  typedef unsigned long long           uint64;      /* 64-bit (always) */
  typedef long long                    int64;       /* 64-bit (always) */

#ifdef __cplusplus__
  extern "C"
  {
#endif
      int _stricmp(const char *__s1, const char *__s2);
      int _strnicmp(const char *__s1, const char *__s2, int __n);
#ifdef __cplusplus__
  }
#endif

  #pragma pack(1)
  #define PACKED
  #define PACKED_GNU
  #define UNALIGNED

#else
/*#error Define PACKED and UNALIGNED macros for your compiler.*/

  typedef unsigned char                byte;
  typedef unsigned short int           uint16;      /* 16-bit (always) */
  typedef unsigned long int            uint32;      /* 32-bit (always) */
#ifdef SUPPORT_LARGE_FILESYS
  typedef unsigned long long int       uint64;      /* 64-bit (always) */
  typedef long long int                int64;       /* 64-bit (always) */
#endif

  #define PACKED
  #define PACKED_GNU
  #define UNALIGNED

  int _stricmp(const char *__s1, const char *__s2);
  int _strnicmp(const char *__s1, const char *__s2, int __n);
#endif


#ifdef SUPPORT_LARGE_FILESYS
  #define ADDRESSTYPE uint64
#else
  #define ADDRESSTYPE uint32
#endif


extern int GLOB_Calc_Used_Bits(unsigned int n);
extern ADDRESSTYPE GLOB_u64_Div(ADDRESSTYPE addr, uint32 divisor);
extern ADDRESSTYPE GLOB_u64_Remainder(ADDRESSTYPE addr, uint32 divisor_type);




#ifdef _SMX
  #include "smx.h"
  typedef SCB_PTR                      FFS_LOCK_HANDLE;
  typedef TCB_PTR                      FFS_TASK_HANDLE;
#if VERBOSE
  #define print printf
#else
  #define print(fmt, args... )
#endif
#elif defined _WIN32
  #include <string.h>   /* for strcpy(), stricmp(), etc */
  #include <stdlib.h>   /* for malloc(), free() */
  #include <stdio.h>
  typedef void *                       FFS_LOCK_HANDLE;
  typedef void *                       FFS_TASK_HANDLE;
#if VERBOSE
  #define print printf
#else
#define print
#endif
#elif defined LINUX_DEVICE_DRIVER
#include <linux/version.h>


 #include "linux/semaphore.h"


  typedef struct semaphore *           FFS_LOCK_HANDLE;
  typedef void *                       FFS_TASK_HANDLE;
  #include <linux/string.h>  /* for strcpy(), stricmp(), etc */
  #include <linux/mm.h>      /* for kmalloc(), kfree() */
  #include <linux/vmalloc.h>
  #define malloc(s)	vmalloc(s)
  #define free(s)	vfree(s)
 #include <linux/module.h>
 #include <linux/moduleparam.h>
 #include <linux/init.h>

 #include <linux/kernel.h> /* printk() */
 #include <linux/fs.h>     /* everything... */
 #include <linux/slab.h>
 #include <linux/errno.h>  /* error codes */
 #include <linux/types.h>  /* size_t */
 #include <linux/genhd.h>
 #include <linux/blkdev.h>
 #include <linux/hdreg.h>
 #include "flash.h"
#if VERBOSE
  #define print(fmt, args... )	printk( KERN_DEBUG "Spectra: " fmt, ## args)
#else
  #define print(fmt, args... )   do { } while (0)
#endif

 #define LINUX_KERNEL_2_6_22    (2622)
 #define LINUX_KERNEL_2_6_24    (2624)


 #define LINUX_KERNEL_VER       LINUX_KERNEL_2_6_22

#else
  #include <string.h>   /* for strcpy(), stricmp(), etc */
  #include <stdlib.h>   /* for malloc(), free() */
  #include <stdio.h>
  typedef void *                       FFS_LOCK_HANDLE;
  typedef void *                       FFS_TASK_HANDLE;
#if VERBOSE
  #define print printf
#else
#define print(fmt,args...) do { } while (0)
#endif
#endif


#ifdef SUPPORT_BIG_ENDIAN
#define INVERTUINT16(w)   ((uint16)(((uint16)(w)) << 8) | (uint16)((uint16)(w) >> 8))
#define INVERTUINT32(dw) (((uint32)(dw) << 24) | (((uint32)(dw) << 8) & 0x00ff0000) | (((uint32)(dw) >> 8) & 0x0000ff00) | ((uint32)(dw) >> 24))
#else
#define INVERTUINT16(w)   w
#define INVERTUINT32(dw) dw
#endif

#ifdef __cplusplus
extern "C"
{
#endif

typedef int (*ISR_FN_T)(int, void *);
extern FFS_LOCK_HANDLE GLOB_CREATE_FFS_API_LOCK (void);
extern void            GLOB_RELEASE_FFS_API_LOCK (FFS_LOCK_HANDLE handle);
extern int             GLOB_FFS_API_ENTER (FFS_LOCK_HANDLE handle);
extern int             GLOB_FFS_API_EXIT (FFS_LOCK_HANDLE handle);
extern int             GLOB_SYSTEM_FATAL_ERROR (void);
extern void           *GLOB_MALLOC (ADDRESSTYPE size);
extern void            GLOB_FREE (void *ptr);
extern void            GLOB_SCHEDULE_FUNCTION (ISR_FN_T fn);
extern int             GLOB_ISR(int dummy1, void *dummy2);
extern uint32         *GLOB_MEMMAP_NOCACHE(unsigned long addr, unsigned long size);
extern void 			GLOB_MEMUNMAP_NOCACHE(volatile uint32 *addr);
extern uint32         *GLOB_MEMMAP_TOBUS(uint32 *ptr);

#ifdef __cplusplus
}
#endif

#if SEAMLESS
int exit_sim(void);
int embedded_strprintf(char *m);
int printf(const char *format,...);
int reset_flash_memory_device(void);
#if VERBOSE
#define printstr embedded_strprintf
#else
#define printstr(m)
#endif
#else
#define printstr(m) print("%s", m)
#endif

#endif // _FFSPORT_
