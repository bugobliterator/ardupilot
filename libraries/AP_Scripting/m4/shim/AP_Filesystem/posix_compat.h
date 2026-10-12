#pragma once
/*
  M4 Lua build: the C file calls of the Lua core (io library, require,
  luaL_loadfile) under the names AP_Filesystem/posix_compat.h gives them,
  implemented by m4/m4_file_proxy.cpp over the M7's file server (m4:fs_*):
  the reach of in-process scripting, the "@" filesystems read only. The
  standard streams are no files on the M4: they fail with EBADF. As
  in-process, feof() maps to apfs_ferror(); remove, rename and sprintf are not
  mapped (the os library is not built).
 */
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct apfs_file APFS_FILE;

APFS_FILE *apfs_fopen(const char *pathname, const char *mode);
int apfs_fprintf(APFS_FILE *stream, const char *format, ...) __attribute__((format(printf, 2, 3)));
int apfs_fflush(APFS_FILE *stream);
size_t apfs_fread(void *ptr, size_t size, size_t nmemb, APFS_FILE *stream);
size_t apfs_fwrite(const void *ptr, size_t size, size_t nmemb, APFS_FILE *stream);
int apfs_fputs(const char *s, APFS_FILE *stream);
char *apfs_fgets(char *s, int size, APFS_FILE *stream);
void apfs_clearerr(APFS_FILE *stream);
int apfs_fseek(APFS_FILE *stream, long offset, int whence);
int apfs_ferror(APFS_FILE *stream);
int apfs_fclose(APFS_FILE *stream);
APFS_FILE *apfs_tmpfile(void);
int apfs_getc(APFS_FILE *stream);
int apfs_ungetc(int c, APFS_FILE *stream);
int apfs_feof(APFS_FILE *stream);
long apfs_ftell(APFS_FILE *stream);
APFS_FILE *apfs_freopen(const char *pathname, const char *mode, APFS_FILE *stream);
int apfs_remove(const char *pathname);

#ifdef __cplusplus
}
#endif

#undef stdin
#undef stdout
#undef stderr
#define stdin ((APFS_FILE*)1)
#define stdout ((APFS_FILE*)2)
#define stderr ((APFS_FILE*)3)

#undef BUFSIZ
#define BUFSIZ 256

#ifndef __cplusplus
#define FILE APFS_FILE

#undef clearerr
#undef ferror
#undef feof
#undef getc

#define fopen(p,m) apfs_fopen(p,m)
#define fprintf(stream, format, args...) apfs_fprintf(stream, format, ##args)
#define fflush(s) apfs_fflush(s)
#define fread(ptr,size,nmemb, stream) apfs_fread(ptr, size, nmemb, stream)
#define fwrite(ptr, size, nmemb, stream) apfs_fwrite(ptr, size, nmemb, stream)
#define fputs(s, stream) apfs_fputs(s, stream)
#define fgets(s, size, stream) apfs_fgets(s, size, stream)
#define clearerr(stream) apfs_clearerr(stream)
#define fseek(stream, offset, whence) apfs_fseek(stream, offset, whence)
#define ferror(stream) apfs_ferror(stream)
#define fclose(stream) apfs_fclose(stream)
#define tmpfile() apfs_tmpfile()
#define getc(stream) apfs_getc(stream)
#define ungetc(c, stream) apfs_ungetc(c, stream)
#define feof(stream) apfs_ferror(stream)
#define ftell(stream) apfs_ftell(stream)
#define freopen(pathname, mode, stream) apfs_freopen(pathname, mode, stream)
#endif // __cplusplus
