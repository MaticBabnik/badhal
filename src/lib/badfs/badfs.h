#pragma once
#include <core/bad.h>

// TODO: move to lib/stdbad/errno.h
#define BADFS_ERR_OK 0
#define BADFS_ERR_ENOENT 1
#define BADFS_ERR_EACCES 2
#define BADFS_ERR_ENOSPC 3
#define BADFS_ERR_EBADF 4
#define BADFS_ERR_EINVAL 5
#define BADFS_ERR_EIO 6
#define BADFS_ERR_EROFS 7
#define BADFS_ERR_ENOMEM 8
#define BADFS_ERR_ENAMETOOLONG 9
#define BADFS_ERR_EMFILE 10
#define BADFS_ERR_BAD_BADFS 1000
extern i32 errno;

i32 badfs_mount(void *base);

typedef struct {
    u32 offset;
    u32 size;
    char path[64];
} BadFsFileInfo_t;

u32 badfs_nfiles();
i32 badfs_filedesc(u32 n, BadFsFileInfo_t *info);

typedef struct {
    void *blobptr;
    void *metaptr;
    u32 size;
    u32 offset;
    u32 flag;
} BadFILE;

#define SEEK_SET 0
#define SEEK_CUR 1
#define SEEK_END 2

BadFILE *badfs_open(const char *restrict path, const char *restrict mode);
i32 badfs_close(BadFILE *f);

u32 badfs_read(void *restrict ptr, u32 size, u32 n, BadFILE *restrict f);
u32 badfs_write(const void *restrict ptr, u32 size, u32 n, BadFILE *restrict f);

i32 badfs_tell(BadFILE *f);
i32 badfs_getpos(BadFILE *restrict f, u32 *restrict pos);
i32 badfs_seek(BadFILE *f, i32 off, i32 whence);
i32 badfs_setpos(BadFILE *restrict f, const u32 *restrict pos);
i32 badfs_rewind(BadFILE *f);

void badfs_clearerr(BadFILE *f);
i32 badfs_eof(BadFILE *f);
i32 badfs_error(BadFILE *f);

i32 badfs_flush(BadFILE *f);

#ifdef BADFS_STDIO_COMPAT
#define FILE BadFILE

#define fopen badfs_open
#define fclose badfs_close
#define fread badfs_read
#define fwrite badfs_write
#define ftell badfs_tell
#define fgetpos badfs_getpos
#define fseek badfs_seek
#define fsetpos badfs_setpos
#define rewind badfs_rewind
#define clearerr badfs_clearerr
#define feof badfs_eof
#define ferror badfs_error
#define fflush badfs_flush
#endif