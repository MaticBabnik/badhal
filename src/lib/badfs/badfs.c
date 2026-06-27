#include "badfs.h"

#define BADFS_MAX_OPEN_FILES 16

#define FLAG_DEAD (1ul << 31)
#define FLAG_EOF (1ul << 0)

i32 errno = BADFS_ERR_OK;
#define RETURN_ERR(err)                                                        \
    do {                                                                       \
        errno = err;                                                           \
        return err;                                                            \
    } while (0)

typedef struct {
    u32 offset;
    u32 size;
    char path[64];
} BadFsFile_t;

typedef struct {
    u32 magic;
    u32 totalSize;
    u32 nFiles;
    BadFsFile_t files[];
} BadFsHeader_t;

static BadFsHeader_t *badfs_base = (BadFsHeader_t *) 0;
static BadFILE open_files[BADFS_MAX_OPEN_FILES];

i32 badfs_mount(void *base) {
    BadFsHeader_t *hdr = (BadFsHeader_t *) base;

    if (hdr->magic != 0x000BADF5) {
        RETURN_ERR(BADFS_ERR_BAD_BADFS);
    }

    badfs_base = hdr;

    for (u32 i = 0; i < BADFS_MAX_OPEN_FILES; i++) {
        open_files[i].flag = FLAG_DEAD;
    }

    RETURN_ERR(BADFS_ERR_OK);
}

u32 badfs_nfiles() {
    if (badfs_base == 0) {
        return 0;
    }

    return badfs_base->nFiles;
}

i32 badfs_filedesc(u32 n, BadFsFileInfo_t *info) {
    if (badfs_base == 0) {
        RETURN_ERR(BADFS_ERR_BAD_BADFS);
    }

    if (n >= badfs_base->nFiles) {
        RETURN_ERR(BADFS_ERR_ENOENT);
    }

    BadFsFile_t *file = &badfs_base->files[n];
    info->offset = file->offset;
    info->size = file->size;
    for (u32 i = 0; i < 64; i++) {
        info->path[i] = file->path[i];
        if (file->path[i] == 0) {
            break;
        }
    }

    RETURN_ERR(BADFS_ERR_OK);
}

// TODO: move this to a string library?
static i32 strncmp(const char *s1, const char *s2, u32 n) {
    for (u32 i = 0; i < n; i++) {
        if (s1[i] != s2[i]) {
            return (i32) ((u8) s1[i] - (u8) s2[i]);
        }
        if (s1[i] == 0) {
            return 0;
        }
    }

    return 0;
}

BadFILE *badfs_open(const char *restrict path, const char *restrict mode) {
    if (badfs_base == 0) {
        errno = BADFS_ERR_ENOENT;
        return NULL; // no fs = no file :)
    }

    // validate mode
    if (mode[0] != 'r') {
        errno = BADFS_ERR_EROFS;
        return NULL; // read-only fs
    }

    // find a free file "descriptor"
    BadFILE *f = 0;
    for (u32 i = 0; i < BADFS_MAX_OPEN_FILES; i++) {
        if (open_files[i].flag & FLAG_DEAD) {
            f = &open_files[i];
            break;
        }
    }
    if (f == 0) {
        errno = BADFS_ERR_EMFILE;
        return NULL; // too many open files
    }

    // check if the file exists
    BadFsFile_t *file = 0;
    for (u32 i = 0; i < badfs_base->nFiles; i++) {
        if (strncmp(badfs_base->files[i].path, path, 64) == 0) {
            file = &badfs_base->files[i];
            break;
        }
    }
    if (file == 0) {
        errno = BADFS_ERR_ENOENT;
        return NULL; // file not found
    }

    f->blobptr = (void *) ((u8 *) badfs_base + file->offset);
    f->metaptr = (void *) file;
    f->size = file->size;
    f->offset = 0;
    f->flag = f->size == 0 ? FLAG_EOF : 0;

    errno = BADFS_ERR_OK;
    return f;
}

i32 badfs_close(BadFILE *f) {
    if (f == 0 || (f->flag & FLAG_DEAD)) {
        RETURN_ERR(BADFS_ERR_EBADF);
    }

    f->flag |= FLAG_DEAD;

    RETURN_ERR(BADFS_ERR_OK);
}

u32 badfs_read(void *restrict ptr, u32 size, u32 n, BadFILE *restrict f) {
    if (f == 0 || (f->flag & FLAG_DEAD)) {
        errno = BADFS_ERR_EBADF;
        return 0;
    }

    u32 bytesToRead = size * n;
    if (f->offset + bytesToRead > f->size) {
        bytesToRead = f->size - f->offset;
        f->flag |= FLAG_EOF;
    }

    for (u32 i = 0; i < bytesToRead; i++) {
        ((u8 *) ptr)[i] = ((u8 *) f->blobptr)[f->offset + i];
    }
    f->offset += bytesToRead;

    return bytesToRead / size;
}

u32 badfs_write(
    const void *restrict ptr, u32 size, u32 n, BadFILE *restrict f
) {
    if (f == 0 || (f->flag & FLAG_DEAD)) {
        errno = BADFS_ERR_EBADF;
        return 0;
    }

    errno = BADFS_ERR_EROFS;
    return 0; // read-only fs
}

i32 badfs_tell(BadFILE *f) {
    if (f == 0 || (f->flag & FLAG_DEAD)) {
        errno = BADFS_ERR_EBADF;
        return -1;
    }

    errno = BADFS_ERR_OK;
    return f->offset;
}

i32 badfs_getpos(BadFILE *restrict f, u32 *restrict pos) {
    if (f == 0 || (f->flag & FLAG_DEAD)) {
        errno = BADFS_ERR_EBADF;
        return -1;
    }

    errno = BADFS_ERR_OK;
    return f->offset;
}

i32 badfs_seek(BadFILE *f, i32 off, i32 whence) {
    if (f == 0 || (f->flag & FLAG_DEAD)) {
        RETURN_ERR(BADFS_ERR_EBADF);
    }

    switch (whence) {
    case SEEK_SET:
        f->offset = off;
        break;

    case SEEK_CUR:
        f->offset += off;
        break;

    case SEEK_END:
        f->offset = f->size + off;
        break;

    default:
        RETURN_ERR(BADFS_ERR_EINVAL);
    }
    f->flag &= ~FLAG_EOF;

    RETURN_ERR(BADFS_ERR_OK);
}

i32 badfs_setpos(BadFILE *restrict f, const u32 *restrict pos) {
    if (f == 0 || (f->flag & FLAG_DEAD)) {
        RETURN_ERR(BADFS_ERR_EBADF);
    }

    f->offset = *pos;
    f->flag &= ~FLAG_EOF;

    RETURN_ERR(BADFS_ERR_OK);
}

i32 badfs_rewind(BadFILE *f) {
    if (f == 0 || (f->flag & FLAG_DEAD)) {
        RETURN_ERR(BADFS_ERR_EBADF);
    }

    f->offset = 0;
    f->flag &= ~FLAG_EOF;

    RETURN_ERR(BADFS_ERR_OK);
}

void badfs_clearerr(BadFILE *f) {
    if (f == 0 || (f->flag & FLAG_DEAD)) {
        return;
    }

    f->flag &= ~FLAG_EOF;
}

i32 badfs_eof(BadFILE *f) {
    if (f == 0 || (f->flag & FLAG_DEAD)) {
        RETURN_ERR(BADFS_ERR_EBADF);
    }

    return (f->flag & FLAG_EOF) ? 1 : 0;
}

i32 badfs_error(BadFILE *f) {
    if (f == 0 || (f->flag & FLAG_DEAD)) {
        RETURN_ERR(BADFS_ERR_EBADF);
    }

    return errno;
}

i32 badfs_flush(BadFILE *f) {
    RETURN_ERR(BADFS_ERR_OK); // lol
}