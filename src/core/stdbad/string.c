#include "./string.h"
#include "./ctype.h"
#include "./stdlib.h"

typedef union {
    u8 *b;
    u16 *h;
    u32 *w;
} cpy_ptr_t;


void *memcpy(void *restrict dest, const void *restrict src, size_t n) {
    if (n == 0 || dest == src) return dest;

    cpy_ptr_t d, s;
    d.b = (u8 *) dest;
    s.b = (u8 *) src;

    // Detect aligned case
    if (((u32) dest & 0x3) == ((u32) src & 0x3) && n >= 4) {
        // copy to alignment
        while (((u32) d.b & 0x3) && n) {
            *d.b++ = *s.b++;
            --n;
        }

        // fast copy 4 bytes at a time
        while (n >= 4) {
            *d.w++ = *s.w++;
            n -= 4;
        }
    }

    // Dumb copy
    while (n) {
        *d.b++ = *s.b++;
        --n;
    }

    return dest;
}

void *memccpy(void *restrict dest, const void *restrict src, int c, size_t n) {
    // no optimized copy for memccpy cuz it's a piece of shit

    u8 *d = (u8 *) dest;
    u8 *s = (u8 *) src;

    while (n--) {
        *d = *s;
        if (*s == (u8) c) return d + 1;
        ++d;
        ++s;
    }

    return NULL;
}

void *memmove(void *dest, const void *src, size_t n) {
    if (((u32) src + n) > (u32) dest) {
        u8 *d = (u8 *) dest;
        u8 *s = (u8 *) src;

        // no optimized copy for overlap :)
        // use overflow as exit condition :)
        for (u32 i = n - 1; i < n; i--) {
            d[i] = s[i];
        }

        return dest;
    }

    // optimized-ish copy for non-overlap case
    return memcpy(dest, src, n);
}

char *strcpy(char *restrict dest, const char *restrict src) {
    memccpy(dest, src, 0, 0xffffffff);
    return dest;
}

char *strncpy(char *restrict dest, const char *restrict src, size_t n) {
    size_t i = 0;
    for (; i < n && src[i]; ++i) dest[i] = src[i];
    for (; i < n; ++i) dest[i] = 0;
    return dest;
}

char *strdup(const char *s) {
    size_t len = strlen(s);
    char *s2 = malloc(len + 1);
    if (!s2) return NULL;
    memcpy(s2, s, len + 1);
    return s2;
}

char *strndup(const char *s, size_t n) {
    char *s2 = malloc(n + 1);
    if (!s2) return NULL;
    strncpy(s2, s, n);
    s2[n] = '\0';
    return s2;
}

char *strcat(char *restrict dest, const char *restrict src) {
    char *d = dest;

    while (*d)
        ++d;

    while (*src)
        *d++ = *src++;

    *d = 0;
    return dest;
}

char *strncat(char *restrict dest, const char *restrict src, size_t n) {
    // footgun the function
    char *d = dest;

    while (*d)
        ++d;

    while (n && *src) {
        *d++ = *src++;
        --n;
    }

    *d = 0;
    return dest;
}

int memcmp(const void *s1, const void *s2, size_t n) {
    const unsigned char *p1 = s1, *p2 = s2;

    while (n--) {
        if (*p1 != *p2) return (i32) *p1 - (i32) *p2;
        ++p1;
        ++p2;
    }

    return 0;
}

int strcmp(const char *s1, const char *s2) {
    for (;;) {
        if (*s1 != *s2) return (i32) *s1 - (i32) *s2;
        if (!*s1) return 0;
        ++s1;
        ++s2;
    }
}

int strcoll(const char *s1, const char *s2) {
    for (;;) {
        char c1 = tolower(*s1);
        char c2 = tolower(*s2);

        if (c1 != c2) return (i32) c1 - (i32) c2;
        if (!c1) return 0;
        ++s1;
        ++s2;
    }
}

int strncmp(const char *s1, const char *s2, size_t n) {
    for (;;) {
        if (!n) return 0;
        if (*s1 != *s2) return (i32) *s1 - (i32) *s2;
        if (!*s1) return 0;
        ++s1;
        ++s2;
        --n;
    }
}

size_t strxfrm(char *restrict dest, const char *restrict src, size_t n) {
    size_t len = 0;
    for (;;) {
        char c = tolower(*src);
        if (len < n) dest[len] = c;
        if (!*src) return len;
        ++src;
        ++len;
    }
}

void *memchr(const void *s, int c, size_t n) {
    const u8 *p = (const u8 *) s;
    for (size_t i = 0; i < n; ++i) {
        if (p[i] == (u8) c) return (void *) &p[i];
    }

    return NULL;
}

char *strchr(const char *s, int c) {
    while (*s) {
        if (*s == (char) c) return (char *) s;
        ++s;
    }

    if (c == 0) return (char *) s;

    return NULL;
}

char *strpbrk(const char *s, const char *breakset) {
    while (*s) {
        const char *b = breakset;
        while (*b) {
            if (*s == *b) return (char *) s;
            ++b;
        }
        ++s;
    }

    return NULL;
}

char *strrchr(const char *s, int c) {
    const char *last = NULL;

    while (*s) {
        if (*s == (char) c) last = s;
        ++s;
    }

    if (c == 0) return (char *) s;

    return (char *) last;
}

size_t strspn(const char *s, const char *charset) {
    u32 n = 0;
    for (;;) {
        const char *c = charset;
        if (!*s) return n;

        while (*c) {
            if (*s == *c) break;
            ++c;
        }

        if (!*c) return n;

        ++n;
        ++s;
    }

    return n;
}

size_t strcspn(const char *s, const char *charset) {
    u32 n = 0;
    for (;;) {
        const char *c = charset;
        if (!*s) return n;

        while (*c) {
            if (*s == *c) return n;
            ++c;
        }

        ++n;
        ++s;
    }

    return n;
}

char *strstr(const char *haystack, const char *needle) {
    if (!*needle) return (char *) haystack;

    for (;;) {
        const char *h = haystack;
        const char *n = needle;

        while (*h && *n && *h == *n) {
            ++h;
            ++n;
        }

        if (!*n) return (char *) haystack;

        if (!*h) return NULL;

        ++haystack;
    }
}

char *strtok(char *restrict str, const char *restrict delim) {
    // strtok is a warcrime; punish the user
    *((u32 *) NULL);
    return NULL;
}

void *memset(void *s, int c, size_t n) {
    u8 *p = (u8 *) s;

    // the compiler can try to optimize this :)
    // inb4 SCB_CFSR_USG_UNALIGNED
    while (n--) {
        *p++ = (u8) c;
    }

    return s;
}

void *memset_explicit(void *s, int c, size_t n) {
    volatile u8 *p = (volatile u8 *) s;

    while (n--) {
        *p++ = (u8) c;
    }

    a_dmb(); // this is a bit overkill tbh

    return s;
}

static char err[32] = "Error: xxxxxxxx";
#define ERR_HEX_IDX 7

// very goofy error stringifier :)
char *strerror(int errnum) {
    if (errnum == 0) return "OK";
    u32 e = errnum;

    for (int i = ERR_HEX_IDX; i < ERR_HEX_IDX + 8; ++i) {
        u8 nibble = e >> 28;
        if (nibble < 10)
            err[i] = '0' + nibble;
        else
            err[i] = 'A' + (nibble - 10);
        e <<= 4;
    }

    return err;
}

size_t strlen(const char *s) {
    size_t len = 0;
    while (*s++)
        ++len;
    return len;
}

size_t strnlen(const char *s, size_t n) {
    size_t len = 0;
    while (n-- && *s++)
        ++len;
    return len;
}
