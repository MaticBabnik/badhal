#include "./stdlib.h"
#include "../../hal/badhal.h"
#include "../../lib/sprintf/sprintf.h"

void call_once(int *flag, void (*func)(void)) {
    if (*flag == 0) {
        *flag = 1;
        func();
    }
}

double atof(const char *nptr) {}

int atoi(const char *nptr) {}

long int atol(const char *nptr) {}

long long int atoll(const char *nptr) {}

int strfromd(
    char *restrict s, size_t n, const char *restrict format, double fp
) {}

int strfromf(char *restrict s, size_t n, const char *restrict format, float fp);
int strfroml(
    char *restrict s, size_t n, const char *restrict format, long double fp
);
double strtod(const char *restrict nptr, char **restrict endptr);
float strtof(const char *restrict nptr, char **restrict endptr);
long double strtold(const char *restrict nptr, char **restrict endptr);
long int strtol(const char *restrict nptr, char **restrict endptr, int base);
long long int
strtoll(const char *restrict nptr, char **restrict endptr, int base);
unsigned long int
strtoul(const char *restrict nptr, char **restrict endptr, int base);
unsigned long long int
strtoull(const char *restrict nptr, char **restrict endptr, int base);

static unsigned int rand_seed = 1;

void srand(unsigned int seed) {
    rand_seed = seed ? seed : 1;
}

int rand(void) {
    rand_seed = (rand_seed * 1103515245 + 12345) & 0x7fffffff;
    return rand_seed;
}


void *aligned_alloc(size_t alignment, size_t size);
void *calloc(size_t nmemb, size_t size);
void free(void *ptr);
void free_sized(void *ptr, size_t size);
void free_aligned_sized(void *ptr, size_t alignment, size_t size);
void *malloc(size_t size);
void *realloc(void *ptr, size_t size);

void abort(void) {
    // TODO: print "ABORT CALLED"
    sys_reboot();
}

#define ATEXIT_MAX 8

void (*arr_fn_atexit[ATEXIT_MAX])(void);
void (*arr_fn_atquickexit[ATEXIT_MAX])(void);
u32 arr_fn_atexit_count = 0, arr_fn_atquickexit_count = 0;

static void call_exit_handlers(void (*arr_fn[])(void), u32 *arr_fn_count) {
    for (u32 i = *arr_fn_count - 1; i < ATEXIT_MAX; i--) {
        arr_fn[i]();
    }
}

int atexit(void (*func)(void)) {
    if (arr_fn_atexit_count >= ATEXIT_MAX) {
        return -1;
    }

    arr_fn_atexit[arr_fn_atexit_count++] = func;
    return 0;
}

int at_quick_exit(void (*func)(void)) {
    if (arr_fn_atquickexit_count >= ATEXIT_MAX) {
        return -1;
    }

    arr_fn_atquickexit[arr_fn_atquickexit_count++] = func;
    return 0;
}

void exit(int status) {
    // TODO: print "EXIT CALLED"

    call_exit_handlers(arr_fn_atexit, &arr_fn_atexit_count);
    sys_reboot();
}

void _Exit(int status) {
    exit(status);
}

char *getenv(const char *name) {
    return NULL; // TODO: maybe allow users to set envflags?
}

void quick_exit(int status) {
    // TODO: print "QUICK EXIT CALLED"
    call_exit_handlers(arr_fn_atquickexit, &arr_fn_atquickexit_count);
    sys_reboot();
}

int system(const char *string) {
    sys_trap("system called");
}

void *bsearch(
    const void *key,
    void *base,
    size_t nmemb,
    size_t size,
    int (*compar)(const void *, const void *)
) {

}

void qsort(
    void *base,
    size_t nmemb,
    size_t size,
    int (*compar)(const void *, const void *)
) {}

int abs(int j) {
    return j < 0 ? -j : j;
}

long int labs(long int j) {
    return j < 0l ? -j : j;
}

long long int llabs(long long int j) {
    return j < 0ll ? -j : j;
}

div_t div(int numer, int denom) {
    return (div_t){
        .quot = numer / denom,
        .rem = numer % denom,
    };
}

ldiv_t ldiv(long int numer, long int denom) {
    return (ldiv_t){
        .quot = numer / denom,
        .rem = numer % denom,
    };
}

lldiv_t lldiv(long long int numer, long long int denom) {
    return (lldiv_t){
        .quot = numer / denom,
        .rem = numer % denom,
    };
}

size_t memalignment(const void *p) {
    return (size_t) p & -(size_t) p;
}