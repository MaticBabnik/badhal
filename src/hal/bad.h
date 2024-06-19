#include <stddef.h>

#define R_RW volatile       // read-write
#define R_WO volatile       // write-only
#define R_RO volatile const // read-only

typedef signed char i8;
typedef unsigned char u8;
typedef short i16;
typedef unsigned short u16;
typedef int i32;
typedef unsigned int u32;
typedef float f32;