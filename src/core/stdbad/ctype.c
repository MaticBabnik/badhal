#include "./ctype.h"

// ctype impl for ascii

i32 isalnum(i32 c) {
    return isalpha(c) || isdigit(c);
}

i32 isalpha(i32 c) {
    return ('A' <= c && c <= 'Z') || ('a' <= c && c <= 'z');
}

i32 isblank(i32 c) {
    return c == ' ' || c == '\t';
}

i32 iscntrl(i32 c) {
    return c < 0x20 || c == 0x7F;
}

i32 isdigit(i32 c) {
    return '0' <= c && c <= '9';
}

i32 isgraph(i32 c) {
    return '!' <= c && c <= '~';
}

i32 islower(i32 c) {
    return 'a' <= c && c <= 'z';
}

i32 isprint(i32 c) {
    return ' ' <= c && c <= '~';
}

i32 ispunct(i32 c) {
    return (c >= '!' && c <= '/') || (c >= ':' && c <= '@')
           || (c >= '[' && c <= '`') || (c >= '{' && c <= '~');
}

i32 isspace(i32 c) {
    return (c >= '\t' && c <= '\r') || c == ' ';
}

i32 isupper(i32 c) {
    return 'A' <= c && c <= 'Z';
}

i32 isxdigit(i32 c) {
    return isdigit(c) || ('A' <= c && c <= 'F') || ('a' <= c && c <= 'f');
}

i32 tolower(i32 c) {
    if (isupper(c)) {
        return c + ('a' - 'A');
    }
    return c;
}

i32 toupper(i32 c) {
    if (islower(c)) {
        return c - ('a' - 'A');
    }
    return c;
}
