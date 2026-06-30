#pragma once
#include <core/numeric.h>

extern i32 __errno;
#define errno __errno
// gotta be standard bro

// C standard error codes

// Mathematics argument out of domain of function.
#define EDOM 1
// Result too large.
#define ERANGE 2

// POSIX error codes

// Argument list too long.
#define E2BIG 1024
// Permission denied.
#define EACCES 1025
// Address in use.
#define EADDRINUSE 1026
// Address not available.
#define EADDRNOTAVAIL 1027
// Address family not supported.
#define EAFNOSUPPORT 1028
// Resource unavailable, try again (may be the same value as [EWOULDBLOCK]).
#define EAGAIN 1029
// Connection already in progress.
#define EALREADY 1030
// Bad file descriptor.
#define EBADF 1031
// Bad message.
#define EBADMSG 1032
// Device or resource busy.
#define EBUSY 1033
// Operation canceled.
#define ECANCELED 1034
// No child processes.
#define ECHILD 1035
// Connection aborted.
#define ECONNABORTED 1036
// Connection refused.
#define ECONNREFUSED 1037
// Connection reset.
#define ECONNRESET 1038
// Resource deadlock would occur.
#define EDEADLK 1039
// Destination address required.
#define EDESTADDRREQ 1040
// Reserved.
#define EDQUOT 1042
// File exists.
#define EEXIST 1043
// Bad address.
#define EFAULT 1044
// File too large.
#define EFBIG 1045
// Host is unreachable.
#define EHOSTUNREACH 1046
// Identifier removed.
#define EIDRM 1047
// Illegal byte sequence.
#define EILSEQ 1048
// Operation in progress.
#define EINPROGRESS 1049
// Interrupted function.
#define EINTR 1050
// Invalid argument.
#define EINVAL 1051
// I/O error.
#define EIO 1052
// Socket is connected.
#define EISCONN 1053
// Is a directory.
#define EISDIR 1054
// Too many levels of symbolic links.
#define ELOOP 1055
// File descriptor value too large.
#define EMFILE 1056
// Too many links.
#define EMLINK 1057
// Message too large.
#define EMSGSIZE 1058
// Reserved.
#define EMULTIHOP 1059
// Filename too long.
#define ENAMETOOLONG 1060
// Network is down.
#define ENETDOWN 1061
// Connection aborted by network.
#define ENETRESET 1062
// Network unreachable.
#define ENETUNREACH 1063
// Too many files open in system.
#define ENFILE 1064
// No buffer space available.
#define ENOBUFS 1065
// No message is available on the STREAM head read queue.
#define ENODATA 1066
// No such device.
#define ENODEV 1067
// No such file or directory.
#define ENOENT 1068
// Executable file format error.
#define ENOEXEC 1069
// No locks available.
#define ENOLCK 1070
// Reserved.
#define ENOLINK 1071
// Not enough space.
#define ENOMEM 1072
// No message of the desired type.
#define ENOMSG 1073
// Protocol not available.
#define ENOPROTOOPT 1074
// No space left on device.
#define ENOSPC 1075
// No STREAM resources.
#define ENOSR 1076
// Not a STREAM.
#define ENOSTR 1077
// Functionality not supported.
#define ENOSYS 1078
// The socket is not connected.
#define ENOTCONN 1079
// Not a directory or a symbolic link to a directory.
#define ENOTDIR 1080
// Directory not empty.
#define ENOTEMPTY 1081
// State not recoverable.
#define ENOTRECOVERABLE 1082
// Not a socket.
#define ENOTSOCK 1083
// Not supported (may be the same value as [EOPNOTSUPP]).
#define ENOTSUP 1084
// Inappropriate I/O control operation.
#define ENOTTY 1085
// No such device or address.
#define ENXIO 1086
// Operation not supported on socket (may be the same value as [ENOTSUP]).
#define EOPNOTSUPP 1087
// Value too large to be stored in data type.
#define EOVERFLOW 1088
// Previous owner died.
#define EOWNERDEAD 1089
// Operation not permitted.
#define EPERM 1090
// Broken pipe.
#define EPIPE 1091
// Protocol error.
#define EPROTO 1092
// Protocol not supported.
#define EPROTONOSUPPORT 1093
// Protocol wrong type for socket.
#define EPROTOTYPE 1094
// Read-only file system.
#define EROFS 1096
// Invalid seek.
#define ESPIPE 1097
// No such process.
#define ESRCH 1098
// Reserved.
#define ESTALE 1099
// Stream ioctl() timeout.
#define ETIME 1100
// Connection timed out.
#define ETIMEDOUT 1101
// Text file busy.
#define ETXTBSY 1102
// Operation would block (may be the same value as [EAGAIN]).
#define EWOULDBLOCK 1103
// Cross-device link.
#define EXDEV 1104
