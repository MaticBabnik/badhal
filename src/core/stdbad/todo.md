# stdbad implementation status

Both headers are currently just prototype dumps (declared in `.h`, redeclared
unimplemented in `.c`). Classification below is to plan implementation order.

## stdlib.h

### Trivial stubs (no OS/env on a freestanding MCU — return failure/no-op)
- `system` — no shell to exec
- `getenv` — no environment block
- `at_quick_exit`, `quick_exit` — no OS-level quick-exit notion; can alias to `atexit`/`exit` or stub
- `mblen` — no multibyte/locale support, return -1
- `call_once` — not even standard `stdlib.h` (threads-ish), irrelevant without threads

### Normal impl (self-contained algorithms, no new infra)
- `atoi`, `atol`, `atoll`, `atof` — thin wrappers over `strtol`/`strtod`
- `strtol`, `strtoll`, `strtoul`, `strtoull` — base 2-36 + auto-detect parse loop
- `strtod`, `strtof`, `strtold` — float parsing, fiddlier but self-contained
- `strfromd`, `strfromf`, `strfroml` — float -> string, can reuse `stb_sprintf`
- `rand`, `srand` — simple PRNG (xorshift/LCG) + static seed state
- `abs`, `labs`, `llabs` — one-liners
- `div`, `ldiv`, `lldiv` — one-liners (quot/rem)
- `bsearch`, `qsort` — generic `void*`/comparator algorithms
- `atexit` — static fixed-size table of function pointers, no allocation
- `abort` — infinite loop / breakpoint / system reset
- `exit`, `_Exit` — run `atexit` handlers (`exit` only) then halt/reset
- `memalignment` — pure pointer arithmetic against alignment

### Non-trivial impl (need new infra — a heap allocator)
- `malloc`, `calloc`, `realloc`, `aligned_alloc` — need a real allocator (bump/free-list over a static arena)
- `free`, `free_sized`, `free_aligned_sized` — depend on the same allocator's bookkeeping

Priority note: malloc/free family is the one real blocker for the
[Doom port](../../../../../.claude/projects/-home-babnik-git-badhal/memory/project_doom_port_plan.md)
(doomgeneric wants a heap).

---

## stdio.h

There's now a real backing store: `src/lib/badfs/badfs.c` is a mounted,
read-only blob filesystem with its own `BadFILE` (open file table, offset,
size, EOF flag) and `badfs_open/close/read/write/seek/tell/...` functions.
It even ships a `BADFS_STDIO_COMPAT` macro mode that `#define`s `fopen` etc.
straight onto `badfs_*`. That alias mode is NOT sufficient as the real
implementation, because:

- `badfs_write` always returns `EROFS` (fs is read-only) — fine for real
  files, wrong for `stdout`/`stderr`, which must actually emit bytes
- `stdin`/`stdout`/`stderr` aren't badfs files at all (no `metaptr`/`blobptr`),
  so badfs's open-file table can't represent them

So `stdio.c` needs real (non-macro) wrapper functions that dispatch per
stream: real `FILE*` → delegate to `badfs_*`; the 3 console pseudo-streams →
go to USART3 (`usart_send_string`, see `Main.c`) for write, USART3 RX for
read. This likely means `FILE` becomes a small tagged wrapper (badfs handle
vs. console id) rather than a bare `BadFILE` alias.

### Trivial stubs (don't make sense even with badfs mounted)

- `remove`, `rename` — badfs is read-only and has no rename/unlink support
- `tmpfile`, `tmpnam` — no writable storage to put a temp file on
- `freopen` — no real use case without a writable fs

### Normal impl (delegate to badfs for files; USART3 + stb_sprintf for console)

- `fopen`, `fclose` — `fopen` delegates to `badfs_open` (mode must be `"r"`-ish); console streams are pre-opened singletons, `fclose` on them is a no-op/error
- `fread` — delegates to `badfs_read` for real files; undefined/error on stdin for now (or USART3 RX loop later)
- `fwrite`, `fputc`, `putc`, `fputs`, `putchar`, `puts` — if stream is stdout/stderr, go to USART3; otherwise `EROFS` via `badfs_write` (files are read-only)
- `fgetc`, `getc`, `getchar` — stdin → USART3 RX; real file → `badfs_read` one byte
- `fgets` — line read, same dispatch as `fgetc`
- `fseek`, `fsetpos`, `fgetpos`, `ftell`, `rewind` — delegate straight to `badfs_seek`/`badfs_tell`/etc.; error on console streams (no seeking a UART)
- `clearerr`, `feof`, `ferror` — delegate to `badfs_clearerr`/`badfs_eof`/`badfs_error` for files; console streams get a tiny static flag each
- `fflush` — `badfs_flush` (currently a no-op) for files; flush UART TX for stdout/stderr
- `setbuf`, `setvbuf` — can be honest no-ops (accept and ignore) since neither badfs nor the UART path buffers yet
- `printf`, `vprintf` — `stb_sprintf` into a buffer + USART3
- `fprintf`, `vfprintf` — same as printf but dispatch like `fputs` above (only makes sense on stdout/stderr until writable files exist)
- `sprintf`, `vsprintf`, `snprintf`, `vsnprintf` — direct `stb_sprintf` wrappers, no stream involved
- `ungetc` — 1-byte pushback slot per stream (console: static slot; file: just decrement offset)
- `perror` — `fputs` of message + errno text (`BADFS_ERR_*` strings) to stderr

### Non-trivial impl (need new infra — real scanf parsing)

- `scanf`, `vscanf`, `sscanf`, `vsscanf`, `fscanf`, `vfscanf` — need a genuine
  format-string parser with conversion-spec state machine (numbers, skip
  whitespace, `%n`, width/precision) — much more involved than the sprintf
  side, nothing to reuse from `stb_sprintf`. Reading from stdin additionally
  needs a byte-at-a-time USART3 RX path with blocking/line-buffering decided.
