# FLASH loader protocol

## Host -> Board

- `begin(size)` (starts a transaction; size is mostly for tracking progress)
- `data(n, bytes[])` (sends `n` bytes of data; append only)
  - `n` must be nonzero, even, and `<=512`
  - the write must not span a page boundary (start and end byte must fall in the same 512B page)
- `verify(crc32)` (computes CRC of transaction and ends it)

## Board -> Host

- `ok`
- `err(id)` (nonzero id)

## Example

`>` is host, `<` is board

```txt
>begin(1234)
* erases 1234 aligned to 8K *
<ok
>data(512, ...)
* writes 512B
<ok
>data(512, ...)
* writes 512B
<ok
>data(11, ...)
* writes 11B
<ok
>verify(0xf00dbabe)
*reads back all written data and computes CRC32*
<err(ERR_CRC)
*host notifies user of failure*
```

## Encoding

Every packet starts with 32bit start-of-packet marker `41 80 01 aa` (random-ish magic bytes).
Followed by 8bit command ID.
There are the following commands:

- `00` - board->host `ok`
- `01` - board->host `err`
- `80` - host->board `begin`
- `81` - host->board `data`
- `82` - host->board `verify`

Messages with bodies have them encoded right after, like so:

```cpp
struct Ok {}; // no body
struct Err {u8 id;};
struct Begin {u32 size;};
struct Data {u16 n; u8 bytes[n];};
struct Verify {u32 crc32;};
```

Note: u32s are little-endian and unaligned.
