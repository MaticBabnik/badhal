#!/bin/env bash

find src -type f \( -name '*.c' -o -name '*.h' \) \
    -exec clang-format -i --verbose -style=file -fallback-style=none {} +