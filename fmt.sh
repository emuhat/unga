#!/bin/sh
find ./Core -iname '*.c' -o -iname '*.h' | xargs clang-format -i
