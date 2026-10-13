#!/usr/bin/env bash
# Run this ON the board.
# Output is <project dir>/<project name>. Extra arguments go straight to
# make, e.g. ./bela-build.sh projectclean
set -e

BELA_HOME="${BELA_HOME:-/root/Bela}"
PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT="$(basename "$PROJECT_DIR")"

# Bela's Makefile only builds projects/<name>, so anywhere else it would build
# (or fail to find) a different directory than the one this script is in.
if [ "$PROJECT_DIR" != "$BELA_HOME/projects/$PROJECT" ]; then
  echo "Error: $PROJECT_DIR is not directly inside $BELA_HOME/projects/" >&2
  exit 1
fi

# The linker cannot overwrite a binary that is still running.
make QUIET=true --no-print-directory -C "$BELA_HOME" stop

make QUIET=true --no-print-directory -C "$BELA_HOME" PROJECT="$PROJECT" \
  CPPFLAGS="-std=c++1z -I$PROJECT_DIR/include" \
  LDLIBS="$PROJECT_DIR/lib/liblsl.so" \
  LDFLAGS="-Wl,-rpath,$PROJECT_DIR/lib" \
  "$@"

echo "Built $PROJECT_DIR/$PROJECT"
