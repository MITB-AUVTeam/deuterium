#!/bin/bash

set -e

# Directory containing this script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Build using ninja
"$HOME/.pico-sdk/ninja/v1.12.1/ninja" -C "$SCRIPT_DIR/pico/build"

# Flash using picotool
sudo "$HOME/.pico-sdk/picotool/2.2.0-a4/picotool/picotool" load \
    "$SCRIPT_DIR/pico/build/pico.elf" -fx