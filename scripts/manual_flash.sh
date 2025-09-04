#!/bin/bash
# flash_stm32f0308.sh - Flash STM32F0308-Discovery using OpenOCD

SCRIPT_DIR="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"
# Configuration
STM32F0_CFG="${SCRIPT_DIR}/../openocd/stm32f0.cfg"
BINARY_FILE="$1"

# Check if binary file is provided
if [ -z "$BINARY_FILE" ]; then
    echo "Usage: $0 <binary_file.elf|binary_file.bin>"
    echo "Example: $0 firmware.elf"
    exit 1
fi

# Check if file exists
if [ ! -f "$BINARY_FILE" ]; then
    echo "Error: File '$BINARY_FILE' not found!"
    exit 1
fi

# Determine file type and set appropriate command
if [[ "$BINARY_FILE" == *.elf ]]; then
    FLASH_CMD="program $BINARY_FILE verify reset"
elif [[ "$BINARY_FILE" == *.bin ]]; then
    # For binary files, you need to specify the base address
    FLASH_CMD="program $BINARY_FILE 0x08000000 verify reset"
else
    echo "Error: Unsupported file format. Use .elf or .bin files"
    exit 1
fi

echo "Flashing $BINARY_FILE to STM32F0308-Discovery..."
echo "================================================"

# Execute OpenOCD with flash commands
openocd -f $STM32F0_CFG \
        -c "init" \
        -c "reset halt" \
        -c "flash probe 0" \
        -c "$FLASH_CMD" \
        -c "reset"

# Check if OpenOCD succeeded
if [ $? -eq 0 ]; then
    echo "================================================"
    echo "Flash successful! Board should be running."
else
    echo "================================================"
    echo "Flash failed! Check connections and try again."
    exit 1
fi

