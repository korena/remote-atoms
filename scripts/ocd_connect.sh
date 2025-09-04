#!/usr/bin/env bash

SCRIPT_DIR="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"
OPENOCD_DIR=${SCRIPT_DIR}/../openocd
openocd -f ${OPENOCD_DIR}/stm32f0.cfg

