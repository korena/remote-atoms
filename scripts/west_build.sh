#!/usr/bin/env bash


SCRIPT_DIR="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"
if [ -z "$VIRTUAL_ENV" ] || [ "$(basename "$VIRTUAL_ENV")" != "zephyr" ]; then
export PYENV_ROOT="$HOME/.pyenv"
[[ -d $PYENV_ROOT/bin ]] && export PATH="$PYENV_ROOT/bin:$PATH"
eval "$(pyenv init - bash)"
eval "$(pyenv virtualenv-init -)"
  pyenv activate zephyr
fi
[ -z ${ZEPHYR_BASE} ] && source ${HOME}/.local/opt/toolchains/zephyr/zephyr-env.sh && source ${HOME}/.local/opt/toolchains/zephyr/zephyr-env.sh 
pushd ${SCRIPT_DIR}/../apps/atmospherics
west build -p always -b stm32f0_atmos --build-dir build -- -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DBOARD=stm32f0_atmos -DBOARD_ROOT=${HOME}/Development/remote-atmos -DDTC_OVERLAY_FILE=boards/arm/stm32f0_atmos.overlay -DEXTRA_ZEPHYR_MODULES=${HOME}/Development/remote-atmos/module -DCMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}:/home/korena/.local/opt/toolchains && cp ./build/compile_commands.json ${SCRIPT_DIR}/../
popd
