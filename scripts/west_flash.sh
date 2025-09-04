#!/usr/bin/env bash

SCRIPT_DIR="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"
if [ -z "$VIRTUAL_ENV" ] || [ "$(basename "$VIRTUAL_ENV")" != "zephyr" ]; then
export PYENV_ROOT="$HOME/.pyenv"
[[ -d $PYENV_ROOT/bin ]] && export PATH="$PYENV_ROOT/bin:$PATH"
eval "$(pyenv init - bash)"
eval "$(pyenv virtualenv-init -)"
  pyenv activate zephyr
fi
[ -z ${ZEPHYR_BASE} ] && source ${HOME}/.local/opt/toolchains/zephyr/zephyr-env.sh
pushd ${SCRIPT_DIR}/../apps/atmospherics
west flash
popd
