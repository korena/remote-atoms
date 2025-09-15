# Setup

user manual: https://www.st.com/resource/en/user_manual/um1658-discovery-kit-for-stm32f030-value-line-microcontrollers-stmicroelectronics.pdf


## 1. openocd setup
```bash
> git clone --depth 1 --branch v0.12.0 --single-branch https://github.com/openocd-org/openocd.git
> cd ./openocd
> sudo apt install libtool libusb-1.0-0-dev pkg-config
> ./bootstrap
> ./configure --enable-stlink --prefix ${HOME}/.local/
> make -j 4
> make install
> sudo cp contrib/60-openocd.rules /etc/udev/rules.d/
> sudo udevadm control --reload-rules
# test (all Jumpurs on the board are ON)
> openocd -f ./tcl/interface/stlink.cfg -f ./tcl/target/stm32f0x.cfg
Open On-Chip Debugger 0.12.0-g9ea7f3d (2025-09-01-13:31)
Licensed under GNU GPL v2
For bug reports, read
        http://openocd.org/doc/doxygen/bugs.html
Info : auto-selecting first available session transport "hla_swd". To override use 'transport select <transport>'.
Info : The selected transport took over low-level target control. The results might differ compared to plain JTAG/SWD
Info : Listening on port 6666 for tcl connections
Info : Listening on port 4444 for telnet connections
Info : clock speed 1000 kHz
Info : STLINK V2J32S0 (API v2) VID:PID 0483:3748
Info : Target voltage: 2.897630
Info : [stm32f0x.cpu] Cortex-M0 r0p0 processor detected
Info : [stm32f0x.cpu] target has 4 breakpoints, 2 watchpoints
Info : starting gdb server for stm32f0x.cpu on 3333
Info : Listening on port 3333 for gdb connections
```
## 2. Zephyr rtos

```bash
sudo apt install --no-install-recommends ninja-build gperf ccache dfu-util device-tree-compiler file libsdl2-dev libmagic1
pyenv virtualenv 3.12.11 zephyr
pyenv activate zephyr
mkdir -p ${HOME}/.local/opt/toolchains/ && cd ${HOME}/.local/opt/toolchains
git clone --depth 1 --single-branch --branch zephyr-v3.5.0 https://github.com/zephyrproject-rtos/zephyr.git
python -m pip install -r scripts/requirements-base.txt
python -m pip install --no-cache-dir west
# enable only modules suitable for STM32F0308
cp ~/Development/remote-atmos/stm32f0/west.yml ~/.local/opt/toolchains/zephyr/
cd ~/.local/opt/toolchains
# instatiate west workspace and install tools
west init -l zephyr
west update --narrow -o=--depth=1
# install module-specific blobs, this is based on stm32f0/west.yml
west blobs fetch hal_stm32
```

## 3. Zephyr SDK

See compatibility matrix here: https://docs.google.com/spreadsheets/d/1wzGJLRuR6urTgnDFUqKk7pEB8O6vWu6Sxziw_KROxMA/edit?gid=0#gid=0

```bash
# Install minimal Zephyr SDK
cd ~/.local/opt/toolchains/
wget https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v0.16.9/zephyr-sdk-0.16.9_linux-x86_64_minimal.tar.xz
tar xf zephyr-sdk-0.16.9_linux-x86_64_minimal.tar.xz
rm zephyr-sdk-0.16.9_linux-x86_64_minimal.tar.xz
cd ${HOME}/.local/opt/toolchains/zephyr-sdk-0.16.9
./setup.sh # follow prompts, arm-zephyr-eabi and host tools
sudo cp ./sysroots/x86_64-pokysdk-linux/usr/share/openocd/contrib/60-openocd.rules /etc/udev/rules.d
```

## 4. Port stm32f0_disco to stm32f0_atmos with correct soc
See the resulting board under board/arm/stm32f0_atmos

## 5. Basic application and testing
```bash
cd ~/.local/opt/toolchains/zephyr
pyenv activate zephyr
source zephyr-env.sh
west build -p always -b stm32f0_atmos samples/basic/blinky -- -DBOARD_ROOT=${HOME}/Development/remote-atmos
west flash
```
## 6. Freestanding application

Created apps/atmospherics

### 6.1 Build with cmake
```bash
pyenv activate zephyr
source  ${HOME}/.local/opt/toolchains/zephyr/zephyr-env.sh
cd remote-atmos/apps/atmospherics
cmake -G "Ninja" -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DBOARD=stm32f0_atmos -DBOARD_ROOT=${HOME}/Development/remote-atmos -DDTC_OVERLAY_FILE=boards/arm/stm32f0_atmos.overlay -B build -S . && cp ./build/compile_commands.json ./
cmake --build build
cd remote-atmos
./scripts/flash.sh ./apps/atmospherics/build/zephyr/zephyr.bin
```
### 6.2 Build with west
```bash
pyenv activate zephyr
source  ~/.local/opt/toolchains/zephyr/zephyr-env.sh
cd remote-atmos/apps/atmospherics
west build -p always -b stm32f0_atmos --build-dir build -s . -- -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DBOARD=stm32f0_atmos -DBOARD_ROOT=${HOME}/Development/remote-atmos -DDTC_OVERLAY_FILE=boards/arm/stm32f0_atmos.overlay && cp ./build/compile_commands.json ./
west flash
```

# 7. Console

| USB TO TTL | STM32f0  |
| ---------- | -------- |
| RX         | TX (PA9) |
| TX         | RX (PA10)|
| GND        | GND      |

```bash
sudo minicom -D /dev/tty.usbserial-B004431P
```
