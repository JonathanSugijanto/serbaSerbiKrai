# Windows
# NUCLEO_F446RE , ARCH_MAX
# NUCLEO =  NUCLEO_F446RE
# NUCLEO =  NUCLEO_G0B1RE
NUCLEO =  NUCLEO_F446RE
PROGRAM = testing-compile
DRIVE_NAME = NODE_F446RE
GET_DRIVE_CMD = $(shell wmic volume where "label='$(DRIVE_NAME)'" get driveletter)
DRIVE_LETTER = $(word 2, $(GET_DRIVE_CMD))

all: configure cmake ninja

build: cmake ninja

run: ninja push

configure:
	mbed-tools configure -m $(NUCLEO) -t GCC_ARM --mbed-os-path "../../mbed-os"

cmake:
	cmake -S . -B cmake_build/$(NUCLEO)/develop/GCC_ARM -GNinja

ninja:
	ninja -C cmake_build/$(NUCLEO)/develop/GCC_ARM

clean:
	rmdir /s /q cmake_build

reset:
	make clean
	make configure
	make cmake
	make ninja

# xcopy cmake_build\$(NUCLEO)\develop\GCC_ARM\Slave_Elephant.bin D:\ /I /Y /C /H /R
# wmic volume where "label='NODE_G0B1RE'" get driveletter
# @echo NUCLEO drive letter is $(DRIVE_LETTER)
#xcopy cmake_build\$(NUCLEO)\develop\GCC_ARM\$(PROGRAM).bin $(DRIVE_LETTER)\

# xcopy cmake_build\$(NUCLEO)\develop\GCC_ARM\$(PROGRAM).bin D:\

push:
	xcopy cmake_build\$(NUCLEO)\develop\GCC_ARM\$(PROGRAM).bin E:\


# =======================================

# APP_TARGET_NAME=Testing-Motor
# BOARD_SERIES=NUCLEO_G0B1RE
# MBED_OS_PATH="../mbed-os"

# # if using wsl
# # sudo mkdir /mnt/d/
# MIKON_PATH=/mnt/d/

# # if using window powershell
# # MIKON_PATH=D:\ 
# # copy cmake_build/$(BOARD_SERIES)/develop/GCC_ARM/$(APP_TARGET_NAME).bin $(MIKON_PATH)


# all: configure build

# run: fast_build copy

# prepCopy: prepared_port copy

# configure:
# 	mbed-tools configure -m $(BOARD_SERIES) -t GCC_ARM --mbed-os-path $(MBED_OS_PATH)

# build:
# 	cmake -S . -B cmake_build/$(BOARD_SERIES)/develop/GCC_ARM -GNinja
# 	ninja -C cmake_build/$(BOARD_SERIES)/develop/GCC_ARM

# fast_build:
# 	ninja -C cmake_build/$(BOARD_SERIES)/develop/GCC_ARM

# copy:
# 	xcopy cmake_build\$(NUCLEO)\develop\GCC_ARM\$(PROGRAM).bin $(DRIVE_LETTER)\