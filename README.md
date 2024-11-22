# ssl-embedded

![PlatformIO CI](https://github.com/robocin/ssl-embedded/workflows/PlatformIO%20CI/badge.svg?branch=master)

<img src="./assets/graph.png" height="180">

- Firmware
    - Robot codes with folder for which version.
    - Board: [NUCLEO-F767ZI](https://os.mbed.com/platforms/ST-Nucleo-F767ZI/) 
    - API: [MbedOS](https://os.mbed.com/docs/v5.10/apis/index.html) 
    - Software IDE: [Visual Studio Code](https://code.visualstudio.com/)
        - Programming Extension: [PlatformIO](https://platformio.org/platformio-ide), installed with VS Code package manager.
    - Updating Firmware: [instructions here](https://os.mbed.com/teams/ST/wiki/Nucleo-Firmware)

- Tests 
    - Folder with all tests and auxiliar softwares.

## Dependencies
### - nRF24Communication lib
It's submodule of [communication-embedded](https://github.com/robocin/communication-embedded/).
- So, to clone this repository please use: 

       git clone --recurse-submodules -j8 <link>
- Whener you want to update the nRF24Communication lib to the latest version on [communication-embedded](https://github.com/robocin/communication-embedded/) (master), use:

       git submodule update --init --recursive

### - ST STM32 Platform
The [ST STM32](https://docs.platformio.org/en/latest/platforms/ststm32.html) is a device platform responsible for interface mbed to ARM boards, so it's necessary to install at the PlatformIO extension.

