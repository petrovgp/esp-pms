# ESP-IDF Component: *esp-pms*

[![ESP-IDF](https://img.shields.io/badge/ESP--IDF-%E2%9A%99-blue)](https://github.com/espressif/esp-idf) [![License](https://img.shields.io/github/license/petrovgp/esp-pms
)](LICENSE)

ESP-IDF component for Plantower PMSX003 air quality sensors.

## Setup

Clone this repository in your project **components directory** (or create one if it is missing):

```Shell
cd ~/esp-idf-projects/myproject/components
git clone https://github.com/petrovgp/esp-pms.git
```

Add components directory to **project main CMakeLists.txt**:

```CMake
cmake_minimum_required(VERSION 3.16)
set(EXTRA_COMPONENT_DIRS /home/user/esp-idf-projects/myproject/components)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(myproject)
```

## Example usage

Two examples are provided with this component:
- Active queue - sensor usage in active mode with FreeRTOS queues
- Passive sleep - sensor usage in passive mode with sleep state

## License

Distributed under the MIT License.  
See [LICENSE](LICENSE) for more information.

