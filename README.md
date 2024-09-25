# 环境配置

## C/C++

```
sudo apt-get install gcc g++ gdb vim 
gcc -v 
g++ -v
gdb -v
g++ -o hello hello.cpp
```

## [Vscode](https://code.visualstudio.com/) 

```
sudo dpkg -i code_1.89.0-1714530869_amd64.deb
插件C/C++ Chinese cmake github coderunner
```

### test

```
#include<iostream>
using namespace std;
 
int main()
{
    cout << "Hello VScode ubder Ubuntu!!" << endl;
    return 0;
}
```

### .vscode

#### launch.json:

```
{
    "version": "0.2.0",
    "configurations": [
    {
        "name": "(gdb) Launch",
        "type": "cppdbg",
        "request": "launch",
        "program": "${workspaceFolder}/${fileBasenameNoExtension}.out",
        "args": [],
        "stopAtEntry": false,
        "cwd": "${workspaceFolder}",
        "environment": [],
        "externalConsole": true,
        "MIMode": "gdb",
        "preLaunchTask": "build",
        "setupCommands": [
            {
            "description": "Enable pretty-printing for gdb",
            "text": "-enable-pretty-printing",
            "ignoreFailures": true
            }
        ]
    }
    ]
}
```

#### settings.json:

```
{
    "files.associations": {
        "ostream": "cpp",
        "iostream": "cpp",
        "array": "cpp",
        "atomic": "cpp",
        "bit": "cpp",
        "*.tcc": "cpp",
        "cctype": "cpp",
        "clocale": "cpp",
        "cmath": "cpp",
        "compare": "cpp",
        "concepts": "cpp",
        "cstdarg": "cpp",
        "cstddef": "cpp",
        "cstdint": "cpp",
        "cstdio": "cpp",
        "cstdlib": "cpp",
        "cwchar": "cpp",
        "cwctype": "cpp",
        "deque": "cpp",
        "string": "cpp",
        "unordered_map": "cpp",
        "vector": "cpp",
        "exception": "cpp",
        "algorithm": "cpp",
        "functional": "cpp",
        "iterator": "cpp",
        "memory": "cpp",
        "memory_resource": "cpp",
        "numeric": "cpp",
        "random": "cpp",
        "string_view": "cpp",
        "system_error": "cpp",
        "tuple": "cpp",
        "type_traits": "cpp",
        "utility": "cpp",
        "initializer_list": "cpp",
        "iosfwd": "cpp",
        "istream": "cpp",
        "limits": "cpp",
        "new": "cpp",
        "numbers": "cpp",
        "stdexcept": "cpp",
        "streambuf": "cpp",
        "typeinfo": "cpp"
    }
}
```

#### tasks.json:

```
{
    "version": "2.0.0",
    "tasks": [
    {
    "label": "build",
    "type": "shell",
    "command": "g++",
    "args": ["-g", "${file}", "-std=c++11", "-o", "${fileBasenameNoExtension}.out"]
    }
    ]
}
```

#### F5调试

## [arm-none-eabi-gcc交叉编译工具链](https://developer.arm.com/downloads/-/gnu-rm)

```
vim ~/.bashrc
export PATH=$PATH:/home/yml/mondrian/arm-gnu-toolchain-13.2.Rel1-x86_64-arm-none-eabi/bin
source ~/.bashrc
```

## [Jlink](https://www.segger.com/downloads/jlink/)

```
sudo dpkg -i JLink_Linux_V792o_x86_64.deb
jlink --version
cd /opt/SEGGER/JLink
```

### 交互式

```
JLinkExe
“connect”
STM32F427IIH6
SWD或JTAG
4000kHz
loadbin ./test.bin,0x08000000
loadfile ./test.hex
q //退出command模式
```

## [camke](https://cmake.org/download/)

```
tar -zxvf cmake-3.27.5.tar.gz
cd cmake-3.27.5
./bootstrap
make
sudo make install
cmake --version
```

## [Cubmax](https://www.st.com/en/development-tools/stm32cubemx.html)

```
chmod +x SetupSTM32CubeMX-<version>-linux.run
./SetupSTM32CubeMX-<version>-linux.run

sudo nano ~/usr/share/applications/stm32cubemx.desktop
[Desktop Entry]
Version=1.0
Name=STM32CubeMX
Exec=/home/your_username/STMicroelectronics/STM32CubeMX/STM32CubeMX
Icon=/home/your_username/STMicroelectronics/STM32CubeMX/icon.png
Type=Application
Categories=Development;

sudo apt install default-jre
java -version

```

## [Ozone](https://www.segger.com/downloads/jlink/#Ozone)

```
sudo dpkg -i Ozone_Linux_V326_x86_64.deb
ozone
```

### .svd文件的寻找--官网

- 搜索
- CAD资源

# 使用

```
cmake -B build -S . -G Ninja(默认make)
cmake --build build -- -j8 
cd build && ninja Flash && cd .. | Download
```

## ozone

- **Watched Data**

  - **Graph**

- #### Break&Tracepoints

  - **View**-->**General**-->**Break&Tracepoints**”
  - 管理断点

- #### Timeline

  - **View**-->**Advanced**-->**Timeline**
  - 查看变量 设置采样频率

- **View**-->**Advanced**-->

  - 设置更高的采样频率

- **调试保存**

