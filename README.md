# PANDA testing software
This program is intended to be used only for testing some functionalities of the PANDA robot from [Franka Emika](https://www.franka.de/) on Windows using also [CoppeliaSim](https://www.coppeliarobotics.com/). <br/>

## Build Libraries
Before building the project it is necessary to have all the external libraries. <br/>
See [Build libs](libs/README.md) for instructions.

## Build panda_win
~~~
cd path/to/panda_win
mkdir build
cd build
cmake -DBoost_INCLUDE_DIR=C:\dev\vcpkg\installed\x64-windows\include -G "Visual Studio 17 2022" ..
cmake --build . --config Release
~~~
Note: you can use also
~~~
-DCMAKE_TOOLCHAIN_FILE=C:\dev\vcpkg\scripts\buildsystems\vcpkg.cmake
~~~

## Run panda_win
Before running open the [scene](./panda.ttt) in Coppelia and be sure you have all the dll in the same path where the executable is located. To help this steps these are gathered [here](./libs/dllNeeded/), but be sure to update them. <br/>
Run to check the available functionalities 
~~~
path/to/panda_win.exe --help
~~~

## Authors
E. Nicotra with huge thanks to M. Ferro!