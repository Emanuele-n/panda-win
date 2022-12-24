# Build Libraries
Here are listed the external libraries used in the project and the references on how to build them.</br>

Note: open cmd as administrator to avoid possible problems while installing with vcpkg

## poco
Can be build either with vcpck with the usual commands:

	cd path/to/vcpck
	vcpkg install poco:x64-windows
	vcpck integrate install

or by following the official [guide](https://pocoproject.org/download.html)


## eigen3
Can be build either with vcpck with the usual commands:

	cd path/to/vcpck
	vcpkg install eigen3:x64-windows
	vcpck integrate install

or by following the official [guide](https://robots.uc3m.es/installation-guides/install-eigen.html)


## boost
Can be build either with vcpck with the usual commands:

	cd path/to/vcpck
	vcpkg install boost:x64-windows
	vcpck integrate install

or by following the official [guide](https://www.boost.org/doc/libs/1_62_0/more/getting_started/windows.html)


## FrankaDynModelLib
It is directly integrated in the project with its own headers and source files. <br/>
Even in this case remember to place FrankaPandaDynModelLib_dll.dll in the same directory of the executable to avoid runtime error (or do it in a smarter way).


## libfranka
See [buildLibFranka](./buildLibfranka.md) for how to build it

