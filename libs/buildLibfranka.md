# Build libfranka using vcpkg and Visual Studio

1. Go to https://frankaemika.github.io/docs/installation_windows.html for the official installation guide

2. Install vcpkg from https://vcpkg.io/en/getting-started.html and be sure to have the english version of Visual Studio

3. Open cmd as admin: 
	~~~
	cd path\to\vcpck 
	vcpck install eigen3
	vcpck install poco
	vcpkg integrate install
	~~~
	
4. git clone --recursive https://github.com/frankaemika/libfranka <br/>Be sure the libfranka version in compatible with the robot system version (which can be checked from the Desk). See [this](https://frankaemika.github.io/docs/compatibility.html) for reference. In case you can clone a specific version X.Y.Z as
	~~~
	git clone --branch X.Y.Z --recursive https://github.com/frankaemika/libfranka
	~~~

5. Open CMakelist.txt and set BUILD_TEST=OFF 	

6. Be sure you are in path\to\libfranka\
	~~~
	mkdir build
	cd build
	cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=C:\dev\vcpkg\scripts\buildsystems\vcpkg.cmake -G "Visual Studio 17 2022" ..
	cmake --build . --config Release
	~~~
	Note: You can use a different compiler but everything has been tested with Visual Studio 17 2022 <br/>
	Note: You can specify manually where to find Poco and Eigen3 as:
	~~~
	cmake -DCMAKE_BUILD_TYPE=Release -DPoco_DIR=path\to\vcpkg\installed\x64-windows\share\poco -DEigen3_DIR=path\to\vcpkg\installed\x64-windows\share\eigen3 -G "Visual Studio 17 2022" ..
	~~~

7. You may need to install zlib and pcre2 or unofficial-pcre.

   If this is the case, install them with 
    ~~~
	vcpkg install zlib
	vcpkg install pcre2
	vcpkg install unofficial-pcre
	vcpkg integrate install
	~~~

   and append the following options to the cmake command, or use -DCMAKE_TOOLCHAIN_FILE=C:\dev\vcpkg\scripts\buildsystems\vcpkg.cmake
   	~~~
	cmake -DCMAKE_BUILD_TYPE=Release -DPoco_DIR=C:\dev\vcpkg\installed\x64-windows\share\poco -DEigen3_DIR=C:\dev\vcpkg\installed\x64-windows\share\eigen3 -DZLIB_INCLUDE_DIR=C:\dev\vcpkg\installed\x64-windows\include -DZLIB_LIBRARY=C:\dev\vcpkg\installed\x64-windows\lib\zlib.lib -DPCRE2_LIBRARY_RELEASE=C:\dev\vcpkg\installed\x64-windows\lib\pcre2-8.lib -DPCRE2_INCLUDE_DIR=C:\dev\vcpkg\installed\x64-windows\include -Dunofficial-pcre_DIR=C:\dev\vcpkg\installed\x64-windows\share\unofficial-pcre -G "Visual Studio 17 2022" ..
	~~~






