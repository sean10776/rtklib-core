# rtklib-core
**Base on [rtklibexplorer/RTKLIB](https://github.com/rtklibexplorer/RTKLIB/archive/refs/tags/b34g.zip)**

## 使用方式
以下是將這個專案匯入到其他專案中的步驟：

1. 在您的專案的 CMakeLists.txt 檔案中，新增以下程式碼：

    ```cmake
    add_subdirectory(rtklib-core)
    ```
   編譯您的專案，CMake 將自動將這個專案作為子專案匯入。
2. 專案有些Option的Define要在`add_subdirectory`前加：
    ```
    options : -DENAGLO   enable GLONASS
              -DENAGAL   enable Galileo
              -DENAQZS   enable QZSS
              -DENACMP   enable BeiDou
              -DENAIRN   enable IRNSS
              -DNFREQ=n  set number of obs codes/frequencies
              -DNEXOBS=n set number of extended obs codes
              -DMAXOBS=n set max number of obs data in an epoch
              -DWIN32    use WIN32 API
              -DWIN_DLL  generate library as Windows DLL
    ```
    `CmakeLists.txt`範例
    ```cmake
    add_compile_definitions(NFREQ=3 NEXOBS=3 ENAGLO ENAGAL ENACMP TRACE)
    add_subdirectory(rtklib-core)
    ```
## 範例
`main.cpp`  
```cpp
#include<rtklib.h>

void main(){

}
```
`CmakeLists.txt`  
```cmake
cmake_minimum_required(VERSION 3.15)

project(main LANGUAGES CXX)

#開全星系跟trace
add_compile_definitions(NFREQ=3 NEXOBS=3 ENAGLO ENAGAL ENACMP TRACE)

add_subdirectory(rtklib-core)

add_executable(main main.cpp)

target_link_libraries(main rtklib-core)
```

## History
* 新增Septentrio PVT解碼
  * ✒️ 新增PVT解碼 ([septentrio.c](./src/rcv/septentrio.c))
* 增加Anrdoid解碼檔
  * ➕ 新增所有解碼實作 ([android.c](./src/rcv/android.c))
  * ✒️ 增加 `STRFMT_ANDROID` ([rtklib.h](./src/rtklib.h))
* 新增TWD97轉換
  * ✒️ 新增 `pos2twd` ([rtkcmn.c](./src/rtkcmn.c))
* 新增Ntou API solution format
  * ✒️ 新增 `outntou`、`outntouold` ([solution.c](./src/solution.c))
* 新增Http request Stream Type
  * ✒️ 新增http request實作 ([stream.c](./src/stream.c))
  * ✒️ 增加 `STR_HTTPREQ` ([rtklib.h](./src/rtklib.h))