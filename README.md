# rtklib-core
**Base on [rtklibexplorer/RTKLIB](https://github.com/rtklibexplorer/RTKLIB/archive/refs/tags/b34g.zip)**

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