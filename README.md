# DCRsystem_base_Jetson

[DCRsystem_base](../DCRsystem_base) の **Jetson Orin Nano Super
(JetPack 6.2 / L4T R36.4.7 / Ubuntu 22.04 / Python 3.10.12) 向け派生**。
Windows版とはPythonパッケージの調達元・バージョンが異なるため、`pyproject.toml` /
`uv.lock` / `.python-version` を別に持つ独立ディレクトリとして管理している。

サクランボ選別ライン向けの外観検査・自動仕分けシステム。4台のカメラでサクランボを撮影し、
YOLO による物体検出で健全 / 障害を二値判定、判定結果に応じてリレー経由で仕分け弁を駆動する。

## ⚠️ Jetson実機での動作前に確認すること

- **`module_relay.py` のリレーボード制御にはY2C社のLinux用ドライバのインストールが必要**。
  Y2C社公式マニュアル（[動作環境（Linux）](https://www.y2c.co.jp/docs/softwaremanual/ub/spec/linux/)）
  によると `RLY-P4/2/0B-UBT` はLinux（Ubuntu / Raspberry Pi / Jetson Nano等）に対応しており、
  Windows版と同じ関数（`YdciOpen`/`YdciClose`/`YdciRlyOutput`等）を持つ共有ライブラリ
  `/usr/lib/y2c/libydci.so` が提供される。コード側は対応済み（`ctypes.CDLL`で読み込むよう分岐）。

  ドライバ本体（`libydci.so-3.4.0.tar.gz`、Y2C社より入手）にはインストーラー
  `install.sh` が同梱されており、実行アーキテクチャを自動判定してJetson(aarch64)向けの
  `lib/arm64/libydci.so.3.4.0` を配置してくれる。手動でファイルを置く必要はない。

  ```bash
  # 1. libusb-1.0を先に入れておく（installerが依存する。マニュアル記載の必須パッケージ）
  sudo apt install libusb-1.0-0

  # 2. ドライバをJetsonへ転送してから展開・インストール（root権限必須）
  tar xzf libydci.so-3.4.0.tar.gz
  cd libydci
  sudo ./install.sh
  ```

  `install.sh` は `/usr/lib/y2c/libydci.so` の配置に加えて、USBデバイスへの読み書き権限を
  一般ユーザーに許可するudevルールも設定する。反映のためUSBケーブルの抜き差しか
  再起動をしておくと確実。

  接続確認:
  ```bash
  uv run --no-sync python -c "import module_relay; c = module_relay.RelayController(); print(c.init())"
  ```
  （USBケーブル接続要）未検証（Jetson実機での動作確認はまだ行っていない）。

## システム構成

```
main.py                      実行専用エントリポイント（GUI起動のみ）
module_main_window_JP.py     ウィンドウ・制御ロジック本体（main.py から分離）
module_gui_JP.py             GUI デザイン定義（レイアウト・カスタムウィジェット）
module_cameras.py      Basler カメラ（pypylon）制御
module_yolo.py               YOLO 推論・状態機械（個体追跡〜健全/障害の二値判定）
module_motor_serial.py       Arduino とのシリアル通信（ターンテーブル制御）
module_relay.py              リレーボード制御（判定結果に応じた仕分け弁の駆動、上記のとおり要対応）
module_patlite.py            パトライト制御（システム状態の表示）
hsv_mask_utils.py            HSV マスク生成・整形の共通処理（本番・校正ツール間で共有）
dcr_logger.py                構造化データログ（cycle / health / events / detections）
log_config.py                コンソール・ファイルログの統一設定
telemetry_sources.py         health スレッド向けテレメトリ取得ヘルパー
```

### 判定の流れ

1. `module_cameras.py` が4カメラ（top / under / inside / outside）から映像を取得
2. `module_yolo.py` が HSV マスクで果実領域を検出し、中心窓に入ったフレームのみ YOLO 推論
3. ByteTracker でフレーム間の個体を追跡し、個体が確定した時点で健全/障害を二値判定
   （障害系クラスは1カメラでも検出があれば即座に障害、健全は複数カメラでの一致を要求）
4. `module_relay.py` が判定結果に応じて仕分け弁（運搬弁 / 除去弁）を駆動
5. `dcr_logger.py` が cycle・detections 等をログへ記録

### standalone/ 配下（校正・検証ツール）

本番とは独立して実行できる GUI ツール群。

| ファイル | 用途 |
|---|---|
| `hsv_calibration.py` | 果実検出（HSV存在検出＋推論ゲート）の閾値をスライダーで調整し `json/hsv_config_{cam}.json` へ保存 |
| `relay_calibration.py` | リレー（仕分け弁）の開弁タイミングをスライダーで実機に合わせ込む（`module_relay.py`が動く前提。上記「Jetson実機での動作前に確認すること」参照） |
| `delay_calibration.py` | カメラ表示遅延（delay_seconds）の実測キャリブレーション |
| `hsv_ripeness_classifier.py` | 赤色占有率スコアによる healthy/unripe 振り分けGUI（学習データ整備用） |
| `classification_gui_demo.py` | 実機なしで個体確定〜健全/障害判定の流れを確認するデモ |
| `analyze_cycle_logs.py` | 本番が書いた cycle ログから1個体あたりの実コストを集計・比較する（OS非依存） |
| `project_jetson.py` | 実測したスケール係数をJetson実機の処理時間へ換算する（OS非依存） |
| `run_as_jetson.py` / `jetson_live_probe.py` / `jetson_probe_common.py` | **[Windows PC専用]** Jetson購入判断のためWindows PCをJetson相当に制限して検証したツール。Jetson実機では動作しない（購入判断は完了済みのため参考として残置） |

## セットアップ（Jetson実機）

前提: JetPack 6.2、システムPython 3.10.12 (`/usr/bin/python3`)、
JetPack付属の `tensorrt` (apt経由、実機確認済み: 10.3.0)。

### 1. torchのimportに必要なNVIDIA cuDSSをシステムへ入れる

JetPack本体には含まれない別配布。入れないと `import torch` が
`libcudss.so.0: cannot open shared object file` で失敗する。

https://developer.nvidia.com/cudss-downloads で
Linux / aarch64-jetson / Ubuntu / 22.04 / deb (local) を選び、
表示された `wget` → `sudo dpkg -i` → keyringコピー → `sudo apt-get update` →
`sudo apt-get install cudss` をそのまま実行する。

（`libcusparseLt.so.0` 等の別の共有ライブラリ不足エラーが出た場合は、同様に
cuSPARSELtも入れる: https://developer.nvidia.com/cusparselt-downloads ）

### 2. venvを作る（システムのtensorrtを共有するため必須）

```bash
uv venv --system-site-packages --python /usr/bin/python3
```

`--python` を省略すると、uvが独自にダウンロードした別バージョンのPythonが使われ、
`--system-site-packages` を付けてもJetPack付属のtensorrtが見えなくなる。

### 3. パッケージを入れる

```bash
uv sync
```

torch / torchvision / onnxruntime-gpu は `pyproject.toml` の `[tool.uv.sources]` で
Jetson AI Lab索引 (`https://pypi.jetson-ai-lab.io/jp6/cu126`) に固定してあるため、
`uv sync` 一発でJetson向けのGPU対応版が入る。

### 4. 動作確認

```bash
uv run python -c "import torch, torchvision, tensorrt, onnxruntime as ort; \
  print(torch.__version__, torch.cuda.is_available()); \
  print(torchvision.__version__); print(tensorrt.__version__); \
  print(ort.__version__, ort.get_available_providers())"
```

`torch.cuda.is_available()` が `True`、`ort.get_available_providers()` に
`CUDAExecutionProvider`（できれば`TensorrtExecutionProvider`も）が含まれていれば成功。

## 実行

```bash
uv run main.py
```

（`--no-sync`は不要。元の`DCRsystem5goki_app_base`ディレクトリではWindows向け
`pyproject.toml`/`uv.lock`自体がJetsonでは不正な内容だったため、`uv run`の自動同期が
環境を壊す原因になっていた。このディレクトリは`pyproject.toml`/`uv.lock`自体が最初から
Jetson向けの正しい内容なので、`uv run`の自動同期はむしろ環境を正しく保つ側に働く）

## Windows版との差分

Windows版 [pyproject.toml](../DCRsystem5goki_app_base/pyproject.toml) との主な違い:

| パッケージ | Windows版 | Jetson版 | 理由 |
|---|---|---|---|
| torch | ==2.12.1 (pytorch-cu126索引) | ==2.11.0 (Jetson AI Lab索引) | Windows向けcu126索引はx86_64のみ。Jetson AI Lab索引でaarch64向けに提供されている最新版を採用 |
| torchvision | ==0.27.1 | ==0.26.0 | 同上（torchとの組み合わせ） |
| onnxruntime-gpu | >=1.23.2,<1.24 | ==1.24.0 | Windows版の上限はPyPI標準にcp310 aarch64 wheelが無いための制約。Jetson AI Lab索引には1.24.0のaarch64 wheelがある |
| pyside6 | >=6.11.1 | ==6.8.0.2 | aarch64 wheelが `manylinux_2_39`(glibc>=2.39)用でJetPack6.2(glibc2.35)に入らないため、入る最後の版に固定 |
| tensorrt | >=11.1.0.106 (pip) | 依存に含めない | Jetson向けpip wheelが無い。JetPack付属のシステムパッケージを`--system-site-packages`で共有 |

2026-08-24時点、Pythonパッケージ層（torch/torchvision/tensorrt/onnxruntime-gpu）はJetson実機で
動作確認済み（`CUDAExecutionProvider`/`TensorrtExecutionProvider`も利用可能）。ただし当時は
`requirements-jetson.txt`による旧方式での確認であり、この`pyproject.toml`/`uv.lock`構成での
実機再確認はまだ行っていない。`module_relay.py`のLinux対応（Y2C製ドライバ経由）も未検証。
