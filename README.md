# DCRsystem_base_Jetson

[DCRsystem_base](../DCRsystem_base) の **Jetson Orin Nano Super
(JetPack 6.2 / L4T R36.4.7 / Ubuntu 22.04 / Python 3.10.12) 向け派生**。
Windows版とはPythonパッケージの調達元・バージョンが異なるため、`pyproject.toml` /
`uv.lock` / `.python-version` を別に持つ独立ディレクトリとして管理している。

サクランボ選別ライン向けの外観検査・自動仕分けシステム。4台のカメラでサクランボを撮影し、
YOLO による物体検出で健全 / 被害を二値判定、判定結果に応じてリレー経由で仕分け弁を駆動する。

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
module_gui_JP.py             GUI デザイン定義（レイアウト・クラス表示名 CLASS_DISPLAY）
module_cameras.py            Basler カメラ（pypylon）制御
module_yolo.py               YOLO 推論・状態機械（個体追跡〜健全/被害の二値判定）
module_motor_serial.py       Arduino とのシリアル通信（ターンテーブル制御）
module_relay.py              リレーボード制御（判定結果に応じた仕分け弁の駆動、上記のとおり要対応）
module_patlite.py            パトライト制御（システム状態の表示）
hsv_mask_utils.py            HSV マスク生成・整形の共通処理（本番・校正ツール間で共有）
dcr_logger.py                構造化データログ（cycle / health / events / detections）
log_config.py                コンソール・ファイルログの統一設定
telemetry_sources.py         health スレッド向けテレメトリ取得ヘルパー

Arduino/turntable_control/   Arduino UNO スケッチ（ターンテーブル・非常停止・モード切替）
cam_pfs/                     カメラ個体ごとの Pylon 設定（{論理名}_{シリアル番号}.pfs）
json/                        校正ツールが書き出す設定（HSV・リレー角度・カメラ遅延）
Trained_Models/              YOLO 重み（TensorRT engine / PyTorch pt）
standalone/                  校正・検証ツール（下記）
```

### 用語

| 用語 | 意味 | コード上の対応 |
|---|---|---|
| 健全果 | 出荷対象。移送弁へ送る | `is_damaged = False` → `RelayChannel.TRANSPORT` |
| 被害果 | 除去対象。未熟果・無判定を含む | `is_damaged = True` → `RelayChannel.REMOVE` |
| 移送 | 健全果をラインへ送り出すこと | `TRANSPORT` (ch0) |
| 除去 | 被害果をラインから外すこと | `REMOVE` (ch1) |
| 個体 | 1個のサクランボ。cycle ログ1行に対応する | `cycle_id` |
| 帯（中心窓） | 推論ゲート。ROI 中心から ±`infer_window_px()` の範囲 | `INFER_FRAMES_PER_CAM` |
| 票 | カメラ1台ぶんの判定結果。被害票 / 健全票 | `_aggregate_per_cam` |
| 無判定 | HSV は捉えたが YOLO が一度も検出しなかった個体 | `yolo_no_det_flag = 1` |
| 単体モード | Arduino が PC なしでターンテーブルを回す運転モード | `STANDALONE` |
| PCモード | PC の GUI から制御する運転モード | `PC_MODE` |

English 識別子は過去ログとの互換のため従来のまま（被害 = `damage` / `is_damaged`、移送 = `TRANSPORT`）。

### 処理の流れ

```
撮影 → HSV存在検出 → 推論ゲート(中心窓) → YOLO推論 → ByteTrack → 個体確定
  → カメラ別に1票へ畳む → 票を統合して健全/被害を二値判定
  → リレー駆動(移送弁 / 除去弁) → cycle ログ
```

1. **撮影** — `module_cameras.py` が Basler 4台（cam_top / cam_under / cam_inside / cam_outside）を
   20fps で取得する。撮影位置が異なる cam_under / cam_inside は `json/delay_config.json` の秒数ぶん
   フレームを遅延キューに溜めてから流し、4カメラが「同じ果実」を同時刻に処理するよう揃える
   （表示だけでなく推論もこの遅延後のフレームを使う）。

2. **HSV存在検出** — `ImageProcessor.get_target_info` が赤系2帯の HSV マスクで果実領域を探す。
   虚像（アクリル反射）除去と果柄除去を掛けたうえで最大ブロブを果実候補とし、左右端に接している
   ブロブは棄却する（上下端は許容）。閾値は `standalone/hsv_calibration.py` で「どんな果実でも確実に
   捉える」よう校正しておく前提で、**HSV 段を通らなかったものは果実ではないとみなして完全に無視する**。
   コスト削減のため `HSV_DETECT_SCALE = 0.5` の縮小画像で処理し、座標・面積はネイティブ座標系へ戻す。

3. **推論ゲート（中心窓）** — 果実の重心が ROI 中心から ±`infer_window_px()` の窓に入っている
   フレームだけを YOLO に回す。窓幅は `fruit_speed_px × INFER_FRAMES_PER_CAM ÷ 2` で自動計算され、
   1カメラあたり `INFER_FRAMES_PER_CAM = 3` 枚（4カメラ合計で最大12枚）で打ち切る。

4. **個体の区切り** — 開始・継続・終了はすべて「帯内で捉えたか」で駆動する。YOLO の検出有無では
   駆動しない（検出できなかった果実も個体として成立させ、ログに残すため）。
   どのカメラも `EMPTY_TIMEOUT_SEC = 0.5` 秒帯内で捉えなければ1個分が通過し終わったとみなして確定する
   （出口ヒステリシス）。帯内可視が `MIN_VISIBLE_SEC = 0.12` 秒未満かつ YOLO 検出も無い瞬間的な
   ノイズは破棄する（入口ヒステリシス）。

5. **推論・追跡** — `model.predict` は `_infer_lock` で直列化する（単一モデルへの同時 predict は
   スレッド安全でないため）。結果はカメラごとに独立した ByteTracker へ通し、`CONF_THRESHOLD = 0.5`
   を超える中で最高信頼度の1件を、そのフレームの検出として記録する。

6. **健全/被害の二値判定** — 個体確定時に2段構えで決める
   （`_aggregate_per_cam` → `_resolve_quality`）。
   1. **カメラ別に1票へ畳む** … そのカメラが healthy 以外を1件でも検出していれば「被害票」、
      していなければ「健全票」。カメラ内は被害優先の OR で、フレーム数やそのカメラの最高信頼度
      クラスは問わない。
      例）cam_top が healthy 0.90 / healthy 0.85 / stemcrack 0.60 → **stemcrack 0.60 の被害票**
   2. **票を統合する** … 被害票が1台でもあれば台数によらず即座に被害（低いハードル）。健全と
      判定するには `HEALTHY_CONFIRM_MIN_CAMS = 2` 台以上の異なるカメラでの healthy 検出を要求する
      （高いハードル）。満たさない場合は見落とし（偽健全）を避けて安全側で被害扱い。
   - **12枚すべてで検出できなかった場合も、健全の確証がゼロなので安全側で被害扱い**
     （`yolo_no_det_flag = 1` として cycle ログに残す）
   - 仕分けに使うのは `is_damaged` であり、`label_name == "healthy"` では判定しない
   - `CLASS_DISPLAY` 未登録クラス（モデルに新クラスが増えた等）も安全側で除去する
   - GUI・ログのカメラ別内訳は判定と同じ `_aggregate_per_cam` の集計から作るため、画面の内訳と
     仕分け判定の根拠が食い違わない

7. **仕分け** — `module_relay.py` が `json/relay_config.json` の実効回転角度から待機時間を出し
   （待機時間 = 1回転の秒数 × 角度 ÷ 360）、該当チャンネルを `RELAY_OPEN_TIME = 0.15` 秒だけ開弁する。

8. **ログ** — `dcr_logger.py` が cycle / detections / health / events を出力する。
   cycle の `decision_reason` 列に「なぜその弁になったか」が残る
   （`damage` / `healthy_confirmed` / `healthy_unconfirmed` / `no_detection` / `unregistered_class`）。
   `healthy_cams[n]` 列には healthy を検出したカメラ台数（0〜4）が入り、`healthy_unconfirmed` の
   原因（0台なのか1台なのか）を切り分けられる。

## デバイス構成と接続

| デバイス | 型式 | 接続 | 識別 |
|---|---|---|---|
| 産業用カメラ ×4 | Basler（pypylon） | USB3 | シリアル番号で論理名に固定割当（下表） |
| リレーボード | Y2C `RLY-P4/2/0B-UBT` | USB | `YdciOpen` にボード名を渡して開く |
| 積層信号灯（パトライト） | NE-USB | USB HID | VID `0x191A` / PID `0x6001` |
| ターンテーブル制御 | Arduino UNO（ATmega328P） | USB シリアル 115200 bps | ポート説明文のキーワードで自動検出（`arduino` / `ch340` / `ch341` / `usb-serial` / `wch` / `ftdi` / `cp210`）。`module_main_window_JP.SERIAL_PORT` で直接指定も可（例 `/dev/ttyACM0`） |
| ステッピングドライバ | DR42A（ミスミ） | Arduino から PUL / DIR / ENA | 非常停止の a接点が ENA+ に入る |

### カメラのシリアル番号割当（`module_cameras.TARGET_SERIALS`）

| 論理名 | シリアル番号 | 設定ファイル |
|---|---|---|
| `cam_top` | 25453227 | `cam_pfs/cam_top_25453227.pfs` |
| `cam_under` | 25453229 | `cam_pfs/cam_under_25453229.pfs` |
| `cam_inside` | 25308967 | `cam_pfs/cam_inside_25308967.pfs` |
| `cam_outside` | 25308968 | `cam_pfs/cam_outside_25308968.pfs` |

帯域上限は**コードで上書きせず PFS の値をそのまま使う**。PFS はカメラ個体ごとに pylon Viewer で
詰めた設定で、帯域もその一部だからである。現行値は cam_top / cam_outside = 163MB/s、
cam_under / cam_inside = 80MB/s（4台とも `DeviceLinkThroughputLimitMode = On`）。実必要帯域は
4台合計で約 83MB/s なのでいずれも余裕がある。変更したいときは PFS を撮り直す
（pylon Viewer → Save Features）。実際に効いている値は起動ログに出る。

### リレーボードのチャンネル割当（`module_relay.RelayChannel`）

| ch | 用途 | 待機角度（`json/relay_config.json`） |
|---|---|---|
| 0 | 移送弁（健全果） | 114.5° |
| 1 | 除去弁（被害果） | 67.5° |

角度は `standalone/relay_calibration.py` で実機に合わせ込む。待機時間は速度レベルに追従する。

待機時間 = （ターンテーブル1回転の秒数）× 角度 ÷ 360。1回転の秒数は
パルス周期 × `PULSE_PER_ROTATION` × ギア比2 で求める。

`module_relay.PULSE_PER_ROTATION = 6400` は DR42A の32分割（1.8°/step → 200step/回転 × 32）。
**実機検証済み（2026-09-11）**: speed=6 で1回転の実測が約12.8秒で、計算値
`0.001s × 6400 × 2 = 12.8秒` と一致する（3200 なら6.4秒）。この実測で「分割数 × ギア比」の積が
確定している。

この定数は `module_relay.py` を唯一の定義元とし、`standalone/relay_calibration.py` と
`standalone/delay_calibration.py` も同じ値を参照する。ドライバの分割設定を変えたときは
`module_relay.py` の1か所だけ直せばよい。

### Arduino UNO のピン割当（`Arduino/turntable_control/turntable_control.ino`）

| ピン | 方向 | 用途 | 備考 |
|---|---|---|---|
| D9 | 出力 | ステップパルス（PUL） | Timer1 の OC1A に固定割当のため **変更不可** |
| D8 | 出力 | 回転方向（DIR） | `setup()` で HIGH。逆転させたい場合はここを LOW に |
| D5 | 入力（内部プルアップ） | 非常停止 検知 | 24V 短絡事故で D3 が故障したため移設 |
| D6 | 入力（内部プルアップ） | モード切替 | **LOW = PCモード / HIGH = 単体モード**。D4 故障のため移設 |
| D7 | 出力 | 状態LED | D5 を非常停止に使うため移設。非常停止中 = 点滅 / 回転中 = 点灯 / 停止中 = 消灯 |

非常停止スイッチ（オムロン製）は a接点・c接点を1ブロックずつ使う。

- **モーター停止回路（a接点）**: `DR42A の ENA+ → a接点 → Arduino 5V`。
  押すと電位差が無くなり励磁解除 ＝ 電源遮断（Arduino を介さないハード的な停止）
- **通知・表示回路（c接点）**: `Arduino GND → c接点 → Arduino D5`。
  D5 は内部プルアップなので、非押下時は c接点が閉じて LOW、押すと開いて HIGH になる。
  スケッチは `ESTOP_NC = false` なので **HIGH ＝ 非常停止 作動** と解釈する。
  検知すると PC へ `ESTOP` を通知し、状態LED（D7）を点滅させる

入力は両方とも `DEBOUNCE_MS = 20` ミリ秒のデバウンスを通してから確定する。

### Arduino ⇔ PC のシリアルプロトコル

PC → Arduino（`module_motor_serial.py` が送信）:

| コマンド | 動作 |
|---|---|
| `R` | 回転開始（非常停止中は `ERR:ESTOP` を返して拒否） |
| `S` | 停止 |
| `V<1-10>` | 速度変更（例 `V7`。大きいほど速い） |
| `C` | 停止して速度を基準値 6 へ初期化 |
| `P` | 疎通確認（`OK:PONG`） |
| `Q` | 非常停止状態の問い合わせ（`ESTOP` / `ESTOP_CLEARED` を返す） |

Arduino → PC（非同期通知。受信スレッドがコールバックで GUI へ渡す）:

| 通知 | 意味 |
|---|---|
| `READY` | 起動完了（PC 側はこの行で初期化完了を判定） |
| `ESTOP` / `ESTOP_CLEARED` | 非常停止の作動 / 解除 |
| `STANDALONE` / `PC_MODE` | 単体モードへ移行 / PCモードへ復帰。単体モード中は GUI の操作系をロックする |

### パトライトの状態表示（`module_patlite.SystemState`）

| システム状態 | 表示 | 意味 |
|---|---|---|
| `INITIALIZING` | 黄 点灯 | デバイス接続中 |
| `STANDBY` | 赤 点灯 | 接続完了・待機中 |
| `RUNNING` | 緑 点灯 | 正常運転中 |
| `ESTOP` | 紫 点灯 | 非常停止中（人が押した＝監視下なのでブザーは鳴らさない） |
| `ERROR` | 赤 点滅 | 想定外異常（カメラ切断等）で停止 |
| `STANDALONE` | 消灯 | Arduino が PC なしで動作中 |

ブザーは `BUZZER_VOLUME = 0`（消音）で全状態とも鳴らさない設定。
LED 制御値は Byte5 の上位ニブル＝色 / 下位ニブル＝点灯パターン。

## 校正・検証ツール（standalone/）

本番とは独立して実行できる GUI ツール群。判定・表示ロジックは本番の関数をそのまま束縛して使うため、
本番を変更すればツール側の挙動も追従する（二重実装によるズレが起きない）。

| ファイル | 用途 |
|---|---|
| `hsv_calibration.py` | 果実検出（HSV存在検出＋推論ゲート）の閾値をスライダーで調整し `json/hsv_config_{cam}.json` へ保存 |
| `relay_calibration.py` | リレー（仕分け弁）の開弁タイミングをスライダーで実機に合わせ込む |
| `delay_calibration.py` | カメラ間の撮影位置差を吸収する遅延（`delay_seconds`）を実測し `json/delay_config.json` へ保存 |
| `hsv_ripeness_classifier.py` | 赤色占有率スコアによる healthy/unripe 振り分けGUI（学習データ整備用） |
| `classification_gui_demo.py` | 実機なしで個体確定〜健全/被害判定の流れを確認するデモ |
| `analyze_cycle_logs.py` | 本番が書いた cycle ログから1個体あたりの実コストを集計・比較する（OS非依存）。**本ディレクトリには未移植**。Windows版 [DCRsystem_base/standalone/](../DCRsystem_base/standalone/analyze_cycle_logs.py) にある |


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
