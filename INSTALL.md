# 開発環境セットアップ

`fibril_zephyr` は Zephyr RTOS の [T2 topology][west_t2] に沿った workspace application です。
このリポジトリ自身が west manifest を提供し、Zephyr 本体と関連モジュール（`cannectivity`、`fibril_can`）を取り込みます。

対応ボード（`boards/fibril/` 配下）はすべて STM32 系です。

- `canmotor_tourobo2023`
- `robomaster_miniv1`
- `robomaster_miniv3`
- `robomaster_miniv4`
- `robomaster_v2`

本ドキュメントでは、Ubuntu / macOS / Windows 上でゼロから開発環境を構築し、ビルド・書き込み・テスト・ドキュメント生成まで一通り実行できる状態にする手順を示します。

Zephyr 側のバージョンは `west.yml` で **v4.4.1** に固定されています。以降の手順・コマンドはこのバージョンの公式 [Getting Started Guide][zephyr_gsg] に準拠しています。

---

## 1. 前提条件

| 項目 | 必要条件 |
| --- | --- |
| OS | Ubuntu 22.04 以降 / macOS 12 以降 / Windows 10 以降 |
| Python | 3.10 以上 |
| CMake | 3.20.0 以上 |
| Git | 2.25 以上 |
| ディスク空き容量 | 15 GB 以上（Zephyr SDK と workspace を含む） |

Windows は PowerShell を前提とします。コマンドプロンプト (`cmd.exe`) を使う場合は仮想環境の有効化コマンドが異なる点にのみ注意してください（後述）。

---

## 2. 依存パッケージのインストール

### 2.1 Ubuntu

パッケージインデックスを更新し、Zephyr 公式が要求するパッケージ一式を導入します。

```bash
sudo apt update
sudo apt upgrade
sudo apt install --no-install-recommends \
  git cmake ninja-build gperf ccache dfu-util device-tree-compiler wget \
  python3-dev python3-pip python3-setuptools python3-tk python3-venv python3-wheel \
  xz-utils file make gcc gcc-multilib g++-multilib libsdl2-dev libmagic1
```

`cmake --version` が 3.20 未満の場合は、Kitware の apt リポジトリを追加して更新してください。手順は Zephyr 公式の [Install Linux Host Dependencies][zephyr_linux] を参照します。

書き込みに ST-Link を利用する場合は次も導入します。`stlink-tools` / `openocd` は導入時に udev ルール（`/lib/udev/rules.d/` 配下）を自動配置するため、非 root ユーザーでの書き込みに必要な権限がそのまま得られます。

```bash
sudo apt install openocd stlink-tools
sudo udevadm control --reload
sudo udevadm trigger
```

ボード固有のプローブや USB-UART アダプタで権限エラーが出る場合は、対象ボードの Zephyr 公式ボードドキュメント（`zephyr/boards/.../doc/index.rst`）の udev 節に従い、個別にルールを追加してください。

### 2.2 macOS

Homebrew が未導入なら先に入れます。

```bash
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
```

Zephyr の依存パッケージを導入します。

```bash
brew install cmake ninja gperf python@3.12 python-tk@3.12 ccache qemu dtc libmagic wget openocd
```

Apple Silicon (arm64) と Intel (x86_64) の双方で動作します。Xcode Command Line Tools が未導入の場合は `xcode-select --install` を先に実行してください。

### 2.3 Windows

管理者権限の PowerShell から `winget` で必要ツールを一括導入します。

```powershell
winget install Kitware.CMake Ninja-build.Ninja oss-winget.gperf Python.Python.3.12 Git.Git oss-winget.dtc wget 7zip.7zip
```

書き込みに ST-Link を使う場合は STMicroelectronics 純正の [STM32CubeProgrammer][cubeprog] を導入してください（`west flash --runner stm32cubeprogrammer` から利用します）。OpenOCD を使う場合は別途 `openocd` を入れてください。

長いパスを扱うため、以下の設定を有効にしておきます。

```powershell
git config --global core.longpaths true
```

---

## 3. Python 仮想環境と west のインストール

Zephyr は Python 仮想環境の利用を推奨しています。以降のコマンドは常に有効化された venv 内で実行してください。

### Ubuntu

```bash
python3 -m venv ~/fibril_zephyr_ws/.venv
source ~/fibril_zephyr_ws/.venv/bin/activate
pip install --upgrade pip
pip install west
```

### macOS

```bash
python3.12 -m venv ~/fibril_zephyr_ws/.venv
source ~/fibril_zephyr_ws/.venv/bin/activate
pip install --upgrade pip
pip install west
```

### Windows (PowerShell)

```powershell
cd $Env:USERPROFILE
python -m venv fibril_zephyr_ws\.venv
fibril_zephyr_ws\.venv\Scripts\Activate.ps1
pip install --upgrade pip
pip install west
```

`cmd.exe` の場合は `fibril_zephyr_ws\.venv\Scripts\activate.bat` を実行します。

---

## 4. workspace の初期化

`fibril_zephyr` を manifest リポジトリとして workspace を作成します。

```bash
west init -m https://github.com/ForteFibre/fibril_zephyr --mr main fibril_zephyr_ws
cd fibril_zephyr_ws
west update
```

既に `fibril_zephyr` を手動で clone してある場合は、その親ディレクトリで local 初期化を使います。

```bash
# ディレクトリ構造: <ws>/fibril_zephyr が clone 済み
cd <ws>
west init -l fibril_zephyr
west update
```

`west update` は `west.yml` に従い、Zephyr（v4.4.1）、`cannectivity`（v1.4.0）、`fibril_can`、および Zephyr が要求するモジュール（`hal_stm32`、`cmsis_6`、`cmsis-dsp`）を取得します。所要時間はネットワーク環境に依存しますが数分〜十数分です。

初期化後のディレクトリ構成は以下のようになります。

```text
fibril_zephyr_ws/
├── .venv/           # Python 仮想環境
├── .west/           # west の設定
├── fibril_zephyr/   # 本リポジトリ（manifest）
├── zephyr/          # Zephyr 本体（v4.4.1）
└── modules/         # 依存モジュール群
```

---

## 5. Zephyr の Python 依存インストール

workspace 直下（venv 有効化状態）で以下を実行します。

```bash
west packages pip --install
```

これは v4.4 で推奨される方法で、Zephyr 側で要求される Python パッケージ群をまとめてインストールします。旧手順の `pip install -r zephyr/scripts/requirements.txt` でも同等の結果になります。

---

## 6. Zephyr SDK のインストール

対応ボードはすべて Cortex-M（STM32）なので、`arm-zephyr-eabi` ツールチェーンのみを導入します。ディスク使用量とダウンロード時間を節約できます。

```bash
cd zephyr
west sdk install -t arm-zephyr-eabi
cd ..
```

全ツールチェーンを入れたい場合は `-t` を省略します。

```bash
west sdk install
```

導入済み SDK の確認は次のコマンドです。

```bash
west sdk list
```

SDK のバージョンは `zephyr/SDK_VERSION` に対応するものが選択されます。必要なバージョンは `cat zephyr/SDK_VERSION` で確認できます。

---

## 7. ビルド動作確認

代表ボード `robomaster_miniv4` で application をビルドします。

```bash
west build -b robomaster_miniv4 fibril_zephyr/app
```

デバッグ用の設定（AMT21 エンコーダの統計とエラーログを有効化するなど）を重ねる場合は次のとおりです。

```bash
west build -b robomaster_miniv4 fibril_zephyr/app -- -DEXTRA_CONF_FILE=debug.conf
```

別ボードに切り替える場合は `build/` を pristine build で再生成してください。

```bash
west build -b robomaster_miniv3 fibril_zephyr/app -p always
```

---

## 8. 書き込み

デバッガ（ST-Link 等）を接続した状態で以下を実行します。

```bash
west flash
```

ボードごとに使用する runner は `boards/fibril/<board>/board.cmake` で定義されています。標準の runner を上書きしたい場合は `--runner` で指定します。

```bash
west flash --runner openocd
west flash --runner stm32cubeprogrammer
```

Windows で ST-Link を使う場合は STM32CubeProgrammer 経由が最も安定します。

---

## 9. Twister によるテスト

`tests/` 配下の integration テストを実行します。

```bash
west twister -T fibril_zephyr/tests --integration
```

特定のプラットフォームのみに絞る場合は `-p` を利用します。

```bash
west twister -T fibril_zephyr/tests -p robomaster_miniv4
```

---

## 10. ドキュメントビルド（任意）

Doxygen と Sphinx でドキュメントを生成できます。

```bash
cd fibril_zephyr/doc
pip install -r requirements.txt
doxygen
make html
```

出力は `_build_doxygen/` および `_build_sphinx/` に生成されます。Doxygen の未導入時は事前に `apt install doxygen` / `brew install doxygen` / `winget install doxygen` で入れてください。

---

## 11. VS Code / clangd 設定（任意）

### 11.1 Zephyr CMake package の登録

`find_package(Zephyr)` で本 workspace の Zephyr を解決させるため、CMake user package registry に登録します。

```bash
west zephyr-export
```

これにより、workspace 外の外部プロジェクトからも本 Zephyr を参照可能になります。

### 11.2 compile_commands.json の生成

`west build` は既定で `build/compile_commands.json` を生成します。clangd から参照させるため、workspace ルートにシンボリックリンクを張っておくと便利です。

```bash
ln -sf build/compile_commands.json compile_commands.json
```

Windows の場合は代わりに `.clangd` に `CompileFlags: { CompilationDatabase: build }` を書いておく方法も使えます。

### 11.3 推奨拡張

- `ms-vscode.cpptools` または `llvm-vs-code-extensions.vscode-clangd`（clangd を推奨）
- `ms-vscode.cmake-tools`
- `nordic-semiconductor.nrf-devicetree`（DTS のシンタックスハイライト）
- `nordic-semiconductor.nrf-kconfig`（Kconfig のシンタックスハイライト）

### 11.4 ワークスペース例

`.vscode/settings.json` の最小例です。

```json
{
  "clangd.arguments": [
    "--compile-commands-dir=build",
    "--query-driver=**/arm-zephyr-eabi-*"
  ],
  "cmake.configureOnOpen": false,
  "files.associations": {
    "*.overlay": "dts",
    "*.dtsi": "dts"
  }
}
```

`clangd.arguments` の `--query-driver` は、Zephyr SDK の GCC を clangd が system include の解決に使えるようにするための指定です。

---

## 12. トラブルシューティング

| 症状 | 対処 |
| --- | --- |
| `west: command not found` | venv が有効化されていません。`source .venv/bin/activate`（Windows は `Activate.ps1`）を実行してください。 |
| `west update` が途中で失敗する | GitHub の一時的な障害やネットワーク切断が原因のことが多いです。`west update` を再実行すれば差分のみ取り直します。 |
| `Zephyr SDK ... not found` | `west sdk install -t arm-zephyr-eabi` を実行してください。導入済みでも認識されない場合は `west sdk list` で場所を確認し、`ZEPHYR_SDK_INSTALL_DIR` を設定します。 |
| `cmake: version 3.x is too old` | Kitware apt リポジトリまたは `pip install cmake` で 3.20 以上に更新してください。 |
| `west flash` で書き込みできない（Linux） | 多くは USB デバイスへのアクセス権不足です。`stlink-tools` / `openocd` を apt から導入すれば付属の udev ルールが自動配置されます。導入後は `sudo udevadm control --reload && sudo udevadm trigger` を実行し、デバッガを挿し直してください。 |
| ボードを切り替えたら古い設定が残る | `west build -p always -b <board> fibril_zephyr/app` で pristine build を実行してください。 |

---

## 参考リンク

- [Zephyr Getting Started Guide (v4.4)][zephyr_gsg]
- [Zephyr Installation on Linux][zephyr_linux]
- [West Workspaces (T2 topology)][west_t2]
- [Zephyr CMake Package][zephyr_cmake]

[zephyr_gsg]: https://docs.zephyrproject.org/4.4.0/develop/getting_started/index.html
[zephyr_linux]: https://docs.zephyrproject.org/4.4.0/develop/getting_started/installation_linux.html
[west_t2]: https://docs.zephyrproject.org/4.4.0/develop/west/workspaces.html#west-t2
[zephyr_cmake]: https://docs.zephyrproject.org/4.4.0/build/zephyr_cmake_package.html
[cubeprog]: https://www.st.com/en/development-tools/stm32cubeprog.html
