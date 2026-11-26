# テスト

テストは Twister から走らせる。
`tests/` 以下が ztest、`apps/` と `app/` がアプリケーションのビルド確認である。

## 実行

```shell
west twister -T tests --integration
```

`--integration` は各テストの `integration_platforms` に挙がったプラットフォームだけを対象にする。
これを外すと、そのテストが許可する全プラットフォームでビルドを試みる。

一部だけ走らせたいときはパスを絞る。

```shell
west twister -T tests/drivers/motor --integration
```

アプリケーションのビルド確認は別に走る。

```shell
west twister -T app --integration
west twister -T apps --integration
```

`app/sample.yaml` は `build_only: true` で、`integration_platforms` に `nucleo_g474re` を指定している。
`app.default` と、`debug.conf` を重ねた `app.debug` の 2 通りをビルドする。

`apps/node/sample.yaml` も `build_only: true` である。
焼く単位ごとに 1 シナリオで、トランスポート snippet とデプロイ snippet を重ねた組み合わせを、そのボードでビルドする。

| シナリオ | ボード | snippet |
| --- | --- | --- |
| `app.node.rc26_air` | `fibril_rc26_mainair_v01` | `rc26-mainair-usb`、`rc26-air` |
| `app.node.rc26_robstride` | `fibril_rc26_mainair_v01` | `rc26-mainair-usb`、`rc26-robstride` |
| `app.node.rc26_odrive` | `fibril_rc26_mainair_v01` | `rc26-mainair-usb`、`rc26-odrive` |
| `app.node.rc26_main` | `fibril_rc26_mainair_v01` | `rc26-mainair-usb`、`rc26-main` |
| `app.node.rc26_main_odom` | `fibril_rc26_mainair_v01` | `rc26-mainair-usb`、`rc26-main`、`rc26-odom` |
| `app.node.miniv4_md_qdec4` | `fibril_robomaster_miniv4` | `miniv4-can`、`miniv4-md-qdec4` |
| `app.node.miniv4_usb_md_qdec4` | `fibril_robomaster_miniv4` | `miniv4-usb`、`miniv4-md-qdec4` |
| `app.node.miniv4_md_rotor8` | `fibril_robomaster_miniv4` | `miniv4-can`、`miniv4-md-rotor8` |
| `app.node.miniv4_md_amt4` | `fibril_robomaster_miniv4` | `miniv4-can`、`miniv4-md-amt4` |
| `app.node.miniv4_md_rotor8_adc` | `fibril_robomaster_miniv4` | `miniv4-can`、`miniv4-md-rotor8`、`miniv4-adc` |
| `app.node.miniv4_md_qdec4_adc` | `fibril_robomaster_miniv4` | `miniv4-can`、`miniv4-md-qdec4`、`miniv4-adc` |

ボードはシナリオごとに `platform_allow` で書く。
twister は `common` のリストをシナリオのリストに連結するので、`common` にボードを並べると、別のボードの snippet でもビルドを試みてしまう。

`apps/` のビルドは `fcan_codegen` CLI を要求する。
`ros-jazzy-fibril-can-codegen` を入れた環境では自動で見つかる。
見つからなければ、`modules/lib/fibril_can` から host 向けにビルドする（シナリオごとのビルドディレクトリの `fcan_codegen_host/`。protobuf と yaml-cpp が要る）。
CI（Linux と macOS）はこの経路でビルドする。Windows は protobuf と yaml-cpp を入れる手順が無いので、`apps/` を飛ばす。
手元の CLI や、木からのビルドを指定するときは環境変数で渡す。
twister の `--extra-args=-DFCAN_CODEGEN=...` はヘルパに届かないので効かない。

```shell
west twister -T apps --integration                                         # 見つからなければ木からビルドする
FCAN_CODEGEN=/abs/path/fcan_codegen west twister -T apps --integration     # 手元の CLI を使う
FCAN_CODEGEN_FROM_SOURCE=ON west twister -T apps --integration             # deb があっても木からビルドする
```

## 構成

| パス | 対象 | プラットフォーム |
| --- | --- | --- |
| `tests/drivers/encoder/accum` | 積算・速度・オフセットの共通コア | `native_sim`、`native_sim/native/64` |
| `tests/drivers/encoder/amt21` | AMT21x ドライバ | `native_sim`、`native_sim/native/64` |
| `tests/drivers/encoder/as5047p` | AS5047P ドライバ（SPI のエミュレータ） | `native_sim`、`native_sim/native/64` |
| `tests/drivers/encoder/robomaster` | RoboMaster のロータ角 | `native_sim`、`native_sim/native/64` |
| `tests/drivers/imu/r6093u` | R6093U ドライバ | `native_sim`、`native_sim/native/64` |
| `tests/drivers/motor/robomaster` | RoboMaster ドライバ | `native_sim`、`native_sim/native/64` |
| `tests/drivers/motor/robomaster_start_retry` | 起動できない CAN バスからの復帰（RoboMaster） | `native_sim`、`native_sim/native/64` |
| `tests/drivers/motor/robstride` | RobStride ドライバ（読み戻しあり、`CONFIG_MOTOR_ROBSTRIDE_VERIFY=n` の 2 通り） | `native_sim`、`native_sim/native/64` |
| `tests/drivers/motor/robstride_start_retry` | 起動できない CAN バスからの復帰（RobStride） | `native_sim`、`native_sim/native/64` |
| `tests/drivers/motor/odrive` | ODrive ドライバ | `native_sim`、`native_sim/native/64` |
| `tests/lib/motor_control` | 制御のコア（モード、PID の入力の単位、FF と摩擦、失速ブレーカ） | `native_sim`、`native_sim/native/64` |
| `tests/lib/odrive_sequencer` | ODrive の軸の状態機械（キャリブレーション、arm、IDLE を経由するモードの切り替え、エラーからの復帰、途絶） | `native_sim`、`native_sim/native/64` |
| `tests/lib/custom` | `lib/custom` | 制限なし |

いずれも実機を必要としない。
ハードウェアの振る舞いはテスト側のスタブで模擬しており、`native_sim` 上で完結する。`tests/lib/custom` だけは `qemu_cortex_m3` で走る。

`tests/drivers/encoder/accum` だけはデバイスを介さず、`drivers/encoder/encoder_accum.c` を直接ビルドして関数を呼ぶ。
STM32 のタイマには `uart_emul` に相当するエミュレータがなく、`drivers/encoder/qdec_stm32.c` は `native_sim` ではコンパイルできない。
直交エンコーダドライバのうちテストできるのは、この共通コアに切り出した部分だけである。
`CONFIG_ENCODER=n` にしてあるのは、有効にすると `drivers/encoder` のライブラリが同じオブジェクトを二重にリンクするためである。

### 注意

- **native_sim の tick は既定で 100 Hz である。** `k_sleep()` は次の tick に切り上げるので、数十 ms の閾値を試すテストでは待ち時間が黙って延びる（`k_sleep(K_MSEC(10))` がほぼ 20 ms になる）。そうしたテストは、`prj.conf` で `CONFIG_SYS_CLOCK_TICKS_PER_SEC` を上げるか（`tests/drivers/encoder/amt21`、`tests/drivers/imu/r6093u`）、`k_uptime_get()` で経過時間を測って待つ。
- **native_sim は Linux でしか動かない。** macOS と Windows の twister は、native_sim を使う構成を「Native platform requires Linux」で外す。ドライバと `lib/motor_control`、`lib/odrive_sequencer` のテストは Linux で確かめる。`qemu_cortex_m3` を使う `tests/lib/custom` は、この制限を受けない。
- **`tests/lib/motor_control` の `native_sim/native/64` は、ホストの g++ 15 ではビルドできない。** Zephyr の C++ の `BUILD_ASSERT` が `declaration does not declare anything` で拒まれる。手元の twister でこれだけが落ちるのは、変更のせいではない。

## Kconfig の組み合わせ

1 つのテストディレクトリから複数の構成をビルドできる。
`testcase.yaml` の `extra_configs` がその指定である。

AMT21x のテストは 3 通りに分かれている。

| テスト名 | 構成 |
| --- | --- |
| `drivers.encoder.amt21` | 集計カウンタのみ（`CONFIG_ENCODER_AMT21_STATS=n`） |
| `drivers.encoder.amt21.stats` | 原因別カウンタとエラーログを有効 |
| `drivers.encoder.amt21.shell` | 上記に加えてシェルコマンドを有効 |

診断機能は Kconfig で切れる設計なので、有効な場合と無効な場合の両方をビルドして両方の経路を確かめる。

## CI

`.github/workflows/build.yml` が Ubuntu、macOS、Windows の 3 つで `app` と `tests` の両方を走らせ、Ubuntu と macOS では `apps` もビルドする。
`tests/` のうち native_sim を使う構成は、実際に走るのは Ubuntu だけである（上の「注意」）。`qemu_cortex_m3` を使う `tests/lib/custom` は OS で外していない。
`fibril_can` が private repository のため、CI は GitHub App のトークンを取得して `git config url.insteadOf` を張ってから `west update` を実行する。

`.github/workflows/docs.yml` は Doxygen と Sphinx を Doxygen 1.9.6 と 1.14.0 の両方でビルドし、`main` では GitHub Pages に公開する。
Sphinx は `-W` 付きで走るため、警告はエラーになる。
公開ヘッダのコメントに `@kconfig{}` を使わない。Zephyr 本体の Doxyfile が定義する alias で、`doc/Doxyfile` には無いので、Doxygen が未知のコマンドを警告し、`WARN_AS_ERROR` で止まる。
