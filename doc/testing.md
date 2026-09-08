# テスト

テストは Twister から走らせる。
`tests/` 以下が ztest、`app/` が実機向けアプリケーションのビルド確認である。

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
```

`app/sample.yaml` は `build_only: true` で、`integration_platforms` に `nucleo_g474re` を指定している。
`app.default` と、`debug.conf` を重ねた `app.debug` の 2 通りをビルドする。

## 構成

| パス | 対象 | プラットフォーム |
| --- | --- | --- |
| `tests/drivers/encoder/accum` | 積算・速度・オフセットの共通コア | `native_sim`、`native_sim/native/64` |
| `tests/drivers/encoder/amt21` | AMT21x ドライバ | `native_sim`、`native_sim/native/64` |
| `tests/drivers/motor/robomaster` | RoboMaster ドライバ | `native_sim`、`native_sim/native/64` |
| `tests/drivers/motor/robomaster_start_retry` | 起動できない CAN バスからの復帰 | `native_sim`、`native_sim/native/64` |
| `tests/lib/custom` | `lib/custom` | 制限なし |

いずれも実機を必要としない。
ハードウェアの振る舞いはテスト側のスタブで模擬しており、`native_sim` 上で完結する。

`tests/drivers/encoder/accum` だけはデバイスを介さず、`drivers/encoder/encoder_accum.c` を直接ビルドして関数を呼ぶ。
STM32 のタイマには `uart_emul` に相当するエミュレータがなく、`drivers/encoder/qdec_stm32.c` は `native_sim` ではコンパイルできない。
直交エンコーダドライバのうちテストできるのは、この共通コアに切り出した部分だけである。
`CONFIG_ENCODER=n` にしてあるのは、有効にすると `drivers/encoder` のライブラリが同じオブジェクトを二重にリンクするためである。

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

`.github/workflows/build.yml` が Ubuntu、macOS、Windows の 3 つで `app` と `tests` の両方を走らせる。
`fibril_can` が private repository のため、CI は GitHub App のトークンを取得して `git config url.insteadOf` を張ってから `west update` を実行する。

`.github/workflows/docs.yml` は Doxygen と Sphinx を Doxygen 1.9.6 と 1.14.0 の両方でビルドし、`main` では GitHub Pages に公開する。
Sphinx は `-W` 付きで走るため、警告はエラーになる。
