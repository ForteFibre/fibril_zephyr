---
status: Proposed
date: 2026-09-22
---

# 0008. fibril_common を Zephyr module として取り込む

## Status

Proposed

実装はまだ無い。実測に基づく調査の結果と、上流に要る変更を記録する。

## Context

[fibril_common](https://github.com/ForteFibre/fibril_common) は ROS 2 や mbed OS のような環境に依存しない C++ ライブラリで、制御器・軌道生成・シリアライザを提供する。
PC 側の ROS 2 ノードが使っているものと同じ実装を基板の上でも使いたい、というのが取り込みの動機である。
`apps/node` を C++17 に移したのはその前段であった（[ADR 0007](0007-cpp-node-implementation.md)）。

fibril_common は ament パッケージであり、Zephyr module としての入口を持たない。
`CMakeLists.txt` は `add_library(fibril_common SHARED)` と `GLOB_RECURSE` で、Zephyr のビルドシステムからは使えない。

取り込めるかどうかは、ヘッダが Zephyr の C++ 環境（picolibc + libstdc++、例外なし、Cortex-M4F）で通るか、通ったとして何を引きずってくるか次第である。
これを実測で確かめた。

## 調査方法

`apps/node` のビルドから C++ のコンパイルコマンドを取り出し、同じフラグで

1. 全ヘッダを 1 つずつ単独の翻訳単位として構文検査した
2. `src/` の 3 つの `.cpp` を `fibril_rc26_mainair_v01`（STM32G474、単精度 FPU）向けにコンパイルした
3. 使う部品を変えた 4 通りの spike アプリを実際にリンクし、フットプリントとシンボルを比べた

以下の「剥がした版」は、コストの出どころを測るための**乱暴な probe** であって、提案する patch ではない。

## 分かったこと

### 全ヘッダがそのまま通る

22 個のヘッダすべてがエラー 0 でコンパイルできた。
`src/` の 3 ファイルも同じである。

例外については `utils/exception.hpp` が `__cpp_exceptions` の有無を見て `std::abort()` に落ちる作りになっており、ライブラリ内に直接の `throw` / `catch` は 1 つも無い。
Zephyr の既定（例外オフ）でそのまま使える。

警告は `-Wdouble-promotion` だけで、`-Werror` は付いていないのでビルドは止まらない。
ただしこの警告は実害を指している（後述）。

### フットプリントは部品によって桁が違う

`fibril_rc26_mainair_v01` に `CONFIG_CPP` / `CONFIG_STD_CPP17` / `CONFIG_REQUIRES_FULL_LIBCPP` を入れた spike アプリで実測した。

| 構成 | FLASH | RAM | double 軟浮動 | iostream | heap | 静的 ctor |
| --- | --- | --- | --- | --- | --- | --- |
| ベースライン（fibril_common なし） | 26900 B | 9728 B | なし | なし | なし | 0 |
| `PIDController` | 28328 B (+1428) | 9792 B (+64) | なし | なし | なし | 0 |
| `TrapezoidalController` | 46188 B (+17860) | 10176 B (+384) | あり | あり | あり | 6 |
| 同、デバッグ出力面を剥がした probe | 37696 B (−8492) | 9984 B (−192) | あり | なし | あり | 0 |
| 同、さらに float リテラル修正の probe | 37304 B (−392) | 9984 B | 一部 | なし | あり | 0 |
| `PurePursuitTracker` を追加 | 47256 B (+1068) | 10496 B (+320) | あり | あり | あり | 6 |

`PIDController` はヘッダだけで完結し、ヒープも iostream も静的コンストラクタも引かない。
一方でコントローラ族は 1 つ使うだけで 17.9 KB 増える。その内訳が以下である。

### iostream を引きずるのは純粋仮想のデバッグ出力

`uniform_speed_controller.hpp` の `ControllerBase` が

```cpp
virtual void print_debug_info(std::ostream & os) const = 0;
```

を持つ。純粋仮想なので vtable から参照され、コントローラを 1 つでも実体化すると
`<ostream>` の実装がリンカに落とせなくなる。
イメージに入る静的コンストラクタ 6 個も全部これ由来である（剥がした probe では 0 になる）。

**このデバッグ出力面が占めているのは 8492 B（実測）。** 剥がすと静的コンストラクタも消える。

probe は正規表現で宣言と実装を削っただけなので、上流に出すときは形を決め直した。
検討したのは次の 3 つで、(a) を採った。

- (a) 純粋仮想と override を `#ifndef` で囲む — probe と同じ数字になる
- (b) `print_debug_info` を仮想でなくし、出力先をコールバックで受ける
- (c) 出力を持たない基底に分ける

[fibril_common#68](https://github.com/ForteFibre/fibril_common/pull/68) が (a) である。
`FIBRIL_NO_IOSTREAM` を定義したときだけ面が消える形なので、ROS 2 側のビルドは変わらない。
実際の patch での実測も probe と同じ差（FLASH −8492 B、RAM −192 B、静的コンストラクタ 6 → 0）だった。

### `-Wdouble-promotion` は実害を指している

STM32G474 の FPU は単精度しか持たない。`double` の演算はすべてサブルーチン呼び出しに落ちる。

逆アセンブルで呼び出し元を特定したところ、軟浮動の呼び出しは fibril_common 由来だった（ベースラインには 1 つも無い）。

| 呼び出し元 | 原因 |
| --- | --- |
| `UniformAcceleration::position_at(float)` | `0.5 * _acceleration * ...` の `0.5` が `double` リテラル |
| `ForwardTrapezoidalController::calc_intersect_pos` | 戻り値と本体が `double` |
| `ForwardTrapezoidalController::merge_trajectories` | 上を経由 |

`position_at` は軌道を評価するたびに呼ばれる。
リテラルを `0.5F` にするだけで、この関数からは軟浮動の呼び出しが消えた（FLASH も 392 B 減る）。
`calc_intersect_pos` は型そのものが `double` なので、こちらは本体を見て決める変更になる。

`pure_pursuit_tracker` は設計として `double` を使っている（ヘッダと実装で 150 箇所）ので、これは promotion の修正では済まない。

### `CircularBuffer` は Mutex 型を明示しないと使えない

```cpp
template <typename T, size_t Size, typename Mutex FIBRIL_SYSTEM_MUTEX>
class CircularBuffer
```

`FIBRIL_SYSTEM_MUTEX` は `_GLIBCXX_HAS_GTHREADS` があるときだけ `= std::mutex` に展開される。
Zephyr の arm-zephyr-eabi ではこのマクロが**定義されていない**（実測）ので、既定値が消えて 3 つ目の型引数が必須になる。

ライブラリ側が freestanding を想定して書いた分岐なので、欠陥ではない。
BasicLockable を満たす 10 行ほどのアダプタで通る（コンパイルとリンクを確認済み）。

```cpp
class ZephyrMutex
{
public:
  ZephyrMutex() { k_mutex_init(&m_); }
  void lock() { k_mutex_lock(&m_, K_FOREVER); }
  void unlock() { k_mutex_unlock(&m_); }
private:
  struct k_mutex m_;
};
```

置き場は fibril_common 側の `zephyr/include/` が筋である。
Zephyr を知っているグルーを利用側に何度も書かせない。

### `SoftwareWatchdog` の内蔵クロックはリンクできない

`SoftwareWatchdog<TimeSource>` はクロックを型引数で受ける作りなので、`k_uptime_get()` を包んだ型を渡せば動く。
ただし同梱の `SteadyClock` は `std::chrono::steady_clock::now()` を呼び、これは

```
undefined reference to `gettimeofday'
```

でリンクに失敗する。
`CONFIG_POSIX_API` を足せば解決するが、Zephyr のクロックを注入するほうが安い。

### ヒープを使う部品と使わない部品がある

`std::vector` / `std::string` / `std::make_shared` を持つのは
`composite_controller`、`limit_profile`、`trapezoidal_controller`、`pure_pursuit_tracker`、`callback_handle` である。

picolibc の malloc アリーナは既定が `CONFIG_COMMON_LIBC_MALLOC_ARENA_SIZE=-1`、つまり**残りの RAM 全部**である。
これは fibril_can ランタイムが init 後に確保しない方針とも、このリポジトリが `K_HEAP_DEFINE` で枠を切ってきた習慣とも合わない。
ヒープを使う部品を載せるイメージでは、アリーナを明示的に切る。

### 部品ごとの状態

| 部品 | ヒープ | iostream | double | そのまま使えるか |
| --- | --- | --- | --- | --- |
| `controller/pid`、`controller/speed_pid` | — | — | — | 使える |
| `controller/acceleration_limit`、`controller/kinematics` | — | — | — | 使える |
| `data/serde`、`data/serde_checksum` | — | — | — | 使える（`std::function` は型抽出の未評価文脈だけ） |
| `input/button`、`utils/non_copyable`、`utils/functional_unwrap` | — | — | — | 使える |
| `navigation/path_tracking_types` | — | — | — | 使える |
| `safety/watchdog` | — | — | — | クロックを注入すれば使える |
| `data/circular_buffer` | — | — | — | Mutex 型を渡せば使える |
| `utils/callback_handle`、`controller/controller_manager` | あり | — | — | アリーナを切れば使える |
| `controller/limit_profile` | あり | — | — | アリーナを切れば使える |
| `controller/uniform_speed_controller`、`composite_controller`、`reverse_controller`、`trapezoidal_controller` | あり | あり | あり | 上流の変更が要る |
| `navigation/pure_pursuit_tracker` | あり | あり | あり | 上流の変更が要る |

## Decision

まだ決めていない。決めるには「最初にどの部品を載せたいか」が要る。

- **PID や serde から始めるなら、止まるものは何も無い。** module のグルーを足せばその日から使える。
- **軌道生成（`TrapezoidalController`）から始めるなら、iostream の 8492 B は [fibril_common#68](https://github.com/ForteFibre/fibril_common/pull/68) で外せる。** マージされれば、Zephyr 側の Kconfig から `FIBRIL_NO_IOSTREAM` を定義して使う。

いずれにせよ module のグルーは共通で要る。形は fibril_can が先例になる。

```text
fibril_common/zephyr/
├── module.yml        build.cmake: zephyr / build.kconfig: zephyr/Kconfig
├── CMakeLists.txt    CONFIG_FIBRIL_COMMON で return、zephyr_library、3 つの .cpp
├── Kconfig           CONFIG_FIBRIL_COMMON（CPP と REQUIRES_FULL_LIBCPP に依存）
├── COLCON_IGNORE     colcon が zephyr/ を ament パッケージとして拾わないように
└── include/fibril/zephyr/   k_mutex アダプタなどのグルー
```

`COLCON_IGNORE` は飾りではない。fibril_common は ament パッケージなので、これが無いと colcon が `zephyr/` をビルドしようとする。
ルートの `CMakeLists.txt` は触らない。`module.yml` が `build.cmake: zephyr` を指すので ament 側の入口と衝突しない。

`west.yml` にはタグで固定する（[ADR 0007](0007-cpp-node-implementation.md) の repin と同じ方針）。
現在の最新タグは v1.0.19 だが、手元の作業ツリーは `refactor/pid-hardening` にいて、制御器の API がまだ動いている。
取り込むときにどこで切るかは上流の区切りに合わせる。

## 次にやること

| 段 | 内容 |
| --- | --- |
| Phase 0 | fibril_common に `zephyr/` のグルーを足す PR。iostream の扱いは [#68](https://github.com/ForteFibre/fibril_common/pull/68) で先に出してある |
| Phase 1 | `west.yml` にプロジェクトを足し、最初の利用者（`lib/fibril_can_node/<type>/` の 1 つ）で使う |

`-Wdouble-promotion` の修正と `SteadyClock` の扱いは、Zephyr で使うかどうかに関わらず上流に返す価値がある。

## Alternatives considered

### fibril_zephyr 側にグルーを置く

`west.yml` に project として足し、`zephyr_library` をこのリポジトリの `lib/` から `${ZEPHYR_FIBRIL_COMMON_MODULE_DIR}` のソースを拾う形で書けば、上流に手を入れずに取り込める。

却下する。ソースの一覧とビルドの前提をライブラリの外に置くことになり、fibril_common に部品が増えるたびにこちらが追従する。
fibril_can が `zephyr/` を自分で持っているのと同じ理由で、グルーはライブラリ側に置く。

### ヘッダだけコピーして持ち込む

取り込みの動機が「PC 側と同じ実装を使う」ことなので、複製した時点で目的が消える。
