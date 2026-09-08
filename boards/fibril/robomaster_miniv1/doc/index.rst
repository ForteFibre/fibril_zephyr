.. _fibril_robomaster_miniv1:

Fibril RoboMaster Mini V1
#########################

概要
****

RoboMaster モータを CAN で駆動するための小型基板。
MCU は STM32G474VE（LQFP100）で、外部 CAN 1 系統と AMT21x 用の RS485 バスを持つ。

ピン割り当ては先行ファームウェア CanMotorMbed の ``robomaster_miniv1`` ターゲットから移してある。
同じハードウェアを配線変更なしで動かせる。

ハードウェア
************

- STM32G474VE、Flash 512 KB、SRAM 128 KB
- HSE 24 MHz 水晶。PLL を経て SYSCLK 160 MHz
- RGB ステータス LED（アクティブ Low）
- 独立ウォッチドッグ（IWDG）

クロック構成
============

FDCAN のカーネルクロックには PLL_Q を使い、80 MHz に置いている。
これは FDCAN の上限であると同時に、5 Mbps の FD データフェーズでビットあたりの time quanta が整数になる値でもある（1 Mbps で 80 tq、5 Mbps で 16 tq）。
HSE の 24 MHz を直接与えると 5 Mbps で 4.8 tq/bit となり表現できない。

HSI48 は有効にしてある。
USB など 48 MHz を要求するペリフェラルが CLK48_SEL 経由で参照できる。

対応機能
********

Twister が認識する機能は ``gpio``、``uart``、``dma``、``can`` である。

配線とピン
**********

.. list-table::
   :header-rows: 1

   * - 機能
     - ノード
     - ピン
     - 既定の状態
   * - コンソール / シェル
     - ``usart2``
     - TX=PD5、RX=PD6、115200 baud
     - 有効
   * - RS485（AMT21x エンコーダ）
     - ``usart3``
     - TX=PB10、RX=PE15、DE=PB14
     - 無効
   * - 外部 CAN
     - ``fdcan1``
     - RX=PD0、TX=PD1
     - 無効
   * - ステータス LED
     - ``led_r`` / ``led_g`` / ``led_b``
     - PC10 / PC11 / PC12（アクティブ Low）
     - 有効

``led0`` エイリアスは緑（``led_g``）、``watchdog0`` は ``iwdg`` を指す。

``usart3`` と ``fdcan1`` はピンとクロックだけを設定して無効のままにしてある。
使う側の overlay が ``status`` を ``"okay"`` にし、ボーレートやビットレート、DMA、子ノードを与える。
RS485 の設定例は :doc:`/drivers/amt21` を参照する。

USB はこのボードの devicetree では設定していない。
``samples/lib/fibril_can/hub_gs_usb_self`` の overlay が PA11/PA12 に ``zephyr_udc0`` を生やす例になっている。

フラッシュのパーティション
==========================

.. list-table::
   :header-rows: 1

   * - ラベル
     - オフセット
     - サイズ
   * - ``mcuboot``
     - 0x00000000
     - 48 KB（読み出し専用）
   * - ``image-0``
     - 0x0000c000
     - 228 KB
   * - ``image-1``
     - 0x00045000
     - 228 KB
   * - ``storage``
     - 0x0007e000
     - 8 KB

ビルドと書き込み
****************

.. code-block:: shell

   west build -b fibril_robomaster_miniv1 app
   west flash

``west flash`` の runner は STM32CubeProgrammer、OpenOCD、pyOCD、J-Link に対応する。
既定以外を使うときは ``west flash -r jlink`` のように指定する。
