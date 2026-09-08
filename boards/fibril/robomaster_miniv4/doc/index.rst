.. _fibril_robomaster_miniv4:

Fibril RoboMaster Mini V4
#########################

概要
****

RoboMaster Mini の最新リビジョン。
MCU とクロック構成は miniv1 と同じ STM32G474VE だが、モータ CAN が 2 系統になり、故障表示用の赤 LED が増えている。
RS485 バスも USART3 の PB10/PE15 から PC10/PC11 へ移した。

ピン割り当ては先行ファームウェア CanMotorMbed の ``robomaster_miniv4`` ターゲットから移してある。

ハードウェア
************

- STM32G474VE、Flash 512 KB、SRAM 128 KB
- HSE 24 MHz 水晶。PLL を経て SYSCLK 160 MHz
- RGB ステータス LED と、独立した故障表示 LED（いずれもアクティブ Low）
- 4 ビット BCD ロータリ DIP スイッチ
- 外部 CAN 1 系統とモータ CAN 2 系統
- USB デバイスポート
- 独立ウォッチドッグ（IWDG）

クロック構成
============

FDCAN のカーネルクロックには PLL_Q を使い、80 MHz に置いている。
これは FDCAN の上限であると同時に、5 Mbps の FD データフェーズでビットあたりの time quanta が整数になる値でもある（1 Mbps で 80 tq、5 Mbps で 16 tq）。
このカーネルクロックは SoC 上の全 FDCAN インスタンスで共有される。

HSI48 は有効にしてある。

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
     - ``usart1``
     - TX=PA9、RX=PA10、115200 baud
     - 有効
   * - RS485（AMT21x エンコーダ）
     - ``usart3``
     - TX=PC10、RX=PC11、DE=PB14
     - 無効
   * - 外部 CAN
     - ``fdcan1``
     - RX=PD0、TX=PD1（STDBY は PB11）
     - 無効
   * - モータ CAN 0（MOTCAN0）
     - ``fdcan2``
     - RX=PB12、TX=PB13
     - 無効
   * - モータ CAN 1（MOTCAN1）
     - ``fdcan3``
     - RX=PA8、TX=PA15
     - 無効
   * - USB デバイス
     - ``usb`` （``zephyr_udc0``）
     - DM=PA11、DP=PA12
     - 無効
   * - ステータス LED
     - ``led_r`` / ``led_g`` / ``led_b``
     - PB9 / PB5 / PD7（アクティブ Low）
     - 有効
   * - 故障表示 LED
     - ``led_fault``
     - PF2（アクティブ Low）
     - 有効
   * - ロータリ ID スイッチ
     - ``rot1`` / ``rot2`` / ``rot4`` / ``rot8``
     - PC0 / PC1 / PE3 / PE2（プルアップ、アクティブ Low）
     - 有効

``led0`` エイリアスは緑（``led_g``）、``watchdog0`` は ``iwdg`` を指す。

FDCAN と ``usart3``、``usb`` はピンとクロックだけを設定して無効のままにしてある。
使う側の overlay が ``status`` を ``"okay"`` にし、ボーレートやビットレート、DMA、子ノードを与える。

RS485 の設定例は :doc:`/drivers/amt21`、モータ CAN の使い方は :doc:`/drivers/robomaster` を参照する。

ロータリスイッチは ``gpio-keys`` として記述しているが、入力イベント源として使うことは想定していない。
ニブルとして読み、機体ごとの CAN ID やホスト名を導出する用途である。

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

   west build -b fibril_robomaster_miniv4 app
   west flash

``west flash`` の runner は STM32CubeProgrammer、OpenOCD、pyOCD、J-Link に対応する。
