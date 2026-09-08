.. _fibril_robomaster_miniv3:

Fibril RoboMaster Mini V3
#########################

概要
****

RoboMaster Mini の LQFP80 版。
MCU は STM32G474ME で、miniv1 や miniv4 と同じダイの系列だがピン数が少ない。
このためピン定義は M パッケージの pinctrl DTSI を取り込んでいる。

バスの配置は miniv4 と同じである（USART1 コンソール、USART3 RS485、FDCAN1 外部 CAN、FDCAN2 が MOTCAN0、FDCAN3 が MOTCAN1、USB）。
miniv4 との違いは RGB LED とロータリ ID スイッチのピンにある。

ピン割り当ては先行ファームウェア CanMotorMbed の ``robomaster_miniv3`` ターゲットから移してある。

ハードウェア
************

- STM32G474ME（LQFP80）、Flash 512 KB、SRAM 128 KB
- HSE 24 MHz 水晶。PLL を経て SYSCLK 160 MHz
- RGB ステータス LED（アクティブ Low）
- 4 ビット BCD ロータリ DIP スイッチ
- 独立ウォッチドッグ（IWDG）

クロック構成
============

FDCAN のカーネルクロックには PLL_Q を使い、80 MHz に置いている。
これは FDCAN の上限であると同時に、5 Mbps の FD データフェーズでビットあたりの time quanta が整数になる値でもある（1 Mbps で 80 tq、5 Mbps で 16 tq）。

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
   * - 外部 CAN
     - ``fdcan1``
     - RX=PD0、TX=PD1（STDBY は PB11）
     - 無効
   * - ステータス LED
     - ``led_r`` / ``led_g`` / ``led_b``
     - PB4 / PD2 / PB3（アクティブ Low）
     - 有効
   * - ロータリ ID スイッチ
     - ``rot1`` / ``rot2`` / ``rot4`` / ``rot8``
     - PC0 / PC1 / PC14 / PC13（プルアップ、アクティブ Low）
     - 有効

``led0`` エイリアスは緑（``led_g``）、``watchdog0`` は ``iwdg`` を指す。

ROT4（PC14）と ROT8（PC13）は LSE 発振器の入力および SYS_WKUP2 と重なる。
どちらも有効にしていないので、これらのピンは GPIO として使える。

ロータリスイッチは ``gpio-keys`` として記述しているが、入力イベント源として使うことは想定していない。
ニブルとして読み、機体ごとの CAN ID やホスト名を導出する用途である。

devicetree に書かれていないペリフェラル
=======================================

このボードの devicetree では、次のノードがコメントアウトされたまま残っている。
基板上には配線があるので、使うときはコメントを外すか overlay で同じ内容を与える。

.. list-table::
   :header-rows: 1

   * - 機能
     - ノード
     - ピン
   * - RS485（AMT21x エンコーダ）
     - ``usart3``
     - TX=PC10、RX=PC11、DE=PB14
   * - モータ CAN 0（MOTCAN0）
     - ``fdcan2``
     - RX=PB12、TX=PB13
   * - モータ CAN 1（MOTCAN1）
     - ``fdcan3``
     - RX=PA8、TX=PA15
   * - USB デバイス
     - ``usb``
     - DM=PA11、DP=PA12

RS485 の設定例は :doc:`/drivers/amt21`、モータ CAN の使い方は :doc:`/drivers/robomaster` を参照する。

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

   west build -b fibril_robomaster_miniv3 app
   west flash

``west flash`` の runner は STM32CubeProgrammer、OpenOCD、pyOCD、J-Link に対応する。
