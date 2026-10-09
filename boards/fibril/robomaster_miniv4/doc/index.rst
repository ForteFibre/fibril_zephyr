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
   * - エンコーダ入力 ENC0
     - ``timers3`` CH1/CH2
     - A=PA6、B=PA7
     - 未定義
   * - エンコーダ入力 ENC1
     - ``timers5`` CH1/CH2
     - A=PB2、B=PC12
     - 未定義
   * - エンコーダ入力 ENC2
     - ``timers4`` CH1/CH2
     - A=PD12、B=PD13
     - 未定義
   * - エンコーダ入力 ENC3
     - ``timers2`` CH1/CH2（``timers5`` も可）
     - A=PA0、B=PA1
     - 未定義
   * - エンコーダ入力 ENC4
     - ``timers1`` CH1/CH2
     - A=PE9、B=PE11
     - 未定義
   * - エンコーダ入力 ENC5
     - ``timers8`` CH1/CH2（``timers3`` も可）
     - A=PC6、B=PC7
     - 未定義
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
     - ``rotary_id`` の ROT1 / ROT2 / ROT4 / ROT8
     - PC0 / PC1 / PE3 / PE2（プルアップ、アクティブ Low）
     - 有効

``led0`` エイリアスは緑（``led_g``）、``watchdog0`` は ``iwdg`` を指す。

FDCAN と ``usart3``、``usb`` はピンとクロックだけを設定して無効のままにしてある。
使う側の overlay が ``status`` を ``"okay"`` にし、ボーレートやビットレート、DMA、子ノードを与える。

RS485 の設定例は :doc:`/drivers/amt21`、モータ CAN の使い方は :doc:`/drivers/robomaster` を参照する。

エンコーダ入力のピン名は CanMotorMbed の ``PinNames.h`` に合わせてある。
ボードの devicetree はこれらのノードを持たず、使う側の overlay が timer の子に ``fibril,stm32-qdec`` のノードを置く（:doc:`/drivers/qdec_stm32`）。
ENC0 と ENC5、ENC1 と ENC3 は、それぞれ同じ timer（``timers3``、``timers5``）にも載るので、両方を使うときは表の割り当てにする。
``miniv4-md-qdec4`` snippet は ENC0〜ENC3 を使う。

ロータリスイッチは ``fibril,id-switch``（``rotary_id``）として記述し、``chosen`` の ``fibril,node-id`` がこれを指す。
ROT1 を最下位ビットとする 4 ビットの値として読み、fibril_can の node_id にそのまま使う（``lib/node_id``、:doc:`/apps` の「node_id を ID スイッチから読む」）。

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
