.. _fibril_rc26_mainair_v01:

Fibril RC26 MainAir V01
#######################

概要
****

RC26 機体のメイン基板。
CAN FD 2 系統、UART 5 系統、空圧バルブ出力 6 系統を持ち、ジャイロとエンコーダをソケットで受ける。

ピン割り当ては KiCad の回路図 ``RC26_MainAir_v01.kicad_sch``（``ForteFibre_PCB/MainBoard``）から起こしてある。
他のボードと違い、先行ファームウェアの移植ではない。

ハードウェア
************

- STM32G474RE（LQFP64）、Flash 512 KB、SRAM 128 KB
- HSE 24 MHz 水晶。PLL を経て SYSCLK 160 MHz
- CAN FD トランシーバ 2 個
- UART ブレイクアウト 5 系統
- USB-C デバイスポート
- 直交エンコーダソケット 2 個
- I2C ジャイロソケット
- TBD62783A ソースドライバ経由の +12 V 空圧バルブ出力 6 系統
- RGB ステータス LED とユーザー LED（いずれもアクティブ High）
- 独立ウォッチドッグ（IWDG）

クロック構成
============

FDCAN のカーネルクロックには PLL_Q を使い、80 MHz に置いている。
これは FDCAN の上限であると同時に、5 Mbps の FD データフェーズでビットあたりの time quanta が整数になる値でもある（1 Mbps で 80 tq、5 Mbps で 16 tq）。

HSI48 は有効にしてある。

対応機能
********

Twister が認識する機能は ``gpio``、``uart``、``dma``、``can``、``i2c``、``usb_device`` である。

CAN
***

基板のシルク表記と FDCAN インスタンスの対応が入れ替わっている点に注意する。

.. list-table::
   :header-rows: 1

   * - 基板の表記
     - ノード
     - エイリアス
     - ピン
     - トランシーバの STB
   * - CAN0
     - ``fdcan3``
     - ``can0``
     - RX=PA8、TX=PA15
     - PA9（Low で動作、High でスタンバイ）
   * - CAN1
     - ``fdcan2``
     - ``can1``
     - RX=PB12、TX=PB13
     - PA10（Low で動作、High でスタンバイ）

``fdcan1`` にはトランシーバが載っていない。
devicetree でも無効のままにしてある。

いずれの FDCAN もピンとクロックだけを設定して無効にしてあるので、使う側の overlay が ``status`` を ``"okay"`` にしてビットレートを与える。

UART
****

.. list-table::
   :header-rows: 1

   * - ノード
     - ピン
     - 引き出し先
     - 既定の状態
   * - ``usart1``
     - TX=PC4、RX=PC5
     - J14（PH コネクタ）
     - 無効
   * - ``usart2``
     - TX=PA2、RX=PA3
     - J16（PH コネクタ）
     - 無効
   * - ``usart3``
     - TX=PB10、RX=PB11
     - SWD/デバッグヘッダ J1（4/5 ピン）と J13
     - 有効（コンソール / シェル、115200 baud）
   * - ``uart4``
     - TX=PC10、RX=PC11
     - J17（PH コネクタ）
     - 無効
   * - ``uart5``
     - TX=PC12、RX=PD2
     - J15（PH コネクタ）と J25 ジャイロソケット（3/4 ピン）
     - 無効

``usart3`` は J1 と J13 が同じ TX/RX の配線を共有するため、同時に両方をつなぐことはできない。
``uart5`` も J15 と J25 で同じ制約がある。

そのほかの配線
**************

.. list-table::
   :header-rows: 1

   * - 機能
     - ノード
     - ピン
     - 既定の状態
   * - I2C（ジャイロソケット J26）
     - ``i2c4``
     - SCL=PC6、SDA=PC7、標準速度
     - 無効
   * - USB デバイス
     - ``usb`` （``zephyr_udc0``）
     - DM=PA11、DP=PA12
     - 有効
   * - ステータス LED
     - ``led_r`` / ``led_g`` / ``led_b``
     - PC13 / PC15 / PC14（アクティブ High）
     - 有効
   * - ユーザー LED
     - ``led_user``
     - PC1（アクティブ High）
     - 有効
   * - ユーザーボタン（BootSW）
     - ``user_button``
     - PB8（プルアップ、アクティブ Low）
     - 有効
   * - 空圧バルブ出力
     - ``air0`` … ``air5``
     - PA1 / PA6 / PA0 / PA5 / PA7 / PA4（アクティブ High）
     - 有効

``led0`` エイリアスはユーザー LED（``led_user``）、``sw0`` は ``user_button``、``watchdog0`` は ``iwdg`` を指す。

I2C4 のプルアップは基板上に 4.7 kΩ が載っている。

空圧バルブ出力は ``gpio-leds`` として記述している。
GPIO を名前で引くのに都合がよいためで、LED ではない。

devicetree に書かれていないペリフェラル
=======================================

基板上には配線があるが、devicetree ではノードを持たないものがある。

直交エンコーダ入力は、J11（Encoder3）が PB4/PB5 の TIM3_CH1/CH2、J12（Encoder4）が PB6/PB7 の TIM4_CH1/CH2 につながっている。
使うときは overlay でタイマーを有効にし、pinctrl を持つ ``st,stm32-qdec`` の子ノードを付ける。
タイマーノード自身は ``pinctrl-0`` を取らない。

アドレサブル RGB LED（D10、WS281x 系）は PB9 につながっている。
PB9 は TIM17_CH1 と SPI2_NSS のどちらでも駆動できるため、devicetree ではピンを空けたままにして overlay に選ばせる。

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

   west build -b fibril_rc26_mainair_v01 app
   west flash

``west flash`` の runner は STM32CubeProgrammer、OpenOCD、pyOCD、J-Link に対応する。
