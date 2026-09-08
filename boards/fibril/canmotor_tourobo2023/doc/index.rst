.. _fibril_canmotor_tourobo2023:

Fibril CanMotor Tourobo 2023
############################

概要
****

Tourobo 2023 向けのモータ制御基板。
このリポジトリで唯一 STM32F4 系を積んでおり、CAN FD は SoC 内蔵ではなく SPI 接続の MCP2517FD が担う。
アナログ入力用に MCP3208、モータ駆動用に 8 チャネルの PWM を持つ。

ピン割り当てとクロック構成は先行ファームウェア CanMotorMbed の ``canmotor_tourobo2023`` ターゲットから移してある。
同じハードウェアを配線変更なしで動かせる。

ハードウェア
************

- STM32F407VG、Flash 1 MB、SRAM 128 KB（別途 CCM 64 KB を ``zephyr,dtcm`` として使用）
- HSE 25 MHz 水晶。PLL を経て SYSCLK 168 MHz（AHB 1、APB1 4、APB2 2 分周）
- MCP2517FD CAN FD コントローラ（SPI2 接続）
- MCP3208 12 ビット 8 チャネル ADC（SPI3 接続）
- PWM 8 チャネル（TIM3 と TIM5 の各 4 チャネル）
- RGB ステータス LED とユーザー LED（いずれもアクティブ High）
- ユーザーボタンと 4 ビット BCD ロータリ DIP スイッチ
- USB OTG FS デバイスポート
- 独立ウォッチドッグ（IWDG）

対応機能
********

Twister が認識する機能は ``gpio``、``uart``、``spi``、``pwm``、``usb_device``、``dma`` である。

配線とピン
**********

.. list-table::
   :header-rows: 1

   * - 機能
     - ノード
     - ピン
     - 既定の状態
   * - コンソール / シェル
     - ``usart3``
     - TX=PD8、RX=PD9、115200 baud
     - 有効
   * - CAN FD コントローラ
     - ``fdcan`` （``microchip,mcp251xfd``）
     - SPI2（SCK=PB13、MISO=PB14、MOSI=PB15、CS=PB9）、INT=PC14
     - 有効
   * - ADC
     - ``mcp3208``
     - SPI3（SCK=PC10、MISO=PC11、MOSI=PC12、CS=PD0）
     - 有効
   * - PWM（TIM3）
     - ``pwm3``
     - CH1=PA6、CH2=PA7、CH3=PB0、CH4=PB1
     - 有効
   * - PWM（TIM5）
     - ``pwm5``
     - CH1=PA0、CH2=PA1、CH3=PA2、CH4=PA3
     - 有効
   * - USB デバイス
     - ``usbotg_fs`` （``zephyr_udc0``）
     - DM=PA11、DP=PA12
     - 有効
   * - ステータス LED
     - ``led_r`` / ``led_g`` / ``led_b``
     - PB11 / PE15 / PB10（アクティブ High）
     - 有効
   * - ユーザー LED
     - ``user_led``
     - PE9（アクティブ High）
     - 有効
   * - ユーザーボタン
     - ``user_button``
     - PE8（プルアップ、アクティブ Low）
     - 有効
   * - ロータリ ID スイッチ
     - ``rot1`` / ``rot2`` / ``rot4`` / ``rot8``
     - PE14 / PE11 / PE13 / PE12（プルアップ、アクティブ Low）
     - 有効

``led0`` エイリアスは緑（``led_g``）、``sw0`` は ``user_button``、``watchdog0`` は ``iwdg`` を指す。

MCP2517FD は 20 MHz の発振子を持ち、devicetree ではアービトレーションを 1 Mbps、トランシーバの上限を 5 Mbps としている。
SoC 内蔵 CAN を使う他のボードと違って、この CAN は SPI 越しに見えるため、SPI の転送レートと割り込み応答が CAN のレイテンシに乗る。
``CONFIG_CAN_MCP251XFD_INT_THREAD_STACK_SIZE`` を 2048 に上げてあるのはこのドライバの割り込みスレッド用である。

フラッシュのパーティション
==========================

.. list-table::
   :header-rows: 1

   * - ラベル
     - オフセット
     - サイズ
   * - ``mcuboot``
     - 0x00000000
     - 64 KB（読み出し専用）
   * - ``image-0``
     - 0x00010000
     - 448 KB
   * - ``image-1``
     - 0x00080000
     - 448 KB
   * - ``storage``
     - 0x000f0000
     - 64 KB

ビルドと書き込み
****************

.. code-block:: shell

   west build -b fibril_canmotor_tourobo2023 app
   west flash

``west flash`` の runner は STM32CubeProgrammer、OpenOCD、pyOCD、J-Link に対応する。
