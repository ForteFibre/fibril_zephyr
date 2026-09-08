fibril_zephyr
=============

ForteFibre のロボット用基板で動く Zephyr ファームウェアのドキュメント。
リポジトリの入口は :doc:`overview` にある。

API リファレンスは Doxygen が別に生成する。

.. toctree::
   :maxdepth: 2
   :caption: ガイド

   overview
   testing

.. toctree::
   :maxdepth: 1
   :caption: ドライバ

   drivers/amt21
   drivers/qdec_stm32
   drivers/robomaster

.. toctree::
   :maxdepth: 1
   :caption: ボード

   boards/fibril_canmotor_tourobo2023
   boards/fibril_robomaster_miniv1
   boards/fibril_robomaster_miniv3
   boards/fibril_robomaster_miniv4
   boards/fibril_rc26_mainair_v01

.. toctree::
   :maxdepth: 1
   :caption: 設計判断

   adr/index

索引
====

* :ref:`genindex`
* :ref:`search`
