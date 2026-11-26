---
status: Proposed
date: 2026-10-13
---

# 0015. hub が繋ぐバスは fibril_can のデバイスだけにし、モータのバスへの生の転送はしない

## Status

Proposed

## Context

CanMotorMbed の miniv4 には、2 次のバス（RobStride、ODrive）のフレームを、主バスと USB の gs_usb へ転送するブリッジがあった（`tmp/saramander/robstride500k`、`tmp/catch26/can_bridge`）。
2 次のバスは classic CAN（500k か 1M）なので、FD のフレームを classic に変換していた。
主バスの旧プロトコルが使う std ID（`0x10`、`0x170+id`）は、拒否リストで 2 次のバスに漏らさなかった。

移植の計画では、これを fibril_can の `fibril,can-hub` で置き換えるつもりだった。
しかし、hub と fibril_can の ID 空間は、この用途に合わない。

- hub は、どの port のフレームも、ほかのすべての port へ中継する。送るフレームは必ず CAN FD（`FDF|BRS`）にし、peer の受信フィルタは全受けである。port ごとのフィルタも、FD から classic への変換も無い。fibril_can の SPEC §2.5 は、中継要素に「CAN ID とペイロードをビット等価に配送する」ことを求めており、変換はこれに反する。
- fibril_can の ext ID は、上位 3 bit が channel（0 が config、1 が service request、2 が service response）である（SPEC §3.2）。RobStride の ext ID は上位 5 bit が通信種別で、運転の指令とフィードバック（種別 1、2）は channel 0 に当たる。
- fibril_can の std ID は、Topic としてマスターが動的に割り当てる（SPEC §3.1）。ODrive の `node_id << 5 | cmd`、RoboMaster の `0x1FF`、`0x200`、`0x201`〜`0x208` と重なる。

旧プロトコルは std ID を数個しか使わなかったので、拒否リストで済んでいた。
fibril_can では、ID 空間のほぼ全体を使うので、同じやり方では済まない。

## Decision

1. **hub に繋ぐバスは、fibril_can のデバイスだけにする。** miniv4 の `miniv4-usb` は、USB の gs_usb を hub の外部 port に、外部 CAN（FDCAN1）を peer にする。FDCAN1 には fibril_can のノードだけを繋ぐ。
2. **モータのバス（FDCAN2、FDCAN3）は hub に入れない。** RoboMaster、RobStride、ODrive は、今までどおりドライバとブロック型で基板の上で閉じる（RobStride は ADR 0006）。
3. **モータのバスのフレームを、ホストへ生のまま転送することは、今の段階ではしない（2026-10-13）。** 必要になったら、下の「却下した選択肢」の gs_usb の別チャネルから検討する。

## Consequences

- ホストから RobStride や ODrive を直接扱うソフト（`robstride_actuator_bridge_ros2`、`ros_odrive`）は、miniv4 を経由しては使えない。これらを fibril_zephyr の基板に載せるときは、ブロック型を使う。ODrive のブロック型は、まだ無い。
- CanMotorMbed のブリッジにあった CAN の診断（送受信の数、FIFO、bus-off、エラーカウンタ）は、移植していない。
- 主バスの経路を選ぶだけの snippet で済むので、`miniv4-usb` は `rc26-mainair-usb` と同じ形になる。

## 却下した選択肢

**hub に、port ごとのモード、ID のフィルタ、FD から classic への変換を足す。**
fibril_can の SPEC §2.5 に反する。
また、ID 空間が重なるので、フィルタを足しても混線は防げない。

**FDCAN1 とモータのバスの間で、生のフレームを転送する部品を fibril_zephyr に作る。**
FDCAN1 には fibril_can のフレームが流れている。
RobStride と ODrive のフレームを混ぜると、マスターと bridge が config、service、Topic として読み違える。

**gs_usb に、モータのバスを別のチャネルとして見せる。**
CANnectivity の `gs_usb_register` はチャネルの配列を取るので、FDCAN2 と FDCAN3 を PC の別の SocketCAN（`can1`、`can2`）にできる。
ID 空間は分かれるので、混線しない。
ホストの既存のソフトも、インタフェース名を変えるだけで済む見込みである。
ただし、ホストが USB で繋がっている基板にしか使えない。
ADR 0006 は、制御周期が USB とホストのスケジューリングを挟むことを理由に、RobStride のゲートウェイを退けている。
今の段階では要求が無いので、作らない。
