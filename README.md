# PCB-Grove-for-M5Atom
a PCB to add 3 Grove Connectors to M5Atom

# M5Atom 用 GROVE 接続基板

M5Atom のピンソケット (GPIO) を GROVE コネクタに変換する基板です。
GROVE デバイスの接続が簡単になります。

## 1. 特徴

- 基板上のピンヘッダに M5Atom を直接接続できます
- M5Atom の GPIO を GROVE コネクタ 3 個で取り出せます
- M5Atom のピンソケットを、基板上のピンソケットから取り出せます
- DC ジャック (5V) があり、USB-C ケーブルなしでも給電できます

※ 稼働中の 5V 電源の切り替え (USB-C / DC ジャック) は、動作継続を保証できません。

## 2. 詳細

回路図をご確認ください。

| GROVE | pin 1 | pin 2 | pin 3 | pin 4 | 想定用途 | 備考 |
|:-:|:-|:-|:-:|:-:|:-:|:-:|
|  J3  | G21 (SCL) | G25 (SDA) | +5V | GND  | I2C | プルアップ抵抗(*1)設定可 |
|  J4  | G33 | G23, G34 | +5V | GND  | アナログ入力 | - |
|  J5  | G22 (RX) | G19 (TX) | +5V | GND  | シリアル |  保護抵抗(*2)バイパス可 |

(*1) ジャンパーパッド JP1, JP2 を短絡すると、プルアップ抵抗 (10kΩ) が有効となります。I2C などで外付けのプルアップ抵抗として使えます。

(*2) ジャンパーパッド JP3, JP4 を短絡すると、保護抵抗 (1kΩ) がバイパスされます。5V 系のシリアルインタフェースと接続する場合の過電圧保護用に予め挿入しています。

DC ジャックからの電源にはフューズ (PTC リセッタブル、保持電流 1.1A, トリップ電流 2.2A) があります。ちなみに M5Atom 内部には 5V 出力に 0.5A のフューズがあります。
