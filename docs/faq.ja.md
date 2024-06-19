# FAQ

## 環境構築

### <u>AWSIM and Autoware間の通信が安定しません。</u>

local でテストする際、すべての terminal で`ROS_LOCALHOST_ONLY=1`に設定すると通信速度が向上します。
.bashrc に以下の行を追加してください。

```bash
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

if [ ! -e /tmp/cycloneDDS_configured ]; then
    sudo sysctl -w net.core.rmem_max=2147483647
    sudo ip link set lo multicast on
    touch /tmp/cycloneDDS_configured
```

なお、今回の大会ではPC2 台構成の Windows+Linux、Linux+Linuxの二台構成も考慮しています。
その場合は、 `ROS_LOCALHOST_ONLY=0`としてください。

注意:

- OS の起動後、ターミナルの起動時にパスワードが要求され、初回には `sudo ip link set lo multicast on` が必要です。
- 一度上記のように.bashrc に書き込んで変更したことを忘れると常に適用されてしまうことになるため、`echo $ROS_LOCALHOST_ONLY`で確認するなど必ず変更点は追ってください。
- `ROS_LOCALHOST_ONLY=1`と`ROS_LOCALHOST_ONLY=0`が混在していると container 間の通信ができません。
- `ROS_LOCALHOST_ONLY`が実行ファイルに記載されていることには注意してください。

---

### <u>ros2 topic list が表示されません。</u>

あなたのマシンの`ROS_DOMAIN_ID`が一致していることを確認してください。（ROS_DOMAIN_ID を設定していない方は問題ないです）
また、ROS2 がソースされていることの確認をお願いします。

---

### <u>WindowsのAWSIMとUbuntuのAutowareを使用しており、$ ros2 topic list が表示されません。</u>

Windows Firewallでの通信を許可してください。
また、`ros2 daemon stop`と`ros2 daemon start`を実行して、不要なプロセスが残っていないか確認し、再起動をお願いします。

---

### <u>Rockerが起動しません。</u>

まず、rockerがインストールされているかの確認をお願いします。
インストールされているにも関わらず、起動しない場合は権限をご確認ください。過去の事例ですと、イメージをビルドしたアカウントと実行する際のアカウントの種類・権限が異なると実行できないことが報告されています。

---

### <u>AWSIMがコアダンプで終了します。</u>

AWSIMを起動した直後にcoredumpで終了する場合、GPUのメモリが不足している可能性があります。そのため、`nvidia-smi`でGPUメモリの利用率が限界に達していないか確認してください。
なお、GPUのメモリは11GB以上を推奨しています。

---

### <u>GPU搭載のWindowsPCしか用意できませんでした。</u>

本大会のサポート対象はHP記載の構成になりますため、詳細のご案内はできませんが、一般的に下記のような方法があると思われます。

参加するためには、Autoware環境が問題です。
そのため、いかに「Autowareを動かすための環境を用意」するかがポイントになるため、
性能やパッケージの有無、ホスト-コンテナ内の通信設定などの問題が起きる可能性がありますが、以下の方法があると思います。

- デュアルブートでUbuntuを用意
- Windows上にVMでUbuntuを用意 (Hyper-V、VirtualBox、VMwareなど)
- WSL2上にUbuntuを用意
- Windows上にdocker環境を用意（直接、Autowareのイメージを入れる）
- クラウドに環境を構築 (過去の大会ではAWSを利用して参加されている方もいらっしゃいました)

---

### <u>AWSで環境構築したところ、AWSIMは表示されたが、Rvizがブラックスクリーンとなりました。</u>

`sudo apt upgrade`で治ったという事例がありますので、内容を確認の上、お試しください。
また、[過去Issue](https://github.com/ros2/rviz/issues/948)にてご質問内容と似た質問がありましたので、こちらも合わせてご確認ください。

---

---

## 操作

### ROS

#### <u>pythonでパッケージを作成すると実行時 no module named \* のerrorが起きます。</u>

[こちら](https://zenn.dev/tasada038/articles/5d8ba66aa34b85#setup.py%E3%81%ABsubmodules%E3%81%A8%E3%81%97%E3%81%A6%E3%83%91%E3%83%83%E3%82%B1%E3%83%BC%E3%82%B8%E3%82%92%E8%BF%BD%E5%8A%A0%E3%81%99%E3%82%8B)を参考にしてみてください。

---

#### <u>トピックの型を調べるには、どのようなコマンドを打てばよろしいでしょうか。</u>

topicの型を調べる際は`ros2 topic info -v fuga_topic`もしくはnodeが特定できれば、`ros2 node info hoge-node`で調べることができます。
その他にもROSに関する情報を調べたい場合は「ROS2　コマンド」で、ネット検索すると良いかもしれません。

---

---

### Autoware

#### <u>Rviz上で地図・ルートが表示されません。</u>

使用しているマップデータが適切な場所に配置されいるか・正しいかを確認してください。

---

#### <u>どのようにしてAutowareを改良して参加すればよいかが分かりません。</u>

Autowareのノードのパラメータ調整やノード改良・置き換えなどが方法としてあります。
Autowareの基本構成などを本サイトの別タブや[こちら](https://automotiveaichallenge.github.io/aichallenge2023-integ/customize/index.html)に少しまとめておりますので、ご活用ください。
また、外部の方の記事ですが、[こちら](https://qiita.com/h_bog/items/86fba5b94b2148c4d9da)も参考になるかもしれません。

---

#### <u>経路生成（Behavior Path/Motion Planner）に関して教えてください。</u>

behavior plannerは、主にODD3以上のいわゆる一般道での走行を行うのに必要な機能（一時停止線、横断歩道、信号停止）など破ってはいけない交通ルールを加味したplanningを行うものとなっています。
それ故、回避機能もルールベースの回避で最適化を行っていません。
一方でmotionはODD2以下のいわゆる限定区域や限定空間での走行を実現するもので、例えば信号や、地図の情報等といった情報を扱うものはありません。
障害物の回避や、停止、速度の最適化など、通常走行に必要な機能を担うものとなっています。

---

#### <u>Autowareの回避行動について教えて下さい</u>

回避には二種類あり、behavior pathとobstacle avoidanceがあります。
デフォルトではobstacle avoidabceの回避はoffで、経路の平滑化のみ行われる設定になっています。
また、デフォルトではbehavior pathで回避する設定にはなっていますがその際の回避対象物は車とトラックのみです。

---

#### <u>center pointについて教えて下さい。</u>

center pointは車両とトラックと歩行者を検知してくれますが、ダンボールなどタグ付けされていないものは検知できません。
ただ、現状のautowareとしてはplanningがobjectを受け取らないと動かないようになっており、objectを受け取る段階でcenter pointを使うデフォルトの構成にしていると、以下の2つの原因により不具合が起こります。

1. center pointが死んだときにplanningが経路を生成できなくなる
2. data associationでclusteringによる障害物検知結果が消される

そのため、perceptionの構成はautoware miniが理想的ですが、このあたりを理解してノードの足し引き、取捨選択をして実装することははなかなか難しいため、center pointが問題なく動くようにすることは重要になってくるかもしれません。
[参考](https://autowarefoundation.github.io/autoware.universe/main/perception/lidar_centerpoint/)

---

---

### AWSIM

#### <u>車を初期位置にリセットするにはどうすればいいでしょうか。</u>

現状、AWSIMを再起動する方法しかございません。

---

#### <u>AWSIMの動作が安定しません。</u>

GPUの性能不足が原因の一つになります。
高性能GPUの利用が難しい場合は、awsimの画面の下部にスライドバーでtime scaleを0.5くらいに設定すると安定して動作する可能性があります。

---

#### <u>mpcのチューニングをしたいのですが，今回AWSIMで使用されているモデルパラメータ（遅れや時定数など）は公開されていないでしょうか．</u>

遅れや時定数については計測も公開もされていませんが、基本的な仕様については[こちら](./specifications/simulator.html)に公開されています。

---

---

## 大会全般

### <u>センサの追加取り付けは可能ですか。</u>

同一条件・難易度で課題に取り組んでいただくために、新たなセンサの取り付けは不可としています。

---
