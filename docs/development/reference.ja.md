# 参考

## 変更点の取り込み

大会環境の重大なアップデートがあった際には適宜アナウンスがあります。
参考までにこちらに記載しています。以下を実行してください。

Dockerのupdate

```bash
docker pull ghcr.io/automotiveaichallenge/autoware-universe:humble-latest
```

Repositoryのupdate

```sh
cd aichallenge2024 # path to aichallenge2024
git pull origin/main
```

## 描画ありAWSIMの導入

AWSIMのシミュレーション画面を確認したい方は、[こちら](./setup/visible-simulation.ja.md)の手順に従って描画ありAWSIMの導入を行ってください。

## Debug用にTerminalを3つ用意して開発したい場合 (参考)

`Alt+Ctrl+T`で１つ目のターミナルを立ち上げてから、以下のコマンド`Ctrl+Shift+P`で貼り付けた後に`Enter`で実行します。

```bash
cd ~/aichallenge-2024
./docker_run.sh dev cpu
```

```bash
cd /aichallenge
bash run_simulator.bash
```

`Alt+Ctrl+T`で2つ目のターミナルを立ち上げてから、以下のコマンド`Ctrl+Shift+P`で貼り付けた後に`Enter`で実行します。

```bash
cd ~/aichallenge-2024
./docker_run.sh dev cpu
```

```bash
cd /aichallenge
bash run_autoware.bash
```

`Alt+Ctrl+T`で3つ目のターミナルを立ち上げてから、以下のコマンド`Ctrl+Shift+P`で貼り付けた後に`Enter`で実行します。

```bash
cd ~/aichallenge-2024
./docker_run.sh dev cpu
```

```bash
cd /aichallenge
ros2 topic pub --once /control/control_mode_request_topic std_msgs/msg/Bool '{data: true}' >/dev/null
```

下記の様な画面が表示されたら起動完了です。終了するには各ターミナル上でCTRL + Cを入力します。
![autoware](./images/autoware.png)
## TroubleShooting

!!! warning

    基本的には、ChatGPTやGoogle検索で解決できる項目は多いです。それでも解決しない質問については、エラーのログの抜粋と添付をお願いします。

Q. `docker_run.sh: 行 35: rocker: コマンドが見つかりません`

A. [rockerのインストール](./setup/docker.ja.md)をお願いします。

Q. `WARNING unable to detect os for base image 'aichallenge-2024-dev', maybe the base image does not exist`

A. Dockerイメージのビルドをお願いします。

Q. Dockerがpullできません。

A. `newgrp docker`か`sudo service docker restart`でdockerの再起動またはUbuntuの再起動をお願いします。