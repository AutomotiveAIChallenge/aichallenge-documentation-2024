# ワークスペース

## 推奨環境

本大会で使用していただくPCの動作環境として以下を推奨しております。

- OS: Ubuntu 22.04
- CPU: Intel Core i5（4コア）以上（推奨）
- メモリ:
  - 8GB以上（最低）
  - 16GB以上（推奨）
- SSD: 60GB以上


## 大会用リポジトリのビルド・実行
大会用リポジトリでは、実際の動作環境はすべてDocker内で完結して提供されています。リポジトリの利用は以下の流れで行います。
1. 大会環境のDockerイメージのビルド
2. Dockerコンテナ上でのAutowareのビルド
3. Dockerコンテナ上でのAutowareとシミュレータの同時起動

## 大会環境のDockerイメージのビルド
大会用リポジトリに入ります。
```
cd ~/aichallenge-2024
```

Dockerイメージのビルドを行います。
```
./docker_build.sh dev
```

```
docker images
```
で以下のイメージが生成されていることを確認しましょう。
```
aichallenge-2024-dev   latest   df2e83a20349   33 minutes ago   8.9GB
```

## Dockerコンテナ上でのAutowareのビルド
以下を実行してDockerコンテナを立ち上げます。

```bash
cd ~/aichallenge-2024
./docker_run.sh dev cpu
```

特に何も変わっていないように見えますが、上記のコマンドを実行すると環境がDockerコンテナ内部に移行します。今の環境がDockerコンテナ内かどうかは以下のコマンドで何も表示されていないかを確認するのがわかりやすいです。

```bash
ls ~
```

Dockerコンテナ内で以下を実行してAutowareをビルドします。

```bash
cd /aichallenge
./build_autoware.bash
```

## Dockerコンテナ上でのAutowareとSimulatorの実行

Autowareのビルド後、以下のコマンドを実行します。

```bash
./run_evaluation.bash
```

下記の様な画面が表示されたら起動完了です。終了するにはターミナル上でCTRL + Cを入力します。
![autoware](./development/images/installation/autoware.png)


A. Dockerイメージのビルドをお願いします。

#### ワークスペースの構成

**docker-dev**

![dev](./images/docker/dev.drawio.svg)

**docker-eval**

![eval](./images/docker/eval.drawio.svg)
