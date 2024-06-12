# 環境構築

## aptパッケージ等のインストール

まずは諸々最初に必要なパッケージをインストールします。

```bash
sudo apt update
sudo apt -y upgrade
sudo apt install -y git python3-pip ca-certificates curl gnupg libvulkan1
sudo ubuntu-drivers autoinstall
```

## Dockerのインストール

[公式ドキュメント](https://docs.docker.com/engine/install/ubuntu/)通りに以下のコマンドでインストールします。

```bash
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg
echo \
  "deb [arch="$(dpkg --print-architecture)" signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  "$(. /etc/os-release && echo "$VERSION_CODENAME")" stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

sudo apt-get update
sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
sudo usermod -aG docker $USER
```

以下のコマンドで正常にインストールされているか確認してください。

```bash
sudo docker run hello-world
```

`Hello from Docker!`と表示されれば正常にインストール出来ています。

ここまで出来たら一度再起動します。

## rockerのインストール

rockerはDockerコンテナのGUIアプリを簡単に起動できるようにするツールです。

[公式README](https://github.com/osrf/rocker?tab=readme-ov-file#debians-recommended)ではaptからのインストールが推奨されていますが、手順と環境をシンプルにするためにここではpipからインストールします。

```bash
pip install rocker
```

デフォルト設定ではrockerの実行ファイルへのパスが通っていないので、以下のコマンドで`.bashrc`に追加しておきます。

```bash
echo export PATH='$HOME/.local/bin:$PATH' >> ~/.bashrc
source ~/.bashrc
```

## autoware環境のDockerイメージのpull

AIチャレンジで使用するautoware環境のDockerイメージを取得します。

Dockerイメージは10GB程度のサイズがあり、ダウンロードには時間が掛かるため有線LANでのダウンロードを推奨します。

```bash
docker pull ghcr.io/automotiveaichallenge/autoware-universe:humble-latest
```

以下のコマンドでダウンロードできているか確認します。

```bash
docker images
```

Dockerイメージがダウンロードできていれば以下のような出力が得られます。

```txt
REPOSITORY                                        TAG                       IMAGE ID       CREATED         SIZE
ghcr.io/automotiveaichallenge/autoware-universe   humble-latest             30c59f3fb415   13 days ago     8.84GB
```

## 大会用リポジトリのダウンロード

リポジトリをクローンします。ここではホームディレクトリにクローンしますが、お好きなディレクトリに入れていただいても構いません。

```bash
cd ~
git clone https://github.com/AutomotiveAIChallenge/aichallenge-2024.git
```

## AWSIMのダウンロード

1. [Google Drive(あとでリンク変える)](https://drive.google.com/drive/) から最新の `AWSIM_GPU.zip` をダウンロードし、`aichallenge-2024/aichallenge/simulator` に展開します。

2. パーミッションを図のように変更します。

   ![パーミッション変更の様子](./images/installation/permmision.png)

実行ファイルが`aichallenge-2024/aichallenge/simulator/AWSIM/AWSIM.x86_64`に存在していることを確認してください。
