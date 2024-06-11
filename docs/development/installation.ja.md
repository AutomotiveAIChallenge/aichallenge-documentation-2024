# 環境構築

こちらのページはチュートリアルの項目を簡素にしたものになります。
初参加の方は[チュートリアル](../tutorial.ja.md)のページをご参照ください。

## ワークスペースのダウンロード

任意のディレクトリにて下記コマンドを実行し、ワークスペースをダウンロードします。

```bash
git clone https://github.com/AutomotiveAIChallenge/aichallenge-2024.git
```

## 必要なパッケージのインストール

```bash
sudo apt update
sudo apt -y upgrade
sudo apt install -y git python3-pip ca-certificates curl gnupg libvulkan1
sudo ubuntu-drivers autoinstall
```

## AWSIMのダウンロード・起動確認

!!! info

    AWSIM は現在準備中です。

1. [Google Drive](https://drive.google.com/drive/) から最新の `AWSIM.zip` をダウンロードし、`aichallenge-2024/aichallenge/simulator` に展開します。

2. パーミッションを図のように変更します。

   ![パーミッション変更の様子](./images/installation/permmision.png)


## Docker環境のインストール

下記のインストールを行います。

- [Docker](https://docs.docker.com/engine/install/ubuntu/)
- [rocker](https://github.com/osrf/rocker)(Dockerコンテナ内でRviz、rqtなどのGUIを使用するために用います。)


### Dockerのインストール

```bash
# Dockerインストールの下準備
sudo apt-get update
sudo apt-get install ca-certificates curl gnupg
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg
echo \
  "deb [arch="$(dpkg --print-architecture)" signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  "$(. /etc/os-release && echo "$VERSION_CODENAME")" stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

# 最新のDockerをインストール
sudo apt-get update
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

# インストールできているかをテスト
sudo docker run hello-world

# 以下の様なメッセージが出ればインストール完了です。
#
# Hello from Docker!
# This message shows that your installation appears to be working correctly.
#
# To generate this message, Docker took the following steps:
#  1. The Docker client contacted the Docker daemon.
#  2. The Docker daemon pulled the "hello-world" image from the Docker Hub.
#     (amd64)
#  3. The Docker daemon created a new container from that image which runs the
#     executable that produces the output you are currently reading.
#  4. The Docker daemon streamed that output to the Docker client, which sent it
#     to your terminal.
#
# To try something more ambitious, you can run an Ubuntu container with:
#  $ docker run -it ubuntu bash
#
# Share images, automate workflows, and more with a free Docker ID:
#  https://hub.docker.com/
#
# For more examples and ideas, visit:
#  https://docs.docker.com/get-started/

# User Groupに自分のアカウントを追加し、sudo権限が無くてもDockerを利用可能にします。
sudo usermod -aG docker $USER

# Groupsにdockerが追加されていることを確認
groups $USER

# 以下の様なメッセージが出れば設定完了です。
# USERNAME : USERNAME adm cdrom sudo ... docker

# ここまで確認できたらLoginし直してください。
reboot
```

### rockerのインストール

```bash
# 依存関係が最も少ないため以下コマンドでのインストールを推奨しています。
pip install rocker
# rocker実行ファイルへのPATHを通します。
echo export PATH='$HOME/.local/bin:$PATH' >> ~/.bashrc
source ~/.bashrc
```

## Dockerイメージの準備

Dockerイメージは10GB程度のサイズがあり、ダウンロードには時間が掛かります。

```bash
# Dockerイメージのダウンロード
docker pull ghcr.io/automotiveaichallenge/autoware-universe:humble-latest

# Dockerイメージの確認
docker images

# Dockerイメージがダウンロードできていれば以下のような出力が得られます。
#
# REPOSITORY                                        TAG                       IMAGE ID       CREATED         SIZE
# ghcr.io/automotiveaichallenge/autoware-universe   humble-latest             30c59f3fb415   13 days ago     8.84GB
```

ここまでできたら最低限必要な環境構築は**完了**です。
次のページにいきましょう。

