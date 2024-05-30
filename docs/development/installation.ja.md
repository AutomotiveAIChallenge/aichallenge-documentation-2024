# 環境構築

## ワークスペースのダウンロード

任意のディレクトリにて下記コマンドを実行し、ワークスペースをダウンロードします。

```bash
git clone https://github.com/AutomotiveAIChallenge/aichallenge-2024.git
```

## NVIDIAドライバのインストール

```bash
# リポジトリの追加
sudo add-apt-repository ppa:graphics-drivers/ppa

# パッケージリストの更新
sudo apt update

# インストール
sudo ubuntu-drivers autoinstall

# 再起動
reboot

# 再起動の後、インストールできていることを確認
nvidia-smi
```

![nvidia-smi](./images/installation/nvidia-smi.png)

## Vulkunのインストール

```bash
# パッケージリストの更新
sudo apt update

# libvulkan1をインストール
sudo apt install libvulkan1
```

## AWSIMのダウンロード・起動確認

!!! info

    AWSIM は現在準備中です。

1. [Google Drive](https://drive.google.com/drive/) から最新の `AWSIM_GPU.zip` をダウンロードし、`aichallenge-2024/aichallenge/simulator` に展開します。

2. パーミッションを図のように変更します。

   ![パーミッション変更の様子](./images/installation/permmision.png)

3. ファイルをダブルクリックで起動します。

4. 下記のような画面が表示されることを確認します。

   ![awsim](./images/installation/awsim.png)

## Docker環境のインストール

下記のインストールを行います。

- [Docker](https://docs.docker.com/engine/install/ubuntu/)
- [rocker](https://github.com/osrf/rocker)(Dockerコンテナ内でRviz、rqtなどのGUIを使用するために用います。)
- [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)(GPU非搭載の方はスキップ)

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

# 以下コマンドはROS依存があるため非推奨としています。
# sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros2.list'
# curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
# sudo apt update
# sudo apt-get install python3-rocker
```

### (GPU非搭載の方はスキップ) NVIDIA Container Toolkit

```bash
# インストールの下準備
distribution=$(. /etc/os-release;echo $ID$VERSION_ID) \
      && curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
      && curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | \
            sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
            sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

# インストール
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker

#インストールできているかをテスト
sudo docker run --rm --runtime=nvidia --gpus all nvidia/cuda:11.6.2-base-ubuntu20.04 nvidia-smi

#最後のコマンドで以下のように出力されれば成功です。
#（下記はNVIDIAウェブサイトからの引用です）
#
# +-----------------------------------------------------------------------------+
# | NVIDIA-SMI 450.51.06    Driver Version: 450.51.06    CUDA Version: 11.0     |
# |-------------------------------+----------------------+----------------------+
# | GPU  Name        Persistence-M| Bus-Id        Disp.A | Volatile Uncorr. ECC |
# | Fan  Temp  Perf  Pwr:Usage/Cap|         Memory-Usage | GPU-Util  Compute M. |
# |                               |                      |               MIG M. |
# |===============================+======================+======================|
# |   0  Tesla T4            On   | 00000000:00:1E.0 Off |                    0 |
# | N/A   34C    P8     9W /  70W |      0MiB / 15109MiB |      0%      Default |
# |                               |                      |                  N/A |
# +-------------------------------+----------------------+----------------------+
# +-----------------------------------------------------------------------------+
# | Processes:                                                                  |
# |  GPU   GI   CI        PID   Type   Process name                  GPU Memory |
# |        ID   ID                                                   Usage      |
# |=============================================================================|
# |  No running processes found                                                 |
# +-----------------------------------------------------------------------------+
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
