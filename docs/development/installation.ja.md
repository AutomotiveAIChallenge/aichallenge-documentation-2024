# 環境構築

ここでは、AWSIMの環境構築から起動まで行います。

## NVIDIAドライバのインストール

```bash
#リポジトリの追加
sudo add-apt-repository ppa:graphics-drivers/ppa
#パッケージリストの更新
sudo apt update
#インストール
sudo ubuntu-drivers autoinstall
#再起動
reboot
#再起動の後、インストールできていることを確認
nvidia-smi
```

![nvidia-smi](./images/installation/nvidia-smi.png)

## Vulkunのインストール

```bash
#パッケージリストの更新
sudo apt update
#libvulkan1をインストール
sudo apt install libvulkan1
```

## Git LFSのインストール

```bash
curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | sudo bash
sudo apt-get install git-lfs
git lfs install
```

## 大会データのダウンロード

```bash
cd
git lfs clone https://github.com/AutomotiveAIChallenge/aichallenge-2024.git
```

## AWSIMのダウンロード・起動確認

<!-- TO-DO:Google Driveのリンク差し替え -->

1. [GoogleDrive](https://drive.google.com/drive/folders/)から最新の`AWSIM_GPU.zip`をダウンロードし、`aichallenge-2024/aichallenge/simulator`に解凍してください。
2. パーミッションを図のように変更してください。
   ![パーミッション変更の様子](./images/installation/permmision.png)
3. ファイルをダブルクリックで起動
4. 下記のような画面が表示されることを確認
   ![awsim](./images/installation/awsim.png)

---

## Docker環境のインストール

下記のインストールを行います。

- [Docker](https://docs.docker.com/engine/install/ubuntu/)
- [rocker](https://github.com/osrf/rocker)(Dockerコンテナ内でRviz、rqtなどのGUIを使用するために用います。)
- [Git LFS](https://packagecloud.io/github/git-lfs/install)
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

#以下の様なメッセージが出ればインストール完了です。

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

#User Groupに自分のアカウントを追加し、sudo権限が無くてもDockerを利用可能にします。
sudo usermod -aG docker $USER

#Groupsにdockerが追加されていることを確認
groups $USER
# $ USERNAME : USERNAME adm cdrom sudo ... docker

# ここまで確認できたらLoginし直してください。
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

#最後のコマンドで以下のような出力が出れば成功です。
#（下記はNVIDIAウェブサイトからのコピペです）

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

---

## Dockerイメージの準備・起動

### Dockerイメージを入手

```bash
docker pull ghcr.io/automotiveaichallenge/autoware-universe:humble-latest

docker images
##下記メッセージを確認し、Dockerイメージがダウンロードできていることを確認する。。
# REPOSITORY                                        TAG                       IMAGE ID       CREATED         SIZE
# ghcr.io/automotiveaichallenge/autoware-universe   humble-latest             30c59f3fb415   13 days ago     8.84GB
```

### 大会用Dockerイメージのビルド

```bash
cd aichallenge2024
bash docker_build.sh dev
```

### 大会用Dockerコンテナの起動

ターミナルを2つ用意します。

- コンテナを起動します。

```bash
# GPU搭載の方は以下
cd aichallenge-2024
bash docker_run.sh dev gpu

# GPU非搭載の方は以下
cd aichallenge-2024
bash docker_run.sh dev cpu
```

- コンテナが起動できているか、別ターミナルで確認します。

```bash
docker images

#以下が出ていれば作成ができています。
#aichallenge-2024-dev                              latest                        df2e83a20349   33 minutes ago   8.9GB
```

### 大会Dockerコンテナを停止する

コンテナが起動しているターミナルで下記コマンドを実行します。

```bash
exit
```

以上でセットアップは終了となります。

## 大会環境の起動

本節では大会環境を起動します。

### Autoware

コンテナを起動します。

```bash
# GPU搭載の方は以下
cd aichallenge-2024
bash docker_run.sh dev gpu

# GPU非搭載の方は以下
cd aichallenge-2024
bash docker_run.sh dev cpu
```

- Autowareのビルド

コンテナを開いたターミナル(コンテナ内)で以下を実行します。

```bash
cd /aichallenge
bash build_autoware.bash
```

- Autowareの起動

```bash
bash run_autoware.sh
```

下記の様な画面が表示されたら起動完了です。
![autoware](./images/installation/autoware.png)

終了するにはターミナル上でCTRL + Cを入力します。

### AWSIM in Docker

コンテナを起動します。

```bash
# GPU搭載の方は以下
cd aichallenge-2024
bash docker_run.sh dev gpu

# GPU非搭載の方は以下
cd aichallenge-2024
bash docker_run.sh dev cpu
```

コンテナを開いたターミナル(コンテナ内)で以下を実行します。

```bash
cd /aichallenge
bash run_simulator.bash
```

下記の様な画面が表示されたら起動完了です。
![awsim](./images/installation/awsim.png)

終了するにはターミナル上でCTRL + Cを入力します。

## 変更点の取り込み

大会環境のアップデートがあった際には以下を実行してください。

### Dockerのupdate

```bash
docker pull ghcr.io/automotiveaichallenge/autoware-universe:humble-latest

```

### Repositoryのupdate

```sh
cd aichallenge2024 # path to aichallenge2024
git pull origin/main
```

<br>

## TroubleShooting

Q. `docker_run.sh: 行 35: rocker: コマンドが見つかりません`

A. [rockerのインストール](#docker環境のインストール)をお願いします。

Q. `WARNING unable to detect os for base image 'aichallenge-2024-dev', maybe the base image does not exist`

A. Dockerイメージのビルドをお願いします。
