# [任意]描画有りのシミュレーション

## 描画有りのシミュレーション環境（環境構築上級者向け）

デフォルトで描画無しのAWSIMを配布しておりますが、描画有りを希望される方の環境構築方法も記載しております。GPUを使用する環境構築では詰まって進まなくなる事例が多々ありましたので、初めてのご参加の方はあくまでも参考程度でとしてください。

- OS: Ubuntu 22.04
- CPU: Intel Corei7 (8 cores) or higher
- GPU: NVIDIA Geforce VRAM 8 GB
- Memory: 16 GB or more
- Storage: SSD 60 GB or higher

## 描画有りのシミュレーション環構築手順

基本的に描画無しのAWSIMを配布しておりますが、描画有りを希望される方の環境構築方法も記載しております。GPUを使用する環境構築では詰まって進まなくなる事例が多々ありましたので、おすすめはいたしません。要求性能のページのスペックのPCが用意できない方や初めてのご参加の方はあくまでも参考程度としてください。

- [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)(GPU非搭載の方はスキップ)

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

## NVIDIA Container Toolkit

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

## AWSIMのダウンロード・起動確認

!!! info

    AWSIM は現在準備中です。

1. [Google Drive](https://drive.google.com/drive/) から最新の `AWSIM_GPU.zip` をダウンロードし、`aichallenge-2024/aichallenge/simulator` に展開します。

2. パーミッションを図のように変更します。

   ![パーミッション変更の様子](./images/installation/permmision.png)

実行ファイルが`aichallenge-2024/aichallenge/simulator/AWSIM_GPU/AWSIM.x86_64`に存在していることを確認してください。

## Dockerコンテナの起動

描画有りのシミュレーション推奨環境を満たしており、
NVIDIA関連のinstallが済んでいる方は上記のコマンドではなく、以下のコマンドでコンテナを起動してください。

```bash
cd aichallenge-2024
./docker_run.sh dev gpu
```

## AutowareのビルドとSimulatorの起動

コンテナを起動したターミナル(コンテナ内)で以下を実行します。

```bash
cd /aichallenge
./build_autoware.bash
```

Autowareのビルド後、run_simulator.bashを変更します

```bash
#!/bin/bash

# shellcheck disable=SC1091
source /aichallenge/workspace/install/setup.bash
sudo ip link set multicast on lo
/aichallenge/simulator/AWSIM_GPU/AWSIM.x86_64
```

※実行ファイルが`aichallenge-2024/aichallenge/simulator/AWSIM_GPU/AWSIM.x86_64`に存在していることを確認してください。

以下のコマンドを実行します。

```bash
./run_evaluation.bash
```

以下のような画面が現れたら成功です。

![AWSIM-Autoware](./images/AWSIM%26Autoware.png)
