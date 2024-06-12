# はじめ方

このページではROS2やAutowareに馴染みのない初学者向けに、AIチャレンジでの一連の流れをチュートリアルとして記載します。

## 必要なもの

## Ubuntu PC

以下のスペックを満たすPCが必要ですが、推奨と書かれているものについては満たしていなくても動作することは可能です。ただし、推奨よりも低いスペックで動作させる場合ROS2側での実行速度が安定せずシミュレーションの実行の度に挙動が大きく変わってしまう可能性があります。

- OS: Ubuntu 22.04
- CPU: Intel Core i5（4コア）以上（推奨）
- メモリ:
  - 8GB以上（最低）
  - 16GB以上（推奨）
- SSD: 60GB以上

Windows環境しかお持ちでない方は、Ubuntu22.04のインストールをお願いいたします。Windows環境と同じディスクにUbuntu環境を入れることもできますが、不慣れな場合Windows環境を破壊してしまう可能性があるため、新しく外付けまたは内蔵SSDを購入したうえでそちらへのインストールをすることを強くお勧めします。

!!! info

    Ubuntuのインストール方法については[こちらの記事](https://qiita.com/kiwsdiv/items/1fa6cf451225492b33d8)が参考になります。

## AIチャレンジの環境構築

`Alt+Ctrl+T`でターミナルを立ち上げてから、以下に従ってコマンドを実行します。

[環境構築の説明ページ](./development/installation.md)

## 大会用リポジトリのビルド・実行

環境構築が終わってから再度`Alt+Ctrl+T`でターミナルを立ち上げてから、以下に従ってコマンドを実行します。

[ワークスペースの使い方説明ページ](development/workspace.md)

## AIチャレンジでの開発の進め方

AIチャレンジで開発する上でベースとなるソースコードは[大会用リポジトリ](https://github.com/AutomotiveAIChallenge/aichallenge-2024/tree/main/aichallenge/workspace/src/aichallenge_submit)内に提供されています。参加者の皆様にはこちらのコードをカスタマイズすることで開発を進めていただきますが、Autowareに不慣れな方はまずは[入門講座](./course/index.md)を一通りやっていただくことをお勧めします。

## 参考

### 変更点の取り込み

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

### TroubleShooting

Q. `docker_run.sh: 行 35: rocker: コマンドが見つかりません`

A. [rockerのインストール](./development/installation.md)をお願いします。

Q. `WARNING unable to detect os for base image 'aichallenge-2024-dev', maybe the base image does not exist`
