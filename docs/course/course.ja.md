# Autoware 入門講座

## はじめに

この講座では Autoware の基本的な開発方法について解説します。
講座は演習形式となっており、課題を達成するためのコードをゼロから開発しながら Autoware の仕組みを学べるようになっています。
各講座のページにはメニューから移動してください。

## 環境構築

任意のディレクトリにて入門講座のリポジトリをクローンし、ビルドを行ってください。

```bash
git clone https://github.com/AutomotiveAIChallenge/autoware-practice.git
cd autoware-practice
vcs import src < autoware.repos
colcon build --symlink-install
```

また、コマンドを実行する際は事前に端末で以下のコマンドが実行されている必要があります。

```bash
source install/setup.bash
```
