# 車両の動かし方

## 各チームに提供される情報

当日までに以下の情報が各チームに共有されます．

| 項目           | 用途                  |
| ------------------ | --------------------- |
| 車両番号　(A1-A8)   | 各チームに割り当てられている車両を識別する番号         |
| 車両ECU用ユーザー名       | 車両ECUにログイン・SSH接続するためのユーザー名         |
| 車両ECU用パスワード             | 車両ECUにログイン・SSH接続するためのパスワード         |


## 注意事項

- スクリプトがパスに依存しているため、ホーム直下の `aichallenge-2024` は名前を変更しないようお願いします。
- ROSBAGが自動で記録されないため、走行時に記録コマンドの実行をお願いします。
- Zenoh で通信はデフォルトで topic の受信しかできない設定になっています。走行中の設定変更などを行いたい場合は以下のどちらかで対応をお願いします。
    - Zenoh Bridge を使わず ssh で接続して ECU 内から直接実行する
    - ECU の `aichallenge-2024` 内にある設定ファイル (`vehicle/zenoh.json5`) の `allow` 部分をコメントアウトする (55-62行目)
- 車両番号を指定して接続するには arp-scan というソフトのインストールが必要ですので以下のコマンドでインストールしてください
    - `sudo apt install arp-scan`

## 車両 ECU への接続方法

- 手元のPCを `CCTB_office_01` というWi-Fiに接続する(車両 ECU と同じネットワーク)。
- 手元のPCで `cd aichallege-2024/remote` を実行して作業ディレクトリを移動する
- 手元のPCで `bash connect_ssh.bash <車両名> <ユーザー名>` を実行する (例 `bash connect_ssh.bash A9 aic-team`)
- 手元のPCのパスワードを聞かれた場合は入力する
- 車両 ECU のパスワードを聞かれるので入力する
- 上記のコマンドが使えない場合
    - 運営スタッフに車両の `<IPアドレス>` を問い合わせください
    - 手元のPCで `ssh <ユーザー名>@<IPアドレス>` を実行する

## 車両ECU接続後の操作

### 1. 各種ドライバやDockerコンテナの起動

```
cd aichallenge-2024
./docker_build.sh dev (最初に1回実行すればOKです)
bash run_vehicle_tmux.sh 
```
以下のように端末が分割されコマンドが実行されます．

- 左側: ./docker_run dev cpu が起動し aichallenge-2024 のコンテナ内に入る
- 右側1段目： ./docker_run dev cpu が起動し aichallenge-2024 のコンテナ内に入る
- 右側2段目：車両のドライバソフトが起動する
- 右側3段目：Zenohのブリッジが起動する
- 右側4段目：特になし
 

### 2. Autowareの起動

Dockerコンテナ内で行います．デフォルトでは左側か右上のコンテナの端末です．

```
cd /aichallenge
./build_autoware.bash (最初に1回実行後は、ビルドが必要な変更を行った際に実行してください)
./run_autoware.bash vehicle (autowareが起動し準備完了)
```

### 3. ROSBAGの記録

Dockerコンテナ内で行います．デフォルトでは左側か右上のコンテナの端末です．

```
cd /aichallenge
source workspace/install/setup.bash
ros2 bag record -a

# 動作上問題はありませんが、警告が出るのが嫌な方は代わりに以下のコマンドを実行してください
ros2 bag record -a -x "(/racing_kart/.*|/to_can_bus|/from_can_bus)"
```

## 手元のPCとECUでROS通信したい場合

- SSHした車両のECU内で `ip a` を実行し、車両のIPアドレスを確認してください（もしくは運営スタッフに問い合わせください）
- 手元のPCで `docker pull eclipse/zenoh-bridge-ros2dds` をしておく（最初の１回でOKです）
- 手元のPCで `aichallenge-2024/vehicle/run_zenoh.bash` の最終行を次のように書き換えます
    - 変更前: `eclipse/zenoh-bridge-ros2dds:latest -c /vehicle/zenoh.json5`
    - 変更後: `eclipse/zenoh-bridge-ros2dds:latest -e "tcp/<車両のIPアドレス>:7447"`
- 書き換えたら `run_zenoh.bash` を起動します
- 終了時は手元のPCで別の端末を起動し
    - `docker ps` で`コンテナID`を調べ
    - `docker stop <コンテナID>` で終了します（少し時間が掛かります）
