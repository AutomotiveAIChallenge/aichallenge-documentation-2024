# 提出

## オンライン環境

本大会では、シミュレーターと自動採点機能を備えたオンライン環境を使用して採点が行われます。以下の手順に従って、作成したパッケージ群をオンライン環境にアップロードしてください。アップロード後、シミュレーションが自動で開始され、結果が表示されます。

### オンライン環境へのアップロード手順

1. 動作確認

    1.1. 事前準備

    - `aichallenge_submit`を圧縮し、結果出力用のフォルダを生成します。
    - `./create_submit_file.bash`

    1.2. Dockerイメージのビルド

    - `./docker_build.sh eval`

    1.3. Dockerコンテナの起動

    - 起動後、自動でAutowareが立ち上がり、自動運転が開始されます。
    - GPU版AWSIMを使用している場合: `./docker_run.sh eval gpu`
    - CPU版AWSIMを使用している場合: `./docker_run.sh eval cpu`

    1.4. `result.json`の確認

    - 評価完了後、`output/latest`フォルダに以下のファイルが格納されます。
        - `autoware.log`
        - `rosbag2_autoware`
        - `capture`
        - `result-summary.json`
        - `result-details.json`

2. オンライン環境にアップロード

    <img src="./images/siteImage.png" width="100%">

    [オンライン環境](https://aichallenge-board.jsae.or.jp/live)にアクセスし、緑色の「UPLOAD」ボタンから`aichallenge_submit.tar.gz`をアップロードしてください。アップロード後、ソースコードのビルドとシミュレーションが順に実施されます。

    - 正常に終了した場合、採点完了と表示され、`result.json`がダウンロードでき、各ラップタイムが表示されます。
    - シナリオ実行後、launchに失敗した等でスコアが出力されていない場合は「結果無し」と表示されます。この場合、サーバーサイドでの内部エラーの可能性があるため、再アップロードをお願いします。問題が続く場合はお問い合わせください。
    - ビルドに失敗した場合は「ビルド失敗」と表示されます。手順の確認後、再度アップロードしてください。
    - ランキングはこれまでの採点での最高点が適用されます。
    - 採点実行中は新たなソースのアップロードはできません。
    - アップロードは1日に10回まで可能で、日本時間0時に回数がリセットされます。

3. 結果の確認

    オンライン環境で評価が終わると、`result.json`がダウンロード可能になります。ダウンロードして結果を確認してください。

4. 結果なしの場合

    4.1. packageの依存関係に問題がないか確認

    - 使用言語に応じて、`package.xml`、`setup.py`、または`CMakeLists.txt`に依存関係の漏れがないか確認してください。

    4.2. dockerの確認

    - 以下のコマンドでDocker内を確認し、必要なディレクトリに正しくインストール・ビルドされているか確認してください。

        - `docker run -it aichallenge-2024-eval:latest /bin/bash`

    - 確認するディレクトリ:

        - `/aichallenge/workspace/*`
        - `/autoware/install/*`
