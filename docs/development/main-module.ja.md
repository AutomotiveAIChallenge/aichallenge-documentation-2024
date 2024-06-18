# メインモジュール

## [任意]Mapの編集

2024年度のAIチャレンジでは[VectorMapBuilder](https://tools.tier4.jp/feature/vector_map_builder_ll2/)などのツールを使ってpoint cloud map , lanelet2 mapなどの地図の編集を推奨しています。

[Mapのファイル置き場](https://drive.google.com/drive/folders/1nsYIg_3rwIjg0x6BC__aWVmnv-25nZkn)からpointcloud map lanelet2 mapなどをダウンロードして編集してみましょう！

[VectorMapBuilderの使い方動画](https://www.youtube.com/watch?v=GvZr707TmuM)にステップバイステップのインストラクションなどがあるので参考にしてみてください。

作成したlanelet2 mapは`aichallenge/workspace/src/aichallenge_submit/aichallenge_submit_launch/map`に格納しましょう！

## Autowareのカスタマイズ

本大会では、自動運転ソフトウェアAutowareをベースとした実装を用意しております．
本ページでは、その背景と説明に加えて、どのように本実装を活用できるかの紹介を行います．

前回のシミュレーション大会では、デフォルトのAutowareから機能を絞り、ノード数を減らした[縮小構成のAutowareを起動できるLaunchファイルを提供]()しました．その際の、背景や用意した意図については、[前大会のドキュメント](https://automotiveaichallenge.github.io/aichallenge2023-racing/customize/index.html)をご覧ください．

今回のシミュレーション大会では、前大会と同様にAutowareの部分的な活用や自由自在な取り込みを可能にするため、[AWSIMとの利用を想定した縮小構成のAutowareを用意しました](https://github.com/AutomotiveAIChallenge/aichallenge-2024/blob/main/aichallenge/workspace/src/aichallenge_submit/aichallenge_submit_launch/launch/reference.launch.xml)しました．

## 縮小構成のAutowareを用意した背景

### Autowareを利用する時の課題

デフォルトのAutowareでは様々な走行環境に対応するため、たくさんのノードから構成されています．

Autowareの公式ドキュメンテーションでは、[Autowareを構成するROSノードの構成図](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-architecture/node-diagram/)を見ることもできます．以下に現時点の図を示します．

![node-diagram](./images/architecture/autoware-node-diagram.png)

自動運転に関わる各コンポーネントで多種多様な機能を揃えており、難易度の高い走行環境にも対応できるように作られています．

一方、その複雑な構成を理解し、各パラメータの意味や調整の仕方、モジュールの切り替え・入れ替えなどを行うことが必ずしも容易ではなくなっています．

### 縮小構成のAutoware-Microの用意

そのため、前回のシミュレーション大会では、デフォルトのAutowareから機能を絞り、ノード数を減らしたAutowareを用意しました．

Autoware-Microのノード図を以下に示します．ノード数が格段と減り、基本的な自動走行を可能とする機能のみが揃っていることが分かります．

![micro-node-diagram](./images/architecture/reference-autoware.png)

Autoware-Microの特徴としては、以下が挙げられます．

- ほぼ全てのノードの起動を直接1つのLaunchファイルから行っている．
- パラメータを直接ノード起動時に書き込んでいるため、どのパラメータがどのノードで必要なのかを簡単に追うことができる．
- 各ノードの入出力となるROSトピック名も直接ノード起動時に`remap`しているため、トピック名も簡単に変えられる．

このAutowareをベースに自動運転ソフトを書くことで、以下のようなことができます．

- シンプルな構成となっているため、Autowareの中身をより深く理解できる
- 自作のモジュールをAutowareのものと簡単に入れ替えることができ、機能の改善に取り組める
- パラメータを変更した場合のシステム全体の動作への影響が分かりやすい
- 今回のAutowareには含まれていない既存のAutowareのノードを追加することもできる

各コンポーネントの変更点・特徴としては、以下が挙げられます．

- Localization：GNSS、IMU及び車輪速による自己位置推定
- Planning：behavior_velocity_plannerやobstacle_stop_plannerなどを省略し、出力経路から走行trajectoryを直接出力するように変更．
- Control：制御の1つの実装例としてsimple_pure_pursuitを用意．

## Autoware-Microの活用方法

Autoware-Microを活用することにより、本大会での課題となる：

1. カーブなどの戦略的な経路計画
2. 高速での車両制御

に集中して取り組むことができるようになります．

また、Autoware-Microの実装例を参考にしながら、Autowareのアーキテクチャとは少し異なる実装方法を試したり、新しくカスタムのノードを作成・導入したりすることができます．

独自のノードの実装を取り入れることにより、走行性能を向上させ点数を伸ばすことができます．

例えば、以下のような構成を考え、「Planning」と「Control」をそれぞれ実装して取り組んだり、「Planning & Control」を両方担うノードを実装できます．

ルートの入力と車両インターフェイス出力のROSトピックさえ合っていれば自由にカスタマイズして頂けます．

![racing-diagram](./images/architecture/racing_simple.png)

昨年度の取り組みはこちらのアドベントカレンダーにまとめられていますので参考にしてみてください。

[アドベントカレンダー](https://qiita.com/advent-calendar/2023/jidounten-ai)

どれから読もうか迷った方は2023年度コミュニティ貢献賞を受賞した田中新太さんが記載してくれた[こちらの記事](https://qiita.com/Arata-stu/items/4b03772348dca4f7ef89)から読み進めると良いと思います。

## 独自実装の作成例

とりあえず新たに自作パッケージを作成してみたい方は既存のパッケージをコピーしたり、[autoware practice](https://github.com/AutomotiveAIChallenge/autoware-practice)をコピーする形で以下のように進めると良いと思います。

1. 元のパッケージをコピーして、下記を変更
   - パッケージ名
   - フォルダ名
   - コード
   - package.xml
   - CMakeLists.txt
2. aichallenge_submitの中に配置
3. autoware_micro_awsim_launchから呼び出されるlaunchファイルを変更
   - 参考例：pose_initializer_custom（ autoware_universe_launch/tier4_localization_launch/launch/util/util.launch.xmlから呼び出しております）

※コピー元のパッケージのライセンスを違反しないよう各自確認お願いいたします。
