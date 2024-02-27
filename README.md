# aichallenge-documentation

[![.github/workflows/document.yml](https://github.com/AutomotiveAIChallenge/aichallenge-documentation/actions/workflows/document.yml/badge.svg?branch=main)](https://github.com/AutomotiveAIChallenge/aichallenge-documentation/actions/workflows/document.yml)

このリポジトリは、自動車技術会主催の自動運転AIチャレンジで使用するドキュメントサイトのテンプレートです。参加者の皆様は、ご自身の参加される大会の専用ページをご覧ください。

- <https://github.com/AutomotiveAIChallenge/aichallenge2023-racing>
- <https://github.com/AutomotiveAIChallenge/aichallenge2023-integ>
- <https://github.com/AutomotiveAIChallenge/aichallenge2022-sim>

## Sample

生成されるドキュメントサイトの例は以下で確認できます。

- [日本語ページ](https://automotiveaichallenge.github.io/aichallenge-documentation/)
- [English Page](https://automotiveaichallenge.github.io/aichallenge-documentation/en/)

## Deploy

ドキュメントの生成環境をインストールするには以下のコマンドを実行します。

```bash
pip install -r requirements.txt
```

ドキュメントをローカル環境で確認したい場合は以下を実行します。

```bash
mkdocs serve
```

ドキュメントを GitHub Pages に反映させる場合は以下を実行します。

```bash
mkdocs gh-deploy
```
