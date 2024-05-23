# 要求性能

## 推奨環境

本大会で使用していただくPCの動作環境として以下を推奨しております。

- OS: Ubuntu 22.04
- CPU: Intel Corei7 (8 cores) or higher
- GPU: NVIDIA Geforce RTX 3080 (VRAM 12 GB) or higher
- Memory: 32 GB or more
- Storage: SSD 30 GB or higher

上記のスペックを満たすPCをご用意できない方は、AutowareとAWSIMを別のPCで動かすことも可能です。下記の「PC2台で参加する方向け」のスペックをご参照ください。

## 2台のPCを使用する方向け

### Autoware PC

- OS: Ubuntu 22.04
- CPU: Intel Corei7 (8 cores) or higher
- GPU: NVIDIA Geforce GTX 1080 or higher
- Memory: 16 GB or higher
- Storage: SSD 10 GB or higher
- 詳細は [Autoware Documentation](https://autowarefoundation.github.io/autoware-documentation/main/installation/) を参照してください

### AWSIM PC

- OS: Ubuntu 22.04 or Windows 10/11
- CPU: Intel Corei7 (6 cores and 12 threads) or higher
- GPU: NVIDIA Geforce RTX 2080 Ti or higher
- 詳細は [AWSIM document](https://tier4.github.io/AWSIM/) を参照してください

Autoware動作PCとAWSIM動作PCは、同じネットワーク内に配置してください。配置できていれば、基本的には追加設定をすることなく、PC間のトピック通信は可能です。万が一、トピック通信ができなかった場合はファイアーウォールの解除、もしくはルールの見直しをお願いします。また、Autoware Documentation の [Troubleshooting](https://autowarefoundation.github.io/autoware-documentation/main/support/troubleshooting/performance-troubleshooting/)も参考になるかもしれません。
