# aichallenge2022-sim
更新日：2022/11/1（暫定情報掲載中）

本リポジトリでは[自動運転AIチャレンジ2022（シミュレーション）](https://www.jsae.or.jp/jaaic/index.html)の参加者のための環境構築手順・大会ルール等、大会参加のために必要なデータを提供しています。   

競技内容・ルールについては[RULE.md](./RULE.md)をご確認ください。

本大会では、[Autoware.universe](https://github.com/autowarefoundation/autoware.universe)と自動運転シミュレータ[AWSIM](https://github.com/tier4/AWSIM)を使用します。
下記の手順に沿って環境を構築し、大会へご参加ください。


## 動作環境
本大会で使用していただくPCの動作環境として以下を推奨しております。

OS: Ubuntu 20.04    
CPU: Intel Corei7(8コア)以上    
GPU:  NVIDIA Geforce 3080(VRAM 12GB)以上  
メモリ: 32GB以上    
ストレージ: SSD 30GB 以上  

上記のスペックのPCが用意できない場合、Autoware動作PCとAWSIMシミュレータ動作PCを分けて用意していただくことも可能です。
環境はそれぞれ以下を推奨しております。  
※同一NW内であれば特別な設定をすることなく、PC間のトピック通信が可能です。
万が一、トピック通信ができなかった場合はFWの設定を確認してください。

### **AWSIMシミュレータ動作PC**  
OS: Ubuntu 20.04   
CPU: Intel Corei7 (6 cores and 12 thread) 以上  
GPU: NVIDIA Geforce RTX 2080 Ti 以上  
詳細は[こちら](https://tier4.github.io/AWSIM/GettingStarted/SetupUnityProject/)  

### **Autoware動作PC**   
OS: Ubuntu 20.04  
CPU: Intel Corei7 (8 cores) 以上  
GPU: NVIDIA Geforce GTX 1080 以上  
メモリ: 16 GB以上  
ストレージ: SSD 10 GB 以上  
詳細は[こちら](https://autowarefoundation.github.io/autoware.universe/main/)  


## 環境セットアップ
### **大会用コース（AWSIM）**
#### 事前準備
##### **・Nvidiaドライバのインストール**
1. Add Nvidia driver to apt repository  
```
sudo add-apt-repository ppa:graphics-drivers/ppa
```

2. Check supported Nvidia driver versions
```
sudo apt update
```

3. Install the recommended version
```
sudo ubuntu-drivers autoinstall
```

4. reboot and check nvidia-smi
```
nvidia-smi 
```

##### ・Vulkunのインストール

1. Update environment.
```
sudo apt update
```

2. Install libvulkan1
```
sudo apt install libvulkan1
``` 

#### **・大会用コース（AWSIM）の準備**
1. 各難易度用の実行ファイルをダウンロードし、解凍  
・チャレンジコース：　　  
・アドバンストコース：　　
3. パーミッションを図のように変更  
![Screenshot from 2022-10-23 21-13-47](https://user-images.githubusercontent.com/113989589/197391459-50955a4b-24e4-4120-ba80-6d4d1de5fdaa.png)
4. 起動

#### **・大会用コースのマップデータ**  
[こちら](https://github.com/AutomotiveAIChallenge/aichallenge2022-sim/releases/download/latest/nishishinjuku_autoware_map.zip)からダウンロードしてください。

### **Autoware**

コンテナイメージ（CUDA利用）を用意しておりますので、ご利用ください。

#### 事前準備  
- [docker](https://docs.docker.com/engine/install/ubuntu/)
- [rocker](https://github.com/osrf/rocker)
  - Dockerコンテナ内のRviz、rqtなどのGUIを使用するために用いります。
  
#### **dockerイメージの準備・Autowareの起動**  
1. イメージをPULL   
```
docker pull ghcr.io/automotiveaichallenge/aichallenge2022-sim/autoware-universe-cuda:latest
```

2. rockerを起動
```
rocker --net host --nvidia --x11 --user --privileged -- ghcr.io/automotiveaichallenge/aichallenge2022-sim/autoware-universe-cuda:latest
```

3. Autowareを起動（例） 
```
source install/setup.bash
```
```
ros2 launch autoware_launch e2e_simulator.launch.xml vehicle_model:=sample_vehicle sensor_model:=awsim_sensor_kit map_path:=<your mapfile location>
```
※Autowareの使い方は[公式ドキュメント](https://autowarefoundation.github.io/autoware-documentation/main/)を参考にしてください。

以下、後日更新。

## サンプルコード
### **概要**
### **ビルド**
### **実行**



## オンライン評価環境について
### subTitle1
### subTitle2
### subTitle3

## その他
### 更新等の通知に関して
githubの更新などがある場合は、以下のURLのissueに新たにコメントします。
本issueをsubscribeいただければ、更新時に通知されます（通知をオンにしてください）。
https://github.com/AutomotiveAIChallenge/aichallenge2022-sim/issues/1
### 問い合わせ方法
競技内容、リポジトリ内容物等に関するお問い合わせについては、github上のissueにてお願いします。質問は日本語、英語どちらでも構いません。
なお、質問内容は競技内容に直接関係あるものに限ります。また、ソフトウェアの使用方法に関するご質問についても、公平性の観点から回答いたしかねます。
  
質問者様は質問が解決した際issueをcloseしてください。  
  
各issueでの質問については、基本的に2営業日以内に回答いたします。ただし、検討に時間を要する質問や質問数が多い場合等については、2営業日以上いただく可能性があることはご理解ください。   
  
オンラインシミュレータにログインできないなど、オンラインシミュレータのアカウントに関するお問い合わせはai-challenge@jsae.or.jp宛にお願いいたします。  
  

