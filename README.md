# aichallenge2022-sim  
日本語 | [English](https://github.com/AutomotiveAIChallenge/aichallenge2022-sim/blob/main/README_en.md)  

更新日：2022/12/12

本リポジトリでは、[自動運転AIチャレンジ2022（シミュレーション）](https://www.jsae.or.jp/jaaic/)の参加者向けに、環境構築手順・大会ルール等、大会に参加するために必要な情報をまとめています。

2021年に行った第３回自動運転AIチャレンジと異なり、本大会では自動運転ソフトウェア[Autoware.universe](https://github.com/autowarefoundation/autoware.universe)と自動運転シミュレータ[AWSIM](https://github.com/tier4/AWSIM)を使用します。下記の手順に沿って環境を構築し、大会へご参加ください。

大会ルールの詳細な説明は[RULE.md](/RULE.md)を参照ください。

## コース選択
本大会は初学者向けの「チャレンジコース」と上級者向けの「アドバンストコース」に分かれております。参加者の皆様には両コースに触れていただきご自身のスキルレベルに合わせて最終的にコースを選択していただきます。

オンライン採点環境ではアドバンストコース・チャレンジコース両方への提出が可能ですが、コースを切り替える際にはそれまでの提出スコアを削除する必要があります。

## 動作環境

本大会で使用していただくPCの動作環境として以下を推奨しております。

OS: Ubuntu 20.04    
CPU: Intel Corei7 (8 cores) 以上  
GPU:  NVIDIA Geforce RTX 3080(VRAM 12 GB) 以上    
メモリ: 32 GB 以上     
ストレージ: SSD 30 GB 以上  

上記のスペックを満たすPCをご用意できない方は、下記の「PC2台で参加する方向け」のスペックをご参照ください。

### **PC2台で参加する方向け**  


#### **Autoware動作PC**   
OS: Ubuntu 20.04   
CPU: Intel Corei7 (8 cores) 以上   
GPU: NVIDIA Geforce GTX 1080 以上   
メモリ: 16 GB以上  
ストレージ: SSD 10 GB 以上   
詳細は[こちら](https://autowarefoundation.github.io/autoware.universe/main/)  

#### **AWSIMシミュレータ動作PC**  
OS: Ubuntu 20.04 or windows 10  
CPU: Intel Corei7 (6 cores and 12 thread) 以上    
GPU: NVIDIA Geforce RTX 2080 Ti 以上  
詳細は[こちら](https://tier4.github.io/AWSIM/GettingStarted/QuickStartDemo/)  

※Autoware動作PCとAWSIM動作PCは、同じネットワーク内に配置してください。    
配置できていれば、基本的には追加設定をすることなく、PC間のトピック通信は可能です。万が一、トピック通信ができなかった場合はファイアーウォールの解除、もしくはルールの見直しをお願いします。


## 環境セットアップ


### **AWSIM（Ubuntu）**
#### 事前準備
##### **・Nvidiaドライバのインストール**
1. リポジトリの追加
```
sudo add-apt-repository ppa:graphics-drivers/ppa
```

2. パッケージリストの更新
```
sudo apt update
```

3. インストール
```
sudo ubuntu-drivers autoinstall
```

4. 再起動の後、インストールできていることを確認
```
nvidia-smi 
```
<img src="https://user-images.githubusercontent.com/113989589/202224587-b5b7b34e-5ed6-4b0d-9a7c-d04b5aa0dd25.png" width="50%">  


##### ・Vulkunのインストール

1. パッケージリストの更新
```
sudo apt update
```

2. libvulkan1をインストール
```
sudo apt install libvulkan1
``` 

#### **コースの準備**
1.　大会用コースの実行ファイルをダウンロードし、解凍  
・チャレンジコース：[ファイルはこちら](https://drive.google.com/drive/u/0/folders/19ThwqQbOFkc201yZIM_OhoWKPuruEsW2)   
・アドバンストコース：[ファイルはこちら](https://drive.google.com/drive/u/0/folders/12-2XlZgsE9mvjlT6b94skfY-6C6-5vI0)   
2. パーミッションを図のように変更  
<img src="https://user-images.githubusercontent.com/113989589/202225167-f3058a84-c268-4cc5-838a-28dad2c232de.png" width="40%">  
3. ファイルをダブルクリックで起動    
4. 下記のような画面が表示されることを確認  
<img src="https://user-images.githubusercontent.com/113989589/201992906-734b40f1-4c95-45e0-9edb-ffe0af9f55e3.png" width="70%">

### **AWSIM（Windows10）**
#### **コースの準備**
1.　大会用コースの実行ファイルをダウンロードし、解凍  
・チャレンジコース：[ファイルはこちら](https://drive.google.com/drive/u/0/folders/19ThwqQbOFkc201yZIM_OhoWKPuruEsW2)   
・アドバンストコース：[ファイルはこちら](https://drive.google.com/drive/u/0/folders/12-2XlZgsE9mvjlT6b94skfY-6C6-5vI0)   
2. ファイルをダブルクリックで起動    
3. 下記のような画面が表示されることを確認  
<img src="https://user-images.githubusercontent.com/113989589/202367079-ff4fc373-a296-4091-aa49-416c0b69df1f.png" width="70%">



### **Autoware**
本大会用にAutowareの Docker イメージ(CUDA利用）を用意しておりますので、ご利用ください。

#### 事前準備  
下記のインストールをお願いします。
- [docker](https://docs.docker.com/engine/install/ubuntu/)
- [rocker](https://github.com/osrf/rocker)
  - Dockerコンテナ内のRviz、rqtなどのGUIを使用するために用います。
- [Nvidia Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)
- [git lfs](https://packagecloud.io/github/git-lfs/install)
- [ROS2](https://docs.ros.org/en/galactic/index.html)（動作確認済みバージョン：Galactic）
#### **Dockerイメージの準備・起動 〜 Autowareの準備**  
1. Dockerイメージを入手
```
docker pull ghcr.io/automotiveaichallenge/aichallenge2022-sim/autoware-universe-cuda:latest
```

2. 大会用データのダウンロード
```
sudo apt install -y git-lfs
git lfs clone https://github.com/AutomotiveAIChallenge/aichallenge2022-sim
```

3. rockerを起動
```
cd ./aichallenge2022-sim
rocker --nvidia --x11 --user --net host --privileged --volume autoware:/aichallenge -- ghcr.io/automotiveaichallenge/aichallenge2022-sim/autoware-universe-cuda:latest
```

### **サンプルコード(ROS2パッケージ)**

#### **サンプルコードについて**
参加者の皆様にはシナリオを遂行するROS2パッケージを作成していただきますが、本リポジトリ内でそのベースとなるサンプルコードとして`autoware/aichallenge_ws/src`に以下のROS2パッケージを提供しております。
- aichallenge_launch
  - 大元のlaunchファイル`aichallenge.launch.xml`を含んでいます。すべてのROS2ノードはこのlaunchファイルから起動されます。
- aichallenge_eval
  - スコア算出用のパッケージです。
- aichallenge_score_msgs
  - メッセージ定義を含みます。
- aichallenge_submit
  - このディレクトリの内容は自由に変更していただいて構いません。
  - 提出時にはこのディレクトリの内容のみ提出していただきますので、参加者の皆さまが実装されたROS2パッケージはすべてこのディレクトリ内に配置してください。配布段階で以下のパッケージを含んでいます。
  - aichallenge_submit_launch
    - `aichallenge_submit_launch.launch.xml`が大元のlaunchファイル`aichallenge.launch.xml`から呼び出されますので、このlaunchファイルを適宜改修して皆様が実装されたROS2ノードが起動されるように設定してください。
  - sample_code_cpp
    - サンプルの自動走行実装です。

### **サンプルコードビルド**
```
# Rockerコンテナ内で
cd /aichallenge/aichallenge_ws
rosdep update
rosdep install -y -r -i --from-paths src --ignore-src --rosdistro $ROS_DISTRO
colcon build
```

皆様に作成していただいたROS2パッケージについても`aichallenge_ws/src/aichallenge_submit`以下に配置していただき、上記手順でビルドできるようにしてください。

### **サンプルコード起動**
```
# Rockerコンテナ内で
source /aichallenge/aichallenge_ws/install/setup.bash
ros2 launch aichallenge_launch aichallenge.launch.xml
```

ここまででAutoware側の設定・実行は完了です。セットアップが正常に行われていれば、rvizには点群地図が表示されます。


### **動作確認**
AutowareとAWSIMを用いて動作確認を行う方法を記載します。
1. AWSIMを起動
2. Autowareを起動
```
# Rockerコンテナ内で
cd /aichallenge
ros2 launch autoware_launch e2e_simulator.launch.xml vehicle_model:=sample_vehicle sensor_model:=awsim_sensor_kit map_path:=nishishinjuku_autoware_map
```

3. 下記のような画面(Rviz2)が表示されることを確認
<img src="https://user-images.githubusercontent.com/113989589/202221115-a3f9ef16-453f-4a7c-bb57-be362886146c.png" width="50%">  

※Autowareの使い方は[公式ドキュメント](https://autowarefoundation.github.io/autoware-documentation/main/)を参考にしてください。

4. RvizのタブにあるPanelからadd new Panelを開き、AutowareStatePanelを追加
<img src="https://user-images.githubusercontent.com/113989589/202221441-aa264504-79cd-40c4-95d6-8eeef9b67993.png" width="70%">
<img src="https://user-images.githubusercontent.com/113989589/202221955-2f803b65-1928-46db-9492-98575f015958.png" width="70%">  

5. 自己位置推定ができていることを確認
<img src="https://user-images.githubusercontent.com/113989589/206501339-a713f027-d694-44d4-a15f-d5894bce0ae1.png" width="70%">  

6. 正しく推定できていなければ、タブにある2D Pose Estimateを選択し、実際の車両の位置をドラッグで指定
<img src="https://user-images.githubusercontent.com/113989589/206501742-a9b8cd85-9ad2-49a3-af52-a67b45e66c17.png" width="70%">  

7. タブにある2D Goal Poseを選択し、ゴールポジションをドラッグで指定
<img src="https://user-images.githubusercontent.com/113989589/206502195-42aa0b92-928e-4759-8b25-f58a7a99680b.png" width="70%">  

8. 画像のように、ルートが表示されている かつ 「waiting for engage」状態になっていることを確認（指定してから少し時間がかかります）
<img src="https://user-images.githubusercontent.com/113989589/206502874-6bd0e54e-0b04-45b5-a1f6-a83605a6c972.png" width="70%">  

9. engageボタンを押下し、自動運転が開始されることを確認  
<img src="https://user-images.githubusercontent.com/113989589/206503383-cd28fb0c-2553-45e6-bf98-b9b1d1412991.png" width="70%">  

## タイム取得
タイム取得方法については[RULE.md](/RULE.md)を参照ください。

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
  
オンラインシミュレータにログインできないなど、オンラインシミュレータのアカウントに関するお問い合わせはinfo-ai@jsae.or.jp宛にお願いいたします。  
  

