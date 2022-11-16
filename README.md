# aichallenge2022-sim  
日本語 | [English](https://github.com/Reee009876/test/blob/main/README_en.md)  

更新日：2022/11/16（暫定情報掲載中）

本リポジトリでは、[自動運転AIチャレンジ2022（シミュレーション）](https://www.jsae.or.jp/jaaic/index.html)の参加者向けに、環境構築手順・大会ルール等、大会に参加するために必要な情報をまとめています。

2021年に行った第３回自動運転AIチャレンジと異なり、本大会では自動運転ソフトウェア[Autoware.universe](https://github.com/autowarefoundation/autoware.universe)と自動運転シミュレータ[AWSIM](https://github.com/tier4/AWSIM)を使用します。下記の手順に沿って環境を構築し、大会へご参加ください。


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
OS: Ubuntu 20.04 or Windows 10  
CPU: Intel Corei7 (6 cores and 12 thread) 以上    
GPU: NVIDIA Geforce RTX 2080 Ti 以上  
詳細は[こちら](https://tier4.github.io/AWSIM/GettingStarted/SetupUnityProject/)  

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

2. インストール
```
sudo apt install libvulkan1
``` 

#### **コースの準備**
1.　大会用コースの実行ファイルをダウンロードし、解凍  
・チュートリアル：[ファイルはこちら](https://drive.google.com/drive/folders/1C9bvsDmBwyz0dpjVC0rFpLNfdovWAJ5_)   
2. パーミッションを図のように変更  
<img src="https://user-images.githubusercontent.com/113989589/202225167-f3058a84-c268-4cc5-838a-28dad2c232de.png" width="30%">  
3. ファイルをダブルクリックで起動    
4. 下記のような画面が表示されることを確認  
<img src="https://user-images.githubusercontent.com/113989589/201992906-734b40f1-4c95-45e0-9edb-ffe0af9f55e3.png" width="50%">

### **Autoware**
本大会用にAutowareの Docker イメージ(CUDA利用）を用意しておりますので、ご利用ください。

#### 事前準備  
下記のインストールをお願いします。
- [docker](https://docs.docker.com/engine/install/ubuntu/)
- [rocker](https://github.com/osrf/rocker)
  - Dockerコンテナ内のRviz、rqtなどのGUIを使用するために用います。
- [Nvidia Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)
- [git lifs](https://packagecloud.io/github/git-lfs/install)
#### **Dockerイメージの準備・起動 〜 Autowareの準備・起動**  
1. Dockerイメージを入手
```
docker pull ghcr.io/automotiveaichallenge/aichallenge2022-sim/autoware-universe-cuda:latest
```

2. 大会用マップデータの準備
```
git clone https://github.com/AutomotiveAIChallenge/aichallenge2022-sim
```

3. ダウロードしたフォルダに移動  
```
cd ./aichallenge2022-sim
```

4. rockerを起動
```
rocker --nvidia --x11 --user --net host --privileged --volume autoware:/home/$USER/autoware -- ghcr.io/automotiveaichallenge/aichallenge2022-sim/autoware-universe-cuda:latest
```

5. Autowareを起動
```
ros2 launch autoware_launch e2e_simulator.launch.xml vehicle_model:=sample_vehicle sensor_model:=awsim_sensor_kit map_path:=autoware/nishishinjuku_autoware_map
```

6. 下記のような画面(Rviz2)が表示されることを確認
<img src="https://user-images.githubusercontent.com/113989589/202221115-a3f9ef16-453f-4a7c-bb57-be362886146c.png" width="50%">  

※Autowareの使い方は[公式ドキュメント](https://autowarefoundation.github.io/autoware-documentation/main/)を参考にしてください。

### **動作確認**
AutowareとAWSIMを実行し、以下の手順を参考に動作確認をお願いします。
1. Rviz2のタブにあるPanelからadd new Panelを開き、AutowareStatePanelを追加
<img src="https://user-images.githubusercontent.com/113989589/202221441-aa264504-79cd-40c4-95d6-8eeef9b67993.png" width="50%"><img src="https://user-images.githubusercontent.com/113989589/202221955-2f803b65-1928-46db-9492-98575f015958.png" width="50%">  

2. 自己位置推定ができていることを確認
<img src="https://user-images.githubusercontent.com/113989589/201994441-6d6da145-37de-48a4-8be7-2054c592be46.png" width="50%">  

3. 正しく推定できていなければ、タブにある2D Pose Estimateを選択し、実際の車両の位置をドラッグで指定。
<img src="https://user-images.githubusercontent.com/113989589/201995212-20b73d6a-2e67-4e13-8829-5d8184241eaf.png" width="50%">  

4. タブにある2D Goal Poseを選択し、ゴールポジションをドラッグで指定
<img src="https://user-images.githubusercontent.com/113989589/201996010-92560a86-cc3c-4684-a04e-c161b0b603ea.png" width="50%">  

5. 画像のように、ルートが表示されている かつ 「waiting for engage」状態になっていることを確認（指定してから少し時間がかかります）
<img src="https://user-images.githubusercontent.com/113989589/201994813-1d6ef19e-3485-4812-aeba-e7ee92eff110.png" width="50%">  

6. engageボタンを押下し、自動運転が開始されることを確認  
<img src="https://user-images.githubusercontent.com/113989589/201994840-57f2288d-c311-4e7b-a2fe-97a030d5351e.png" width="50%">  

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
  

