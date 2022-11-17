# aichallenge2022-sim  
日本語 | [English](https://github.com/Reee009876/test/blob/main/README_en.md)  

Last updated：2022/11/16（provisional information）  

This repository contains information for participants of the [Automated AI Challenge 2022 (Simulation)](https://www.jsae.or.jp/jaaic/index.html), including the procedure of building the development environment, competition rules, and other information.  


This competition will use autonomous driving software [Autoware.universe](https://github.com/autowarefoundation/autoware.universe) and a self-driving vehicles simulator [AWSIM](https://github.com/tier4/AWSIM), unlike the 3rd Automated Driving AI Challenge held in 2021. Please follow the steps below to build your environment and participate in the competition.   

 
 

## Development Environment   
We recommend that you use the following system requirements in this tournament. 

OS: Ubuntu 20.04  
CPU: Intel Corei7 (8 cores) or higher  
GPU: NVIDIA Geforce 3080 (VRAM 12 GB) or higher  
Memory: 32 GB or more  
Storage: SSD 30 GB or higher     

If you cannot prepare a PC that meets the above specifications, please refer to the "For participants with two PCs" specifications below.   

### **For participants with two PCs**  


#### **Autoware PC**   
OS: Ubuntu 20.04  
CPU: Intel Corei7 (8 cores) or higher  
GPU: NVIDIA Geforce GTX 1080 or higher  
Memory: 16 GB or higher  
Storage: SSD 10 GB or higher  
For more information, [click here](https://autowarefoundation.github.io/autoware.universe/main/)  .

#### **AWSIM PC**  
OS: Ubuntu 20.04 or Windows 10  
CPU: Intel Corei7 (6 cores and 12 threads) or higher  
GPU: NVIDIA Geforce RTX 2080 Ti or higher  
For more information, [click here](https://tier4.github.io/AWSIM/GettingStarted/SetupUnityProject/)  .

※PC should be on the same network. 
If that, you can use topic communication without additional settings. In the unlikely event that topic communication is not possible, please deactivate the firewall or review the rules. 


## Environment Setup 


### **AWSIM(Ubuntu)**
#### Advance Preparation 
##### **Installing NVIDIA Drivers**
1. Add the repository. 
```
sudo add-apt-repository ppa:graphics-drivers/ppa
```

2. Update the package list. 
```
sudo apt update
```

3. Install the driver using ubuntu-drivers. 
```
sudo ubuntu-drivers autoinstall
```

4. Restart your system and verify the successful installation of the driver. 
```
nvidia-smi 
```
<img src="https://user-images.githubusercontent.com/113989589/202224587-b5b7b34e-5ed6-4b0d-9a7c-d04b5aa0dd25.png" width="50%">  


##### ・Installing Vulkun 

1. Update the package list. 
```
sudo apt update
```

2. Install libvulkan1. 
```
sudo apt install libvulkan1
``` 

#### **Course Preparation**
1.　Download and unzip the executable of the course for the competition.    
・Tutorial: [click here](https://drive.google.com/drive/folders/1C9bvsDmBwyz0dpjVC0rFpLNfdovWAJ5_)   
2. Change "aichallenge_tutorial_ubuntu.x86_64" permissions as shown below:   
<img src="https://user-images.githubusercontent.com/113989589/202225167-f3058a84-c268-4cc5-838a-28dad2c232de.png" width="40%">  
3. Double-click the file to start AWSIM.   
4. You will see the AWSIM window:   
<img src="https://user-images.githubusercontent.com/113989589/201992906-734b40f1-4c95-45e0-9edb-ffe0af9f55e3.png" width="70%">

### **AWSIM(Windows10)**

#### **Course Preparation**
1.　Download and unzip the executable of the course for the competition.    
・Tutorial: [click here](https://drive.google.com/drive/folders/1C9bvsDmBwyz0dpjVC0rFpLNfdovWAJ5_)   
2. Double-click the file to start AWSIM.   
3. You will see the AWSIM window:   
<img src="https://user-images.githubusercontent.com/113989589/202367079-ff4fc373-a296-4091-aa49-416c0b69df1f.png" width="70%">


### **Autoware**
We recommend that you use the Docker image of Autoware (using CUDA) for the competition. 

#### Advance Preparation  
Please install the following: 
- [docker](https://docs.docker.com/engine/install/ubuntu/)
- [rocker](https://github.com/osrf/rocker)
  -  A tool to use Rviz, rqt, and other GUI application in Docker containers. 
- [Nvidia Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)
- [git lifs](https://packagecloud.io/github/git-lfs/install)
#### **Starting Docker Image and Autoware**  
1. Pull the Docker image using docker pull. 
```
docker pull ghcr.io/automotiveaichallenge/aichallenge2022-sim/autoware-universe-cuda:latest
```

2. Get the data for the competition. 
```
sudo apt install -y git-lfs
git lfs clone https://github.com/AutomotiveAIChallenge/aichallenge2022-sim
```

3. Go into the directory.   
```
cd ./aichallenge2022-sim
```

4. Start rocker. 
```
rocker --nvidia --x11 --user --net host --privileged --volume autoware:/home/$USER/autoware -- ghcr.io/automotiveaichallenge/aichallenge2022-sim/autoware-universe-cuda:latest
```

5. Start Autoware. 
```
ros2 launch autoware_launch e2e_simulator.launch.xml vehicle_model:=sample_vehicle sensor_model:=awsim_sensor_kit map_path:=autoware/nishishinjuku_autoware_map
```

6. You will see the Rviz2 window: 
<img src="https://user-images.githubusercontent.com/113989589/202221115-a3f9ef16-453f-4a7c-bb57-be362886146c.png" width="50%">  
※For how to use Autoware, refer to [the official documentation](https://autowarefoundation.github.io/autoware-documentation/main/). 



### **Operation Verification**
To verify that AWSIM and Autoware are installed and running, follow these steps. 
1. Click "Panels" -> "Add new panel" from the Panel in the Rviz2 tab and add AutowareStatePanel.
  <img src="https://user-images.githubusercontent.com/113989589/202221441-aa264504-79cd-40c4-95d6-8eeef9b67993.png" width="70%">
  <img src="https://user-images.githubusercontent.com/113989589/202221955-2f803b65-1928-46db-9492-98575f015958.png" width="70%">

2. You can see that self-location estimation is working.
  <img src="https://user-images.githubusercontent.com/113989589/201994441-6d6da145-37de-48a4-8be7-2054c592be46.png" width="70%">  

3. Note that in some cases, you may have to select 2D Pose Estimate in the tab and drag the actual position of the vehicle.
  <img src="https://user-images.githubusercontent.com/113989589/201995212-20b73d6a-2e67-4e13-8829-5d8184241eaf.png" width="70%">  

4. Select 2D Goal Pose in the tab and specify the goal position by dragging.
  <img src="https://user-images.githubusercontent.com/113989589/201996010-92560a86-cc3c-4684-a04e-c161b0b603ea.png" width="70%">  

5. You can see that the route is displayed and "WAITING FOR ENGAGE" status as shown below (it can take several minutes to run):
  <img src="https://user-images.githubusercontent.com/113989589/201994813-1d6ef19e-3485-4812-aeba-e7ee92eff110.png" width="70%">  

6. Press Engage button, you can see that self-driving started.
  <img src="https://user-images.githubusercontent.com/113989589/201994840-57f2288d-c311-4e7b-a2fe-97a030d5351e.png" width="70%">  

## Others
### Notification of updates
When there are updates on GitHub, we will make a new comment on the issue at the following URL. Please SUBSCRIBE to this issue to be notified of updates (please turn on notifications)https://github.com/AutomotiveAIChallenge/aichallenge2022-sim/issues/1. 

### Contact 
If you have questions about the competition or repository contents, please submit an issue on GitHub. You can ask questions in either English or Japanese.   

Questions must be directly related to the competition. We will not be able to answer questions regarding the use of the software. 

Please close an issue when resolved.   
  
We generally reply to questions within two business days. Please note that depending on the contents of your questions, it may take longer than two business days to answer. 

   

For inquiries regarding an account of an online simulator, for example, if you cannot log in to the online simulator, please contact us. 

email：ai-challenge@jsae.or.jp 

