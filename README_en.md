# aichallenge2022-sim  
[日本語](https://github.com/AutomotiveAIChallenge/aichallenge2022-sim) | English

Last updated：2022/12/29

This repository contains information for participants of the [Automated AI Challenge 2022 (Simulation)](https://www.jsae.or.jp/jaaic/), including the procedure of building the development environment, competition rules, and other information.  


This competition will use autonomous driving software [Autoware.universe](https://github.com/autowarefoundation/autoware.universe) and a self-driving vehicles simulator [AWSIM](https://github.com/tier4/AWSIM), unlike the 3rd Automated Driving AI Challenge held in 2021. Please follow the steps below to build your environment and participate in the competition.   

 
 See [RULE_en.md](/RULE_en.md) for a detailed explanation of the tournament rules.

## Course Selection
This competition is divided into a "Challenge Course" for beginners and an "Advanced Course" for experts. Participants will be exposed to both courses and will be asked to make a final course selection based on their own skill level.

The online scoring environment allows submissions to both the Advanced and Challenge Courses, but you will need to delete your previous submitted scores when switching courses.

## Development Environment   
We recommend that you use the following system requirements in this tournament. 

OS: Ubuntu 20.04  
CPU: Intel Corei7 (8 cores) or higher  
GPU: NVIDIA Geforce RTX 3080 (VRAM 12 GB) or higher  
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
For more information, [click here](https://tier4.github.io/AWSIM/GettingStarted/QuickStartDemo/)  .

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
- [git lfs](https://packagecloud.io/github/git-lfs/install)
- [ROS2](https://docs.ros.org/en/galactic/index.html)（Confirmed Operation：Galactic）
#### **Starting Docker Image and Autoware**  
1. Pull the Docker image using docker pull. 
```
docker pull ghcr.io/automotiveaichallenge/aichallenge2022-sim/autoware-universe-cuda:latest
```
※If the above method takes a long time or times out, please use the following command.  
Please use the following command, as we have placed a tar file of the images at [here](https://drive.google.com/drive/u/2/folders/1VZAcGzcFpOBJlmmybcGau7BaHzZW5Chc).
````
gzip -d aichallenge2022_sim_autoware_v2.0.tar.gz  
docker load < aichallenge2022_sim_autoware_v2.0.tar
````

2. Get the data for the competition. 
```
sudo apt install -y git-lfs
git lfs clone https://github.com/AutomotiveAIChallenge/aichallenge2022-sim
```

3. Start rocker. 
```
cd ./aichallenge2022-sim
rocker --nvidia --x11 --user --net host --privileged --volume autoware:/aichallenge -- ghcr.io/automotiveaichallenge/aichallenge2022-sim/autoware-universe-cuda:latest
```

### **Sample code (ROS2 package)**

#### **About the sample code**
We provide the following ROS2 package in `autoware/aichallenge_ws/src` as a sample code to be used as a base in this repository.
- aichallenge_launch
  - Contains the main launch file `aichallenge.launch.xml`. All ROS2 nodes are launched from this launch file.
- aichallenge_eval
  - Package for score calculation.
- aichallenge_score_msgs
  - Contains message definitions.
- aichallenge_submit
  - The contents of this directory may be freely modified.
  - All ROS2 packages implemented by participants should be placed in this directory, as only the contents of this directory will be submitted at the time of submission. The following packages are included in the distribution phase
  - aichallenge_submit_launch
    - Since `aichallenge_submit_launch.launch.xml` is called from the original launch file `aichallenge.launch.xml`, so please modify this launch file so that the ROS2 node in which you are implemented will be launched.
  - sample_code_cpp
    - This is a sample automatic run implementation.

### **sample code build**
````
# In the Rocker container
cd /aichallenge/aichallenge_ws
rosdep update
rosdep install -y -r -i --from-paths src --ignore-src --rosdistro $ROS_DISTRO
colcon build
````

Please place the ROS2 packages you have created under `aichallenge_ws/src/aichallenge_submit` so that they can be built using the above procedure.

### **Example code startup**.
````
# In the Rocker container
source /aichallenge/aichallenge_ws/install/setup.bash
ros2 launch aichallenge_launch aichallenge.launch.xml
````

At this point, the setup and execution on the Autoware side is complete. If the setup was successful, rviz will display a point cloud map.

### **Operation Verification**
This section describes how to check the operation using Autoware and AWSIM.
1. Start AWSIM.
2. Start Autoware. 
```
cd /aichallenge
ros2 launch autoware_launch e2e_simulator.launch.xml vehicle_model:=sample_vehicle sensor_model:=awsim_sensor_kit map_path:=nishishinjuku_autoware_map
```

3. You will see the Rviz2 window: 
<img src="https://user-images.githubusercontent.com/113989589/202221115-a3f9ef16-453f-4a7c-bb57-be362886146c.png" width="50%"> 


※For how to use Autoware, refer to [the official documentation](https://autowarefoundation.github.io/autoware-documentation/main/)

4. Click "Panels" -> "Add new panel" from the Panel in the Rviz2 tab and add AutowareStatePanel.
  <img src="https://user-images.githubusercontent.com/113989589/202221441-aa264504-79cd-40c4-95d6-8eeef9b67993.png" width="70%">
  <img src="https://user-images.githubusercontent.com/113989589/202221955-2f803b65-1928-46db-9492-98575f015958.png" width="70%">

5. You can see that self-location estimation is working.
<img src="https://user-images.githubusercontent.com/113989589/206501339-a713f027-d694-44d4-a15f-d5894bce0ae1.png" width="70%">  

6. Note that in some cases, you may have to select 2D Pose Estimate in the tab and drag the actual position of the vehicle.
<img src="https://user-images.githubusercontent.com/113989589/206501742-a9b8cd85-9ad2-49a3-af52-a67b45e66c17.png" width="70%">  

7. Select 2D Goal Pose in the tab and specify the goal position by dragging.
<img src="https://user-images.githubusercontent.com/113989589/206502195-42aa0b92-928e-4759-8b25-f58a7a99680b.png" width="70%">  

8. You can see that the route is displayed and "WAITING FOR ENGAGE" status as shown below (it can take several minutes to run):
<img src="https://user-images.githubusercontent.com/113989589/206502874-6bd0e54e-0b04-45b5-a1f6-a83605a6c972.png" width="70%">  

9. Press Engage button, you can see that self-driving started.
<img src="https://user-images.githubusercontent.com/113989589/206503383-cd28fb0c-2553-45e6-bf98-b9b1d1412991.png" width="70%">  

## Time Measurement
Please refer to [RULE_en.md](/RULE_en.md) for the time acquisition method.

## Others
### Notification of updates
When there are updates on GitHub, we will make a new comment on the issue at the following URL. Please SUBSCRIBE to this issue to be notified of updates (please turn on notifications)https://github.com/AutomotiveAIChallenge/aichallenge2022-sim/issues/1. 

### Contact 
If you have questions about the competition or repository contents, please submit an issue on GitHub. You can ask questions in either English or Japanese.   

Questions must be directly related to the competition. We will not be able to answer questions regarding the use of the software. 

Please close an issue when resolved.   
  
We generally reply to questions within two business days. Please note that depending on the contents of your questions, it may take longer than two business days to answer. 

   

For inquiries regarding an account of an online simulator, for example, if you cannot log in to the online simulator, please contact us. 

email：info-ai@jsae.or.jp

