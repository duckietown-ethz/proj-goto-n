![alt text](https://github.com/duckietown-ethz/proj-goto-n/blob/master/header.png)
#### Contributors: Alexander Hatteland, Marc-Philippe Frey & Demetris Chrysostomou ####
##### Mentors: Amaury Camus, Tomasz Zaluska #####
##### Supervisor: Jacopo Tani #####

# 1. Goto-N Project: #
The video below shows the successful completion of the Goto-N Project Pipeline for two duckiebots in an autolab. 

INSERT 2_bot_completion_ex1.mov from GoogleDrive

We have also added an additional short video of the visual representation of the planner. The video outlines the Autobot's generated path in RVIZ to be inspected visually. Furthermore, it also showcases an instance of the planner's ability to replan when it detects a mistake in it's path. It is important to note that the video below is a seperate instance of the Goto-N node and not related to the video above. 

INSERT replanning_screenrecording.mov from GoogleDrive


# 2. Duckietown Set-up: #
To run the Goto-N pipeline, the user has to setup an operational duckietown according to the specifications found [here](https://docs.duckietown.org/daffy/opmanual_duckietown/out/index.html). Furthermore, the package requires access to a localization system in order to ensure that the duckiebots are localizable. Consequently, the duckietown has to be configured as an autolab according to the autolab specifications found [here](https://docs.duckietown.org/daffy/opmanual_autolab/out/index.html). Once the duckietown is setup as a functioning autlab, with watchtowers and april tags placed correctly and the localization system configured, the user can proceed to the duckiebot and server setup. 

# 3. Duckiebot Set-up: #
The entire project is based on the "daffy" (develop) configuration of Duckietown. It is built around the dt-core pipeline and comminicating with the autolab server. It is important that this is only tried in autolabs, where the watchtowers and localization system is properly set up.

### Setting up the framework on each Autobot: ###
To test this project, the Duckiebot needs to be updated to Autobots. You do this by following [this](https://docs.duckietown.org/daffy/opmanual_autolab/out/autolab_autobot_specs.html).

Ensure that the Autobot has the latest **dt-core**, **dt-car-interface** and **dt-duckiebot-interface**:

```
$ docker -H DUCKIEBOT_NAME.local pull duckietown/dt-car-interface:daffy

$ docker -H DUCKIEBOT_NAME.local pull duckietown/dt-duckiebot-interface:daffy

$ docker -H DUCKIEBOT_NAME.local pull duckietown/dt-core:daffy
```

First, make sure all old containers from the images dt-duckiebot-interface, dt-car-interface, and dt-core are stopped. These containers can have different names, instead look at the image name from which they are run.

Then, start all the drivers in **dt-duckiebot-interface**:
```
$ dts duckiebot demo --demo_name all_drivers --duckiebot_name DUCKIEBOT_NAME --package_name duckiebot_interface --image duckietown/dt-duckiebot-interface:daffy
```

Also start the “glue” nodes that handle the joystick mapping and the kinematics, the **dt-car-interface**:
```
$ dts duckiebot demo --demo_name all --duckiebot_name DUCKIEBOT_NAME --package_name car_interface --image duckietown/dt-car-interface:daffy
```

Indefinite navigation needs to running. The version of indefinite navigation used in this demo is without the random random_april_tags_turn_node. Hence, one needs to edit the current version to disable this node from the pipeline. A way of doing that easily will be to first go into the container of **dt-core**:

```
$ docker -H DUCKIEBOT_NAME.local run -it --name goton-dt-core -v /data:/data --privileged --net host duckietown/dt-core:daffy /bin/bash
```

Once inside the root of the container, one needs to navigate to the location of the launchfile that needs to be edit (packages/duckietown_demos/launch). The file that needs to be edited is the indefinite_navigation.launch. Do this by installing a text editor (e.g. vim):
```
$ cd packages/duckietown_demos/launch
$ apt-get update
$ apt-get install vim
$ vim indefinite_navigation.launch
```
(You edit a file in vim using insert “i”, and save by writing “:wq”)
Inide indefinite_navigation.launch turn the flag for apriltag_random from “true” to “false”.  Our node will decide where the autobot goes at each intersection, and therefore it is crucial that it is turned to false.

If this setup has been done before, and you want to start it again. Start the container and then attach yourself to the docker container using:
```
$ docker -H DUCKIEBOT_NAME.local attach goton-dt-core
```

Launch the **indefinite navigation** nodes using:
```
$ roslaunch duckietown_demos indefinite_navigation.launch veh:=DUCKIEBOT_NAME
```

Next, one needs to build the **goto-n version** of the **acqusition bridge** on the Autobot. In order to use the watchtowers as guidance, an updated version of the acqusition brige must be on the Autobot. Clone the acqustion brige from this repository:
```
git clone https://github.com/alexushatten/acquisition-bridge
```
Once it is cloned, build it on the Autobot.
Remember to stop the watchtower when building new images on the Autobot:
```
$ dts devel watchtower stop -H DUCKIEBOT_NAME.local
```
Once that is done, build the **acquisition bridge** on each Autobot individually by running the command:
```
$ dts devel build -f --arch arm32v7 -H DUCKIEBOT_NAME.local
```

Finally, the **goto-n-duckiebot node** needs to be built and started. This is the node that will get the planning commands from the server and ensure that the robot takes the right decision at each intersection. First clone the goto-n-duckiebot repositiory:
```
$ git clone https://github.com/alexushatten/goto_n_duckiebot
```

Then, build the image on the Autobot:
```
$ dts devel build -f --arch arm32v7 -H DUCKIEBOT_NAME.local
```
# 4. Server Setup
### Setting up the framework on the server: ####
First, ensure that the Autolab is properly initalized. Instructions on how to do that is found [here](https://docs.duckietown.org/daffy/opmanual_autolab/out/autolab_minimal_requirements.html).
When the autolab is properly set up, start the c-slam localization. It is possible to do that by:
```
$ docker run --rm -e ATMSGS_BAG=/data/processed_BAG_NAME.BAG -e OUTPUT_DIR=/data ROS_MASTER=YOUR_HOSTNAME -e ROS_MASTER_IP=YOUR_IP --name graph_optimizer -v PATH_TO_BAG_FOLDER:/data -e DUCKIETOWN_WORLD_FORK=YOUR_FORK_NAME -e MAP_NAME=YOUR_MAP_NAME duckietown/cslam-graphoptimizer:daffy-amd64
```

Once the localization system is up and running, clone the goto-n node to the server:
```
$ git clone https://github.com/alexushatten/goto_n
```

This node will do the planning, as well as the closed-loop precision for each autobot during the driving. 
Build the image by using:
```
$ dts devel build -f –arch amd64
```

# 5. Pre-Flight Checklist: #
* The Autolab is setup according to section  *2.0 Duckietown Setup*, with watchtowers and april tags correctly placed.
* The Autobots are setup according to the autolab definition. 
* The yaml file of the map contains all the relevant information including tile information, tile size and april tag locations.
* The Autobot and Termination Position ymal files are updated to the desired parameters.
* The latest version of **dt-core**, **dt-car-interface** and **dt-duckiebot-interface** has been pulled.
* The latest version of the **acquisition-bridge** is pulled onto each Autobot and the **dt-core** packages has been overriden to the goto-n **dt-core-edit** package.
* The localization system is started up and running. RVIZ is also open to visualize the created plan. 
* The Autobots are running the goto_n_duckiebot node, built from the required repositories. 

# 6. Demo Instructions: #
Once the setup instructions for the Autolab, Autobot and server have been executed, it is possible to start the demo. Make sure that the watchtower localization system is up and running. Furthermore, make sure that the relevant information if fed into the Goto-N server node such as the Autobot list and desired termination positions. Lastly, also make sure that the Autobots are present in the Autolab and meet our pre-defined assumptions. 

Begin by start the **acqusition bridge** for each Autobot using (keep in mind that each Autobot requires a seperate terminal):
```
docker -H HOSTNAME.local run --name goto-n-acquisition-bridge --network=host -v /data:/data -e LAB_ROS_MASTER_IP=YOUR_LAB_ROS_MASTER_IP -dit duckietown/acquisition-bridge:daffy-arm32v7
```

Once the **acquisition bridge** is started, open a new terminal for each Autobot and run the **goto_n_duckiebot** node opn each Autobot individually using the command:
```
$ docker -H DUCKIEBOT_NAME.local run -it –privileged --rm --network=host -v /data:/data duckietown/goto_n_duckiebot:v1-arm32v7
```
The Autobot is now ready to receive commands from the server on where to go.

Finally, start the server node in another seperate terminal using the command:
```
$ docker run -it –-rm –net host duckietown/goto_n:v1-amd64
```
Once the system is running, it will send waypoint commands to all the Autobots defined in the duckiebot.yaml file specififed in the config folder (remember to have the same amout of termination states as Autobots defined).


### Additional: ###
It is possible to see the output of the planning through Rviz. The topic _/autobotXX/planning_vizualisation_ will show the path of each Autobot. Remember to overlay the map.

# 7. Configuration parameters: #
All the parameters are in the server-node of goto-n found [here](https://github.com/alexushatten/goto_n/tree/6a7c8d0dd7a4f0b0c2ec46f4239e2c00278b835a/packages/goto_n/config).
### duckiebots.yaml ###
Here it is important that the Autobot number (e.g. 21) is defined for all the Autobots that are in the system. The reason this is done is to be able to test the system while other groups are doing their tests. This is so that the planner will not get affected by other present duckiebots.
### termination_positions.yaml ###
There needs to be one termination position for each autobot present in the city. Otherwise it will not be possible to drive all the bots to a desired location. It is assumed that there are always equal number of termination positions and Autobots.
### watchtowers.yaml ###
All the watchtowers used in this tests needs to be defined here, so that the node sends the _request image_ message to all the watchtowers. This is done so that it is possible to locate autobots without any movement.
### map ###
The map used is found [here](https://github.com/alexushatten/goto_n/tree/v1/packages/goto_n/maps). Currently, edit the mapfile that is already there since this map is defined in the code already.
# Pipeline: #
![alt text](https://github.com/duckietown-ethz/proj-goto-n/blob/master/pipeline.png)


# 8. Troubleshooting: #
If experiencing poor driving buy the Autobots, make sure that the calibration (intrincic, extrincic and kinematic) are done properly, since the Autobot uses these parameters when driving.

Also, it is possible to tune different parameters when experiencing poor performance.
See the full parameter list by:
```
$ dts start_gui_tools DUCKIEBOT_NAME
$ rosparam list
$ rosparam get /directory/path/file DESIRED_VALUE
```
It is possible to tune independent parameters using:
```
$ docker run -it --rm  -e ROS_MASTER_URI="http://DUCKIEBOT_IP:11311/" duckietown/dt-ros-commons:daffy-amd64 /bin/bash
$ rosparam set /name/of/param DESIRED_VALUE
```

For trouble with the intersection navigation (i.e. the bot does not turn the way it is supposed to do). Follow the guidelines [here](https://docs.duckietown.org/daffy/opmanual_duckiebot/out/trouble_unicorn_intersection.html)

If the messages from the server are not being recieved on the Autobot, check if the correct version of acqusition bridge is being used, and that it is still running properly. The version in this repository are the only one which containes the messages that needs to be sent between the different ROS_MASTERs. Easiest way to ensure that it is set up correctly is to remove the container and building it again from scratch.

# 9. Demo Failure # 

# 10. Imrovements: #

# 11. Remarks: #
This code is only tested for ML k31 autolab enviornment.
The system is scaleable to as many robots as needed, but only tested for up to 3 bots.
Please note all videos have duckies present on the Autobot. However, as the April Tag had to be free from obstruction, it is possible that the duckies are not 


