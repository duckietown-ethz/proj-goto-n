![alt text](https://github.com/duckietown-ethz/proj-goto-n/blob/master/header.png)
#### Contributors: Alexander Hatteland, Marc-Philippe Frey & Demetris Chrysostomou ####

# Set-up: #
The entire project is based on the "daffy" (develop) configuration of Duckietown. It is built around the dt-core pipeline and comminicating with the autolab server. It is important that this is only tried in autolabs, where the watchtowers and localization system is properly set up.

### Setting up the framework on each Autobot: ###
To test this project, the Duckiebot needs to be updated to Autobots. You do this by following [this](https://docs.duckietown.org/daffy/opmanual_autolab/out/autolab_autobot_specs.html).

Ensure that the Autobot has the latest **dt-core**, **dr-car-interface** and **dt-duckiebot-interface**:

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


Finally, we need to start indefinite navigation. The version of indefinite navigation used in this demo is without the random random_april_tags_turn_node. Hence, one needs to edit the current version to disable this node from the pipeline. A way of doing that easily will be to first go into the container of **dt-core**:

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
docker -H DUCKIEBOT_NAME.local attach goton-dt-core
```

Launch the **indefinite navigation** nodes using:
```
roslaunch duckietown_demos indefinite_navigation.launch veh:=DUCKIEBOT_NAME
```

Next, one needs to build the **goto-n version** of the **acqusition bridge** on the Autobot. In order to use the watchtowers as guidance, an updated version of the acqusition brige must be on the Autobot. Clone the acqustion brige from this repository, and once it is cloned build it on the Autobot
Remember to stop the watchtower when building new images on the Autobot:
```
$ dts devel watchtower stop -H DUCKIEBOT_NAME.local
```
Once that is done:
```
$ dts devel build -f --arch arm32v7 -H DUCKIEBOT_NAME.local
```

Start the acqusition bridge using:
```
docker -H HOSTNAME.local run --name goto-n-acquisition-bridge --network=host -v /data:/data -e LAB_ROS_MASTER_IP=YOUR_LAB_ROS_MASTER_IP -dit duckietown/acquisition-bridge:daffy-arm32v7
```

Finally, start the **goto-n-duckiebot node**. This is the node that will get the planning commands from the server and ensure that the robot takes the right decision at each intersection. First clone the goto-n-duckiebot repositiory:

Then, build the image on the Autobot:
```
$ dts devel build -f --arch arm32v7 -H DUCKIEBOT_NAME.local
```

Start the goto-n-duckiebot node by:
```
docker -H DUCKIEBOT_NAME.local run -it –privileged --rm --network=host -v /data:/data duckietown/goto_n_duckiebot:v1-arm32v7
```
The Autobot is now ready to receive commands from the server on where to go.

### Setting up the framework on the server: ####
First, ensure that the Autolab is properly initalized. Instructions on how to do that is found [here] (https://docs.duckietown.org/daffy/opmanual_autolab/out/autolab_minimal_requirements.html)
When the autolab is properly set up, start the c-slam localization. It is possible to do that by:
```
$ docker run --rm -e ATMSGS_BAG=/data/processed_BAG_NAME.BAG -e OUTPUT_DIR=/data ROS_MASTER=YOUR_HOSTNAME -e ROS_MASTER_IP=YOUR_IP --name graph_optimizer -v PATH_TO_BAG_FOLDER:/data -e DUCKIETOWN_WORLD_FORK=YOUR_FORK_NAME -e MAP_NAME=YOUR_MAP_NAME duckietown/cslam-graphoptimizer:daffy-amd64
```

Once the localization system is up and running, clone the goto-n node to the server:



This node will do the planning, as well as the closed-loop precision for each autobot during the driving. 
Build the image by using:
```
dts devel build -f –arch amd64
```
Run the system by:
```
docker run -it –-rm –net host duckietown/goto_n:v1-amd64
```
Once the system is running, it will send waypoint commands to all the Autobots defined in the duckiebot.yaml file specififed in the config folder.


