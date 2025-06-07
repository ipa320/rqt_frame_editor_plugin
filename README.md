frame editor rqt-plugin
=========================================

* This repository contains the **frame editor rqt-plugin.**

* Frame-editor helps you **creating and arranging tf-frames.** 

* You can arrange your frames by **copying full poses** or selected positions or orientations **from other existing tf-frames**. Of course you can **manually type-in** your wanted values or even **use an interactive marker within rviz** to arrange your frame.

* Additionally you may **attach a mesh to your frame**. The mesh is also visible within rviz (by published marker topic). The frame-editor provides some basic shapes or you may use your own stl file.

* **Save your configuration and load it** again.

* Most features are **also available via ros services**, if you want to use the functions but not the interface.

![The rqt plugin frame_editor](/etc/img/rqt_frame_editor.png "The rqt plugin frame_editor")

* Feature: Moving frames

![Moving Frames](/etc/img/moving_frames.gif "Feature: Moving frames")


* Feature: Add Frames and Change their respective parents

![Parent Frames](/etc/img/parent_frames.gif "Feature: Parent frames")

* Feature: Filter and Delete Frames

![Filter and Delete Frames](/etc/img/filter_delete_frames.gif "Feature: Filter and Delete Frames")


![The rviz view](/etc/img/rviz.png "Using an interactive marker to manipulated a tf-frame")


### Additional features: 
* Group frames: You can group frames by adding a "group" entry in the yaml file. Frames with the same "group" entry will grouped.

### Installation:
#### Clone this repository into your colcon workspace.

``` 
git clone https://github.com/ipa320/rqt_frame_editor_plugin.git
```

#### Update your colcon workspace, e.g.

``` 
colcon build --symlink-install
```

#### Refresh rqt

``` 
rqt --force-discover
```

### Usage:
#### You may just start rqt and select the frame-editor plugin.

``` 
rqt
```

![The rqt plugin frame_editor](/etc/img/rqt_frame_editor_exp.png "The rqt plugin frame_editor with remarks")


#### To be able to use the interactive marker to move frames around you have to add the InteractiveMarkers-plugin to rviz and select the topic '/frame_editor_interactive/update'
* The frame to be manipulated must be selected in the frame-editor.

#### To see the configured shapes you have to add the Marker plugin in rviz and select the topic '/frame_editor_marker'
* Shapes are not published too often and it may take some seconds for your shape to appear. However, appeared once it will be attached to its tf-frame and not lag behind if the frame moves.

#### You can also run it standalone, look into the given launch file. 

``` 
ros2 launch frame_editor frame_editor_launch.py
```

### Known issues: 
#### Starting the plugin twice 
When starting the rqt plugin twice, you will receive a long error message with these last lines: 

``` 
  [...] 
  File "/opt/ros/indigo/lib/python2.7/dist-packages/rospy/service.py", line 116, in register 
    raise ServiceException(err) 
ServiceException: service [/align_frame] already registered 
``` 

* Note that rqt automatically starts in the last configuration, so **the plugin may already be running, possibly minimized**. You can check the "Running" entry in the header menu to see whether the plugin is already active. 
* Hidden plugins can be recovered by rightclicking in the empty space and selecting the plugin (so far only called "Form") 
* Closing and starting the plugin doesn't seem to work, so: 
* The best way is to restart rqt.  
* If the plugin is not found in rqt, try 'rqt --force-discover' to make rqt search for new plugins 
* You can reset your rqt configuration, try 'rqt --clear-config' to make rqt reset saved configurations

#### Cannot use the interactive marker when its behind a semitransparent mesh.
Rviz is not giving your mouse event to the interactive marker. Disable the marker plugin and the you can touch the interactive marker.

### Acknowledgements:

Updates from 2024 on (including ROS2 port) are part of RIG (Robotics Institute Germany) project.

| Logo | Project | Runtime |
|------|---------|---------|
| <img src="/frame_editor/etc/img/RIG.jpg" alt="RIG" width="50"/> | <a href="https://robotics-institute-germany.de/">Robotics Institute Germany</a> | 01.07.2024 – 30.06.2028 |
| <img src="http://www.project-leanautomation.eu/fileadmin/img/LIAALogo/Logo_LIAA.png" alt="LIAA" width="100"/> | <a href="http://www.project-leanautomation.eu/">Lean Automation (LIAA)</a><br/>Grant: 608604 | 02.09.2013 – 31.08.2017 |
| <img src="http://www.project-leanautomation.eu/typo3temp/pics/b3ba71db31.jpg" alt="European Commission" width="100"/> | European Commission | — |
