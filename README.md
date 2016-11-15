frame editor rqt-plugin
=========================================

This repository contains the frame editor rqt-plugin


### Installation:
* Clone this repository into your catkin workspace.
    git clone https://github.com/ipa-lth/rqt_frame_editor_plugin.git
* Update your catkin workspace
    catkin_make
* Refresh rqt
    rqt --force-discover

### Start:
* You may just start rqt and select the frame-editor plugin.
* You can also run it standalone, try this:
    roslaunch frame_editor frame_editor.launch

### Known issues: 
#### Starting the plugin twice 
When starting the rqt plugin twice, you will receive a long error message with these last lines: 
+
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

### Acknowledgements:
This project is a result of the LIAA project.

LIAA received funding from the European Union’s Seventh Framework Programme for research, technological development and demonstration under grant agreement no. 608604.

Project runtime: 02.09.2013 – 31.08.2017.

http://www.project-leanautomation.eu/
