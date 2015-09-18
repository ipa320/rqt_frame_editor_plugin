wp06_multimodal_robot_programming_toolbox
=========================================

This repository contains the Augmented Reality enhanced multimodal robot programming toolbox and related programs.


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
