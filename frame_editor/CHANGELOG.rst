^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package frame_editor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2022-03-15)
------------------
* Added argparse option to change publishing rate
* frame_editor is now compatible to Python 3 (ROS Noetic)
* Use the previous file name and parent frame as defaults for save_yaml and the set_frame service
* Contributors: Daniel Bargmann, Martin Günther, Philipp Tenbrock

1.0.4 (2019-01-31)
------------------
* Query if to add frames or update the whole list
* Fixed static initialisation of tf broadcaster
* Show current file name in label instead of window title
* Disabled menu
* Add load and save yaml service
* remember last opened folder
* Contributors: Lorenz Halt, ipa-lth, ipa-pgt

1.0.3 (2018.05.15)
------------------
* namespacing ROS services
* wait for transform for 1 sec
* Contributors: Witalij Siebert, ipa-frn

1.0.2 (2017-09-27)
------------------
* Fix: import rospkg
  Add rviz view to headless demo launch
* headless version
* during close, save_as proposes to save as current yaml
* added Duplicate Frame button
* Installing tags for frameeditor
* package.xml v2
* Ask for automatic mesh version path update if possible.
* fix legacy yaml loading, if you want to change to rospackages+path you have to reselect your mesh and save again
* Warning Widget if saved with absolute pathes
* resolve mesh pathes with rospackages if possible, fall back to absolute pathes if not possible.
* ugly qt5-slots workaround to make meshes and forms work
* Contributors: ipa-frn, ipa-lth, ipa-pgt
