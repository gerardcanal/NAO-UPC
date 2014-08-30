NAO-UPC
=======

Repository for nao code development. The goal is to develop 3 tests:

* Put off the fire: the robot must check which kitchen fire is lit, and put it off by pressing the corresponding button.

* Shopping list: the robot must recognize which products (from a pre-defined set) are available on the shelves, and which other products are missing, and make a shopping-list.

For this task OpenCV 3.0 is needed to compute AKAZE features.
https://github.com/Itseez/opencv

ROS do not support OpenCV 3.0, so you need a modified version of vision_opencv(cv_bridge) and image_pipeline.
https://github.com/mikejmills/vision_opencv
https://github.com/edgarriba/image_pipeline

With this new repositories, you must uninstall this precompiled respositories:
sudo apt-get autoremove ros-indigo-cv-bridge ros-indigo-image-pipeline

Probably, with this command rqt_image_view will be uninstalled or simply won't work. Until now, I haven't found the public code to modify it.

* Meal preparation: the robot must grasp a tomato and put it into a cooking pan.
