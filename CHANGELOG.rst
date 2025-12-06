^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package event_camera_renderer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.0.0 (2025-12-06)
------------------
* updated license string in package.xml
* better error message when no events are received
* need include for queue
* removed remaining ROS1 stuff (except for ros_compat)
* handle arbitrarily fine time slices now
* eventExtTrigger() returns true now
* This pull request resolves a build failure caused by an invalid-constexpr error on modern compilers (like AppleClang).
* Contributors: Bernd Pfrommer, Dhruv Patel

2.0.2 (2025-09-01)
------------------
* updated workflow
* support new image transport node interface
* Contributors: Bernd Pfrommer

2.0.1 (2025-07-29)
------------------
* remove old ROS2 from CI
* remove ROS1 from CI
* support new transport api
* bumped cmake required
* Contributors: Bernd Pfrommer

2.0.0 (2025-05-22)
------------------

1.0.4 (2024-05-29)
------------------
* fix linter errors on noble
* updated build instructions
* Contributors: Bernd Pfrommer

1.0.3 (2024-02-04)
------------------
* permit variable sensor size
* change formatting of python files to make flake8 linter happy
* ignore pyc files
* Contributors: Bernd Pfrommer

1.0.2 (2023-09-21)
------------------
* added dependency on ament_cmake_clang_format
* Contributors: Bernd Pfrommer

1.0.1 (2023-09-19)
------------------
* Initial release
* Contributors: Bernd Pfrommer
