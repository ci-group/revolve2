=========================
Creating a physical robot
=========================
This tutorial shows you how to create a physical robot from existing parts.
It assumes you understand the concept of a modular robot and its modules.
The guide is only for V2 modular robots. V1 modular robots are legacy and only to be used within CI Group. If you are from that group, ask around.

--------------
Required parts
--------------

The core module
===============
A working core module is required. It looks like <<<photo>>>
Ask around your research group if a core is readily available.
If you are a maintainer of your research group, :ref:`take a look at the guide on how to create a core module<physical_robot_core_setup/index:Setting up a new modular robot core>`.

Brick and active hinge modules
==============================
They look like <<<photo>>>

Ask around your research group if a brick and hinge modules are readily available.
If not, see :ref:`creating_a_physical_robot/index:3D printing and assembling modules`.

Screws for attaching modules
============================
They look like <<<photo, specs>>>

--------------------------
Assembling a modular robot
--------------------------
Modules are connect by screwed them together. See the following photos as examples:
<<<photos>>>

----------------------------------
3D printing and assembling modules
----------------------------------
Bricks and active hinges can be 3D printed using the files at `<https://github.com/ci-group/revolve-models/tree/master/v2>`_.
To assemble an active hinge you will need screws.
<<<photos and screw specs>>>
See following photos on how to assemble an active hinge.
<<<photos>>>

-------------------------
Using your physical robot
-------------------------
The `physical_robot_remote` :ref:`example<getting_started/index:Step 2: Play with the examples>` will show you how to control your newly built physical robot.
