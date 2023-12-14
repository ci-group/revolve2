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

.. image:: ../core.png
  :width: 400
  :alt: A core module

A working core module is required.
Ask around your research group if a core is readily available.
If you are a maintainer of your research group, :ref:`take a look at the guide on how to create a core module<physical_robot_core_setup/index:Setting up a new modular robot core>`.

Brick and active hinge modules
==============================

.. image:: ../brick.png
   :width: 45%
.. image:: ../active_hinge.png
   :width: 45%

Ask around your research group if brick and hinge modules are readily available.
If not, see :ref:`creating_a_physical_robot/index:3D printing and assembling modules`.

Bolts for attaching modules
============================

.. image:: ./bolts.png
  :width: 400
  :alt: The bolts used for connecting modules

There are two types of bolts. Both are Torx 10.
The bolts connecting modules to the core are 8mm long, whereas the bolts connecting two non-core modules are 20mm long and require matching nuts.

--------------------------
Assembling a modular robot
--------------------------

.. image:: ./bolt_together_1.png
  :width: 45%
  :alt: Example of modules bolted together
.. image:: ./bolt_together_2.png
  :width: 45%
  :alt: Example of a connected active hinge

Modules are connect by bolting them together. See the photo above as an example.
Note that when connecting to the core nuts are not required as thread is embedded in the core.
Route the wires from the active hinges into the core and plug them in as displayed on the photo.
Make sure the ground (dark) wire is oriented correctly.

.. image:: ./active_hinge_to_core.png
  :width: 45%
  :alt: How to connect an active hinge to the core

----------------------------------
3D printing and assembling modules
----------------------------------
Bricks and active hinges can be 3D printed using the files at `<https://github.com/ci-group/revolve-models/tree/master/v2>`_.
If you are working in a research group that is already using Revolve2, this step is probably not necessary and modules are already available.

If you arrive at this step it is likely that you also do not have servos and you may have to order those.
Exact specs are not available, but keep in mind that you need to be able to read the current position of the servo (or be able to physically alter the servo to provide that, which is often possible).
Of course, keep in mind the form factor and possibly adjust the 3d model before printing.
To assemble an active hinge you will need screws. These will either come with the bought servo, or you have to find matching ones.

.. image:: ./assemble_servo_1.png
   :width: 17%
.. image:: ./assemble_servo_2.png
   :width: 17%
.. image:: ./assemble_servo_3.png
   :width: 17%
.. image:: ./assemble_servo_4.png
   :width: 17%
.. image:: ./assemble_servo_5.png
   :width: 17%

-------------------------
Using your physical robot
-------------------------
The `physical_robot_remote` :ref:`example<getting_started/index:Step 2: Play with the examples>` will show you how to control your newly built physical robot.
