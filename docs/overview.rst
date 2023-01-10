Overview
========

Roslabware is a simple Python library providing a common interface bewteen ROS
sets of commands to control various pieces of equipment which are typically
found in the chemical laboratory from a Python script.

Features
--------

* Provides a higher level abstraction for actions that call on pylabware functions


Minimal requirements
---------------------

* Rospy
* ROS noetic.

Platforms
---------

The library has been developed and tested on Windows and Linux. 

Documentation
-------------

The current documentation with examples can be found `here <./docs>`_.

Installation
------------

Nodes
-----

A list and description of susbcribed and public topics is `here </nodes.rst>`_. All the nodes available in this library are:

* fisher_pp14102
* ika_ret_control_visc
* kern_pcb2500
* mettler_quantos_qb1
* tecan_xlp6000

Launchers
---------

The launch files contain all the **default** arguments for a serial connection in the port ``/dev/ttyUSB0``. 

.. code-block:: html

   <launch>
      <arg 
         name="device_name"
         default="fisher_pp14102"/>
      <arg
         name="connection_mode"
         default="serial"/>
      <arg
         name="address"
         default="None"/>
      <arg
         name="port"
         default="/dev/ttyUSB0"/>
      <node 
         name="fisher_pp14102"
         pkg="fisher_pp14102"
         type="fisher_pp14102"
         args="
            $(arg device_name)
            $(arg connection_mode)
            $(arg address)
            $(arg port)"
         output="screen"/>
   </launch>


