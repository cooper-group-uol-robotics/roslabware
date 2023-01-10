Usage
=====

The easiest way to launch the package is with roslaunch:

.. code-block:: shell

   roslaunch kern_pcb_top_balance kern_pcb_top_balance.launch port:=<port>

Alternatively, can be launched using rosrun:

.. code-block:: shell

   rosrun kern_pcb_top_balance kern_pcb_top_balance <port>

This will launch the driver assuming the device (kern_balance) is connected to the provided serial port.

If there no arguments provided, it will use the default values:

The parameteres needed are:

* ``device_name`` is an arbitrary string used to identify a device. Default to ``<manufacturer>_<device_model>``.
* ``port`` is a serial port name (platform-dependent). Default to ``/dev/ttyUSB0``
* ``connection_mode`` determines which connection would be activated for the device. Defaults to ``serial``
* ``address`` determines IP address/DNS name for socket-based or HTTP REST connection, it is not used for serial connection. Defaults to ``None``


ROS Topics
-----------

To trigger commands, the following fomart is followed:

.. code-block:: shell

   rostopic pub -1 / <manufacturer>_<device_model>_Commands <manufacturer>_<device_model>/<ManufacturerDeviceModel>Command "<manufacturer>_<device_model>: "command" 

For example for the kern_pcb_top_balance:   

.. code-block:: shell

   rostopic pub -1 / Kern_Commands kern_pcb_balance/KernCommand "<manufacturer>_<device_model>_command: 0" 

