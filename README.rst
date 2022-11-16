.. image:: docs/images/_static/logo_with_text_600px.png
   :alt:

.. image:: images/_static/logo_with_text_600px.png
   :alt:

Roslabware is a simple Python library to provide an interface between ROS and 
the drivers of various pieces of equiment tipically used in a chemistry laboratory.

Features
--------

* Provides a consistent set of commands for every device type. I.e. every
  hotplate is obliged to have methods :py:meth:`set_temperature` and
  :py:meth:`get_temperature` irrespective of the exact manufacturer's protocol /
  command names for that device.

* Provides an abstract device type hierarchy.

* Supports multiple transports to physically communicate with device:

    * Standard serial connection with `PySerial <https://pythonhosted.org/pyserial/>`_.
    * Standard TCP/UDP over IP connection with Python :py:mod:`sockets`.
    * Connection to devices exposing HTTP REST API with Python `Requests <https://requests.readthedocs.io/en/master/>`_.

* Provides an easy way to add new devices with Python dictionary-based command set definitions (see :ref:`add_new_device`).

* Can run arbitrary tasks for multiple devices in parallel (see :ref:`tasks`).

* Provides full simulation mode to test any action sequences without physical
  devices present (see :ref:`simulation`).


Minimal requirements
---------------------

* Python 3.8 or newer.
* PySerial package for serial communications.
* Requests package for HTP REST API devices.
* PyYAML package for using OpenAPI parser tool (see :doc:`utils`).

Platforms
---------

The library has been developed and tested on Windows and Linux.


Documentation
-------------

The current documentation with examples can be found `here <https://link_to_docs>`_.


Installation
------------

Installing from pip:

>>> pip install PyLabware

.. rubric:: Footnotes
.. [1] More details can be found in the `respective paper <https://doi.org/10.1126/science.aav2211>`_.
