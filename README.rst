========
Roslabware
========

Roslabware is a simple Python library to provide an interface between ROS and
pylabware for various pieces of equiment tipically used in a chemistry laboratory.

Features
--------

* Provides higher level commands for normal action executions in the chemistry laboratory

* Provides an easy way to add new devices with Python dictionary-based command set definitions (see :ref:`add_new_device`).

Minimal requirements
---------------------

* Python 3.8 or newer.
* `Pylabware <https://github.com/cooper-group-uol-robotics/pylabware>`_
* ROS (noetic)

Platforms
---------

The library has been developed and tested on Windows and Linux.

Documentation
-------------

The current documentation with examples can be found `here <./docs>`_.

Installation
------------

Clone the repo into your ROS workspace/src folder

.. code-block:: shell

    git clone https://github.com/cooper-group-uol-robotics/roslabware

Install requirements (mind your Python enviroment)

.. code-block:: shell

    pip install -r requirements.txt

Build your messages from your workspace parent folder

.. code-block:: shell

    catkin_make
    source ./dev/setup.sh
