.. _installing:

Installing
===========

Build from sources
----------------------

Dependencies:

- A decent C++17 compiler.
- CMake >= 3.1
- MRPT (>=2.4.0)

In Ubuntu, this will install all requirements:

.. code-block:: bash

 sudo apt install \
   build-essential cmake g++ \
   libmrpt-opengl-dev

Compile as usual:

.. code-block:: bash

 mkdir build
 cd build
 cmake ..
 make
 #make test

