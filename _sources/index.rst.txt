.. mvsim documentation master file, created by
   sphinx-quickstart on Sat May 30 11:36:10 2020.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

OpenBeam project
=====================

.. toctree::
   :maxdepth: 3
   :hidden:

   Home <self>
   Online solver <ob-solver-simple/#http://>
   compile
   structure-definition-format
   finite_elements
   C++ API <doxygen/html/classes.html#http://>

.. (JLBC note: Do not remove the #http:// above, it's the only way I found to allow that link to be included in the TOC).

This project includes a C++ library, CLI tools, and web applications.

Use the menu on the left to navigate through the documentation and the on-line apps:

* `ob-solver <ob-solver-simple/>`_: Main **online app** to solve static analysis of any structure,
  defined by means of the `YAML problem definition file <structure-definition-format.html>`_.


Demo video
==================

.. image:: imgs/openbeam-demo.gif
  :width: 650
  :alt: Animation of deformed structure

Github repository: https://github.com/open-beam/openbeam

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

License
==================

Source code is licensed under the GNU GPL-3 License, contact authors for other options.
