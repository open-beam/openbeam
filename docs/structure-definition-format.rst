.. _yaml_definition:

Problem definition file format
======================================

OpenBeam reads problem definitions from a YAML file, with the format explained in this page.
You can also directly jump to see the `YAML example files <https://github.com/open-beam/openbeam/tree/master/examples-structures>`_
or test and edit them in the `online app <ob-solver-simple/>`_.

Then, come back to this page when in doubt about how to define a particular
element, load, or parameter.

.. dropdown:: YAML format summary
    :open:

    You can learn more on the YAML format in its `Wikipedia article <https://en.wikipedia.org/wiki/YAML>`_
    or directly in its `official site <https://yaml.org/>`_. Here we provide a quick cheatsheet only:

    **Comments**: A ``#`` means that the rest of the line is a comment, so it is ignored by the YAML parser.

    **Maps** or **Dictionaries**: It is the main data structure used in the OpenBeam problem definition files.
    It defines a map between ``keys => values`` with this format:

    .. code-block:: yaml

        dictionary_name:
          key1: 100e34
          key2: 2.0
          key3: 'a text can be defined like this'
          key4: "or like this too"
          key5: also_a_text_string

    The short JSON-like version works too:

    .. code-block:: yaml

        dictionary_name: {key1: 100e34, key2: foo}

    **IMPORTANT**: The space after the colon ``:`` is mandatory.


1. Parameters
---------------

This **optional** section allows you to define variables or 
parameters, to easily parameterize a structure dimensions,
loads, etc.
Its format is just a YAML map, which each key being converted
into a parameter. 

.. note::

    Mathematical expressions and formulas can be used in all entries,
    including the parameters that have been defined
    formerly. This is an extension of YAML implemented in OpenBeam, 
    making use of `mrpt::expr::CRuntimeCompiledExpression <https://docs.mrpt.org/reference/latest/class_mrpt_expr_CRuntimeCompiledExpression.html>`_
    which in turn uses the `exprtk language <https://github.com/ArashPartow/exprtk#readme>`_.


Example:

.. code-block:: yaml

    # -------------------------------------------------
    #  Parameters
    # -------------------------------------------------
    parameters:
      G: 9.81
      L: 1         # Dimensions of the problem
      P: G * 1000  # External load value

The YAML short format is also supported:

.. code-block:: yaml

    parameters: { G: 9.81, L: 1, P: G * 1000 }


2. Beam sections
-------------------

This **optional** section allows you to define short names for beam
sections that will be used in more than one element.
A mandatory ``name`` key must be defined, along with the material and 
beam section geometry parameters that will be needed by the particular
finite element.

Example:

.. code-block:: yaml

    # -------------------------------------------------
    # beam sections
    # -------------------------------------------------
    beam_sections:
    - name: IPE200
      E: 2.1e11     # Young module
      A: 28.5e-4    # Area
      Iz: 1940e-8   # Second moment of area in z

3. Geometry: nodes
--------------------

Each node must be assigned a unique ``id`` number, coordinates (in 2D or 3D), 
and an **optinal label**, for easy identification of node names in diagrams
and animations.
In case of tilted planes using nodal coordinates, additional entries ``rot_x``,
``rot_y`` and ``rot_z`` may be added to especify the roll, pitch, and yaw
rotation angles **in degrees**, respectively. Planar structures only need to 
specify ``rot_z``.

Example:

.. code-block:: yaml

    nodes:
    - {id: 0, coords: [0   , 0], label: A}
    - {id: 1, coords: [2*L , 0], label: B}
    - {id: 2, coords: [3*L , 0], label: C, rot_z: 45}


4. Geometry: elements
-----------------------

This section defines how many finite elements exist,
to which nodes they are attached, and their type.

For the full list of finite elements implemented in 
this library, along with their ``type`` names and 
mathematical definitions, see :ref:`finite_elements`.

Example:

.. code-block:: yaml

    nodes:
      - {type: BEAM2D_RA, nodes: [0, 1], section: IPE200}
      - {type: BEAM2D_AR, nodes: [1, 2], section: IPE200}


5. Geometry: constraints
--------------------------

This sections lists all the degrees-of-freedom that are 
constrained such that their displacement is zero (or any other fixed value).

Each YAML map entry must define the node ``id`` number
(as defined in the `node section above <#geometry-nodes>`_)
and **which DoF are constrained**, using these names:

* ``DX``, ``DY`` or ``DZ``: Translation in one axis only (X,Y,Z).
* ``RX``, ``RY`` or ``RZ``: Rotation along one axis only (X,Y,Z).
* ``DXDYDZ``: All translations are constrained.
* ``RXRYRZ``: All rotations are constrained.
* ``DXDY``, ``DXDZ``, ``DYDZ`` : Translation in two axis is constrained.
* ``DXDYRZ``: A very common case in 2D structures: translation in X and Y, and rotation in Z are constrained.
* ``DXRZ``: Translation in X and rotation in Z are constrained.
* ``DYRZ``: Translation in Y and rotation in Z are constrained.
* ``DXDYRXRZ``: Translations in X,Y and rotations in X and Z are constrained (for beams with torsion loads).
* ``ALL`` or ``DXDYDZRXRYRZ``: All 6 DoFs are constrained.

.. note::

    You do not need to specify constraints in DoFs that are not used
    by your finite elements. For example, to fix a 2D rod element to
    ground, you do not need to explicitly specify that the rotation DoFs
    are zero, since the library will automatically discard the unused DoFs.
    Though, it is not an error to overspecify those constraints, only a 
    warning will be generated.

Example:

.. code-block:: yaml

    # Constraints
    constraints:
      - {node: 0, dof: DXDYRZ}
      - {node: 3, dof: DXDYRZ}
      #- {node: 2, dof: DY, value: 1e-3} # optional value different than zero.


6. Loads on nodes
----------------------

This sections allows the definition of concentrated forces or torques
at particular DoFs of nodes. 
Each entry must define these keys:

* ``node``: The node ``id`` as defined in the `node section above <#geometry-nodes>`_.
* ``dof``: The DoF on which the load is defined. Any of: 

  * ``DX``, ``DY``, ``DZ``: for forces. Positive values are in the direction of the axes.
  * ``RX`` (torsion), ``RY`` (bending), ``RZ``  (bending): for torques.

Example:

.. code-block:: yaml

    # Loads:
    node_loads:
      - {node: 2, dof: DX, value: -P}


7. Distributed loads on the elements
---------------------------------------

Loads which are distributed along the elements.

The element ``id`` number is the 0-based index of the elements
as defined in the ``elements`` section above.

Implemented load ``type`` are:

* ``TEMPERATURE``: Temperature increment load. Parameters:

  * ``deltaT``: Temperature increment (in Celsius degrees).
  * NOTE: Temperature coefficient is right now fixed to ``12e-6``.

* ``DISTRIB_UNIFORM``: Uniformly distributed load. Parameters:

  * ``q``: Load density value.
  * ``DX``, ``DY`` (and optionally, ``DZ``): They must form a unit vector specifying the load direction.

* ``TRIANGULAR``: Triangular or trapezoidal load.  Parameters:

  * ``q_ini``, ``q_end``: Load density values at the first and second element nodes.
  * ``DX``, ``DY`` (and optionally, ``DZ``): They must form a unit vector specifying the load direction.


* ``CONCENTRATED``: Concentrated load at a particular point amid the element.  Parameters:

  * ``p``: Concentrated load value.
  * ``DX``, ``DY`` (and optionally, ``DZ``): They must form a unit vector specifying the load direction.
  * ``dist``: Distance from the first element node.


Example:

.. code-block:: yaml

    element_loads:
      - {element: 0, type: DISTRIB_UNIFORM, q: 2000*G, DX: 0, DY: -1, DZ: 0}

