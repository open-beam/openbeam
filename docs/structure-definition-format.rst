.. _yaml_definition:

Problem definition file format
======================================

OpenBeam reads problem definitions from a YAML file, with the format explained in this page.
You can also directly jump to see the `YAML example files <https://github.com/open-beam/openbeam/tree/master/examples-structures>`_
or test and edit them in the `online app <ob-solver-simple/>`_.

Then, come back to this page when in doubt about how to define a particular
element, load, or parameter.

1. Parameters
---------------

This **optional** section allows you to define variables or 
parameters, to easily parameterize a structure dimensions,
loads, etc.
Its format is just a YAML map, which each key being converted
into a parameter. 
Note that you can use mathematical expressions and formulas
in all entries, including the parameters that have been defined
formerly.

Any line starting with ``#`` is a comment.

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

Example:

.. code-block:: yaml

    nodes:
    - {id: 0, coords: [0   , 0], label: A}
    - {id: 1, coords: [2*L , 0], label: B}


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


