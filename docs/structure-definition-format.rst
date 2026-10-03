.. _yaml_definition:

Problem definition file format
======================================

OpenBeam reads a structure (geometry, materials, supports and loads) from a YAML file,
with the format explained in this page.
You can also directly jump to see the `YAML example files <https://github.com/open-beam/openbeam/tree/develop/examples-structures>`_
or test and edit them in the `online app <ob-solver-simple/>`_.
A :ref:`complete example <yaml_complete_example>` is given at the end of this page.

.. contents:: Contents
   :local:
   :depth: 1


Overview
-----------

A problem file is a YAML map with these top-level sections, which are read in this order:

.. list-table::
   :header-rows: 1
   :widths: 20 12 68

   * - Section
     - Required
     - Contents
   * - ``parameters``
     - no
     - Named values to use in any other field (lengths, loads...).
   * - ``beam_sections``
     - yes (may be an empty list ``[]``)
     - Named sets of material and section properties shared by several elements.
   * - ``nodes``
     - yes
     - Node IDs, coordinates and labels.
   * - ``elements``
     - yes
     - Finite elements (beams, bars, springs) and the nodes they join.
   * - ``constraints``
     - yes
     - Supports: constrained degrees of freedom (DoF) and prescribed displacements.
   * - ``node_loads``
     - no
     - Forces and moments applied at nodes.
   * - ``element_loads``
     - no
     - Distributed, concentrated and thermal loads on elements.

Other top-level keys are ignored.
Names of element types, load types and DoFs are case insensitive (``BEAM2D_RR`` and
``beam2d_rr`` are equivalent).

**Units.** OpenBeam does not assume any unit system: use any consistent one, and results come
out in the same units. The SI is recommended (m, N, Pa, m², m⁴), as in all examples.
Two exceptions: node rotations (``rot_x``, ``rot_y``, ``rot_z``) are given in **degrees**, and
the thermal expansion coefficient of the ``TEMPERATURE`` load is fixed (see below).

**Errors.** If the file cannot be parsed, the error messages give the section and the line
number of the offending entry.

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

    The short JSON-like version works too, and it is the usual one for nodes, elements and loads:

    .. code-block:: yaml

        dictionary_name: {key1: 100e34, key2: foo}

    **Lists** (sequences): one entry per line starting with ``-``, or in the short form ``[a, b, c]``.
    Most sections of a problem file are lists of maps:

    .. code-block:: yaml

        nodes:
        - {id: 0, coords: [0, 0]}
        - {id: 1, coords: [4, 0]}

    **IMPORTANT**: The space after the colon ``:`` is mandatory.


Values, expressions and parameters
--------------------------------------

Every numeric field may be a number or a **mathematical expression**, which may use the
parameters defined in the ``parameters`` section, e.g. ``L/2``, ``2*sqrt(2)*P`` or ``-q*cos(30*pi/180)``.
Expressions are evaluated with `mrpt::expr::CRuntimeCompiledExpression <https://docs.mrpt.org/reference/latest/class_mrpt_expr_CRuntimeCompiledExpression.html>`_,
which uses the `exprtk language <https://github.com/ArashPartow/exprtk#readme>`_: the usual
operators (``+ - * / ^``), parentheses, functions such as ``sqrt``, ``sin``, ``cos``, ``tan``,
``abs``, ``min``, ``max``, and the constant ``pi``. Trigonometric functions use radians.

.. note::

    Most expressions can be written as plain YAML values. Those that start with a character
    with a special meaning in YAML (``*``, ``&``, ``!``, ``[``, ``{``, ``%``, ``@``), or that
    contain a colon or a ``#`` preceded by a space, must be quoted, e.g. ``label: "A: left"``.


1. Parameters
---------------

This **optional** section defines named values, to easily parameterize the dimensions,
loads, etc. of a structure. It is a YAML map: each key becomes a parameter.
Parameters are evaluated in order, so each one may use the ones defined before it.
Defining the same name twice is an error.

Example:

.. code-block:: yaml

    parameters:
      G: 9.81
      L: 1         # Dimensions of the problem
      P: G * 1000  # External load value

The YAML short format is also supported:

.. code-block:: yaml

    parameters: { G: 9.81, L: 1, P: G * 1000 }


2. Beam sections
-------------------

This section gives names to sets of element properties (material and cross section), to be
used by several elements. Each entry needs a unique ``name``; the other keys are the
properties needed by the finite elements that will use it:

.. list-table::
   :header-rows: 1
   :widths: 15 85

   * - Key
     - Meaning
   * - ``E``
     - Young's modulus (e.g. Pa).
   * - ``A``
     - Cross-section area (e.g. m²).
   * - ``Iz``
     - Second moment of area about the local `z` axis, for bending in the XY plane (e.g. m⁴).
   * - ``G``, ``J``
     - Shear modulus and torsion constant (not used by the planar elements).
   * - ``K``, ``Kx``, ``Ky``, ``KRz``
     - Stiffness constants of the spring elements.

Extra keys are not an error: they are ignored by the elements that do not need them.
See :ref:`finite_elements` for the properties required by each element type.

Example:

.. code-block:: yaml

    beam_sections:
    - name: IPE200
      E: 2.1e11     # Young's modulus
      A: 28.5e-4    # Area
      Iz: 1940e-8   # Second moment of area in z
    - {name: TIE, E: 2.1e11, A: 5e-4}

A structure without named sections still needs the (empty) section: ``beam_sections: []``.


3. Geometry: nodes
--------------------

Each node has:

* ``id``: unique integer, starting at 0. IDs must be consecutive (no gaps), but nodes may be
  listed in any order.
* ``coords``: ``[x, y]`` for planar structures, or ``[x, y, z]``.
* ``label`` (optional): a name shown in drawings, diagrams and animations, e.g. ``A``.
* ``rot_x``, ``rot_y``, ``rot_z`` (optional): rotation of the **nodal axes**, in degrees
  (roll, pitch and yaw). Planar structures only need ``rot_z``. Constraints and reactions of a
  node refer to its nodal axes, which is the way to model **inclined supports**: a roller on a
  plane tilted 30 degrees is a node with ``rot_z: 30`` and a ``DY`` constraint.

Example:

.. code-block:: yaml

    nodes:
    - {id: 0, coords: [0   , 0], label: A}
    - {id: 1, coords: [2*L , 0], label: B}
    - {id: 2, coords: [3*L , 0], label: C, rot_z: 45}


4. Geometry: elements
-----------------------

This section lists the finite elements, their type and the nodes they join.
Each entry has:

* ``type``: the element type (see the table below).
* ``nodes``: list of the IDs of the connected nodes, ``[first, second]``. The local `x` axis of
  a bar goes from its first node to its second node.
* ``section``: the name of a beam section, **or** the properties given inline
  (``E``, ``A``, ``Iz``, ``K``...). If ``section`` is present, inline properties are ignored.

Elements have no ID: they are numbered in the order they appear, starting at 0, and
``element_loads`` refer to them by that index.

Planar element types:

.. list-table::
   :header-rows: 1
   :widths: 22 50 28

   * - Type
     - Description
     - Properties
   * - ``BEAM2D_RR``
     - Beam, rigidly connected at both ends.
     - ``E``, ``A``, ``Iz``
   * - ``BEAM2D_AR``
     - Beam with a hinge (no bending moment) at its first node.
     - ``E``, ``A``, ``Iz``
   * - ``BEAM2D_RA``
     - Beam with a hinge at its second node.
     - ``E``, ``A``, ``Iz``
   * - ``BEAM2D_AA``
     - Bar hinged at both ends: axial force only (truss bar).
     - ``E``, ``A``
   * - ``BEAM2D_RD``
     - Beam whose second node slides along the local `y` axis.
     - ``E``, ``A``, ``Iz``
   * - ``SPRING_1D``
     - Linear spring along the line joining both nodes.
     - ``K``
   * - ``SPRING_XY``
     - Two linear springs, along the local `x` and `y` axes.
     - ``Kx``, ``Ky``
   * - ``SPRING_TORSION``
     - Rotational spring about `z`.
     - ``K``
   * - ``SPRING_DXDYRZ``
     - Two linear springs and a rotational one.
     - ``Kx``, ``Ky``, ``KRz``

See :ref:`finite_elements` for the details and stiffness matrices of each type.

.. note::

    Hinges belong to element ends: a hinged joint between two beams is modeled by making the
    end of (at least) one of them a hinge, e.g. ``BEAM2D_RA`` followed by ``BEAM2D_RR``.
    If all the elements meeting at a node are hinged there, the rotation of that node is
    unused and simply discarded.

Example:

.. code-block:: yaml

    elements:
    - {type: BEAM2D_RA, nodes: [0, 1], section: IPE200}
    - {type: BEAM2D_AR, nodes: [1, 2], section: IPE200}
    - {type: BEAM2D_AA, nodes: [0, 2], E: 2.1e11, A: 5e-4}   # inline properties


5. Geometry: constraints
--------------------------

This section lists the supports: the degrees of freedom whose displacement is zero, or a
given value (prescribed displacement, e.g. a support settlement).
Each entry has:

* ``node``: the node ID.
* ``dof``: which DoFs are constrained, by any of these names:

  .. list-table::
     :header-rows: 1
     :widths: 30 70

     * - Name
       - Constrained DoFs
     * - ``DX``, ``DY``, ``DZ``
       - Translation along one axis.
     * - ``RX``, ``RY``, ``RZ``
       - Rotation about one axis.
     * - ``DXDY``, ``DXDZ``, ``DYDZ``
       - Translation along two axes, e.g. ``DXDY`` is a pinned support in 2D.
     * - ``DXDYRZ``
       - Both translations and the rotation: a fixed (clamped) support in 2D.
     * - ``DXRZ``, ``DYRZ``
       - One translation and the rotation: a sliding clamp in 2D.
     * - ``DXDYDZ``, ``RXRYRZ``
       - All translations, or all rotations.
     * - ``DXDYRXRZ``
       - Translations in X, Y and rotations about X and Z (beams with torsion).
     * - ``ALL`` or ``DXDYDZRXRYRZ``
       - All 6 DoFs.

* ``value`` (optional, default 0): prescribed displacement (length units) or rotation
  (radians). It applies to **every** DoF named in ``dof``, so give each DoF with a nonzero
  value its own entry (see the example).

For nodes with rotated axes (``rot_z``, etc.), constrained DoFs and their ``value`` refer to
the rotated nodal axes, e.g. ``DY`` of a node with ``rot_z: 30`` is normal to a plane tilted
30 degrees.

.. note::

    You do not need to specify constraints in DoFs that are not used
    by your finite elements. For example, to fix a 2D rod element to
    ground, you do not need to explicitly specify that the rotation DoFs
    are zero, since the library will automatically discard the unused DoFs.
    Though, it is not an error to overspecify those constraints, only a
    warning will be generated.

Example:

.. code-block:: yaml

    constraints:
    - {node: 0, dof: DXDYRZ}             # fixed support
    - {node: 3, dof: DXDY}               # pinned support
    - {node: 5, dof: DY}                 # roller
    - {node: 6, dof: DXRZ}               # fixed support...
    - {node: 6, dof: DY, value: -1e-3}   # ...that settles 1 mm


6. Loads on nodes
----------------------

Concentrated forces and moments at nodes. Each entry has:

* ``node``: the node ID.
* ``dof``: ``DX``, ``DY``, ``DZ`` for forces along the global axes; ``RX``, ``RY``, ``RZ``
  for moments about them.
* ``value``: magnitude of the force or moment. Positive values follow the axes: forces towards
  +X, +Y, and moments counter-clockwise (right-hand rule about +Z, for planar structures).

Several entries on the same node and DoF add up. Loads are always given in **global axes**,
even on nodes with rotated axes. A load on a constrained DoF is taken directly by the support
(it shows up in the reaction).

Example:

.. code-block:: yaml

    node_loads:
    - {node: 2, dof: DX, value: -P}
    - {node: 2, dof: RZ, value: 5e3}   # counter-clockwise moment


7. Loads on the elements
---------------------------------------

Loads applied along the elements. Each entry has:

* ``element``: the element index (0-based, in the order of the ``elements`` section).
* ``type``: one of the types below, with its own parameters.

Forces need a direction, given by the components ``DX``, ``DY`` (and optionally ``DZ``) of a
vector in **global axes**. It is normalized, so any non-null vector works:
``{DX: 0, DY: -1}`` is a downward load, ``{DX: 1, DY: 1}`` points at 45 degrees.
Load densities are per unit length **of the element** (not of its projection), and
negative values reverse the direction.

.. list-table::
   :header-rows: 1
   :widths: 22 78

   * - Type
     - Parameters
   * - ``DISTRIB_UNIFORM``
     - Uniformly distributed load over the whole element.
       ``q``: load density (e.g. N/m). Direction: ``DX``, ``DY`` [, ``DZ``].
   * - ``TRIANGULAR``
     - Linearly varying (triangular or trapezoidal) load over the whole element.
       ``q_ini``, ``q_end``: load densities at the first and second nodes.
       Direction: ``DX``, ``DY`` [, ``DZ``].
   * - ``CONCENTRATED``
     - Point force inside the element.
       ``p``: force (e.g. N). ``dist``: distance from the first node, between 0 and the
       element length. Direction: ``DX``, ``DY`` [, ``DZ``].
   * - ``TEMPERATURE``
     - Uniform temperature increment, which makes the element expand (or contract, if
       negative). ``deltaT``: increment in Celsius degrees. The thermal expansion
       coefficient is fixed to ``12e-6`` (1/°C, structural steel).

An element may have any number of loads.

Example:

.. code-block:: yaml

    element_loads:
    - {element: 0, type: DISTRIB_UNIFORM, q: 2000*G, DX: 0, DY: -1}
    - {element: 1, type: TRIANGULAR, q_ini: 0, q_end: 4e3, DX: 0, DY: -1}
    - {element: 1, type: CONCENTRATED, p: 10e3, dist: L/2, DX: 0, DY: -1}
    - {element: 2, type: TEMPERATURE, deltaT: 25}


.. _yaml_complete_example:

8. Complete example
---------------------

A portal frame with a diagonal tie, a support settlement, an inclined roller and one load of
each type:

.. code-block:: yaml

    # SI units: m, N, Pa
    parameters:
      L: 6          # span
      H: 4          # height
      q: 10e3       # load on the beam, N/m
      P: 20e3       # lateral force, N

    beam_sections:
    - {name: IPE300, E: 210e9, A: 53.8e-4, Iz: 8356e-8}
    - {name: HEB200, E: 210e9, A: 78.1e-4, Iz: 5696e-8}
    - {name: TIE, E: 210e9, A: 5.0e-4}

    nodes:
    - {id: 0, coords: [0, 0], label: A}
    - {id: 1, coords: [0, H], label: B}
    - {id: 2, coords: [L, H], label: C}
    - {id: 3, coords: [L, 0], label: D, rot_z: 30}   # nodal axes rotated 30 degrees

    elements:
    - {type: BEAM2D_RR, nodes: [0, 1], section: HEB200}   # element 0: left column
    - {type: BEAM2D_RR, nodes: [1, 2], section: IPE300}   # element 1: beam
    - {type: BEAM2D_RA, nodes: [2, 3], section: HEB200}   # element 2: right column, hinged at D
    - {type: BEAM2D_AA, nodes: [0, 2], section: TIE}      # element 3: tie (axial force only)

    constraints:
    - {node: 0, dof: DXRZ}                # A: fixed support...
    - {node: 0, dof: DY, value: -0.01}    # ...that settles 10 mm
    - {node: 3, dof: DY}                  # D: roller on a plane tilted 30 degrees

    node_loads:
    - {node: 1, dof: DX, value: P}
    - {node: 2, dof: RZ, value: -5e3}     # clockwise moment, N·m

    element_loads:
    - {element: 1, type: DISTRIB_UNIFORM, q: q, DX: 0, DY: -1}
    - {element: 1, type: CONCENTRATED, p: 15e3, dist: L/3, DX: 0, DY: -1}
    - {element: 0, type: TRIANGULAR, q_ini: 4e3, q_end: 0, DX: 1, DY: 0}
    - {element: 3, type: TEMPERATURE, deltaT: 20}

More examples, from simple beams to frames and trusses, are in the
`examples-structures <https://github.com/open-beam/openbeam/tree/develop/examples-structures>`_
directory of the repository.
