

MOAB SIMD BVH
=============

The [Mesh Oriented dAtaBase](http://sigma.mcs.anl.gov/moab-library/) is an excellent library for performing engineering
analysis on arbitrary forms of mesh at scale. Its ability to perform spatial
searches on large mesh is lacking, however.

The purpose of this repository is to provide a performant spatial search kernel
targeted at common architectures in both personal workstations and cluster
environments. This kernel takes the form of a bounding volume hierarchy (BVH)
using axis-aligned bounding boxes. Some of the speed will come from the
representation of these boxes in single precision (in progress) and the rest
will come from a set of explicit vectorization commands allowing the kernel to
take full advantage of modern chipset architectures (coming soon).

This project is in the beginning stages and as such its build requirements are
not yet specified other than the need for an installation of MOAB version 5.0 or
greater.


Build
=====

This library builds using [CMake](https://cmake.org/).

From a fresh build directory run:

`cmake <path_to_moab_simd_bvh_source> -DMOAB_ROOT=<path_to_MOAB_installation>`

This tool relies on MOAB Version5.0 and is designed to operate on MOAB files containing
[DAGMC](https://svalinn.github.io/DAGMC/) mesh geometries.

Options
=======

This tool supports multiple bounding box types. This option is specified at runtime
using the `BBOX_TYPE` cmake variable. Three options for bounding box types are available:

  - AABBs (Axis-Aligned Bounding Boxes)
  - OBBs (Oriented Bounding Boxes)
  - Mixed (both AABBs and OBBs, applied automatically)


For example, to specify that the tool should use OBBs the CMake command will be:

`cmake <path_to_moab_simd_bvh_source> -DMOAB_ROOT=<path_to_MOAB_installation>` -DBBOX_TYPE=OBBs
