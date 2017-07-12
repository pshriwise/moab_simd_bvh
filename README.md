

MOAB SIMD BVH
=============

The Mesh Oriented dAtaBase is an excellent library for performing engineering
analysis on arbitrary forms of mesh at scale. Its ability to perform spatial
searches on large mesh is lacking, however.

The purpose of this repository is to provide a performant spatial search kernel
targted at common architectures in both personal workstations and cluster
environments. This kernel takes the form of a bounding volume hierarchy (BVH)
using axis-aligned bounding boxes. Some of the speed will come from the
representation of these boxes in single precision (in progress) and the rest
will come from a set of explicit vectorization commands allowing the kernel to
take full advantage of modern chipset architectures (coming soon).

This project is in the beginning stages and as such its build requirements are
not yet specified other than the need for an installation of MOAB version 5.0 or
greater.

