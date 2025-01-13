# NDQuadTree

C++ implementation of a N-dimensional quadtree with custom features.

## Description

The goal for this project was to create a general purpose space partitioning data structure that would be able to perform a variety of tasks usually handled by multiple different data structures. Each of its core requirements/features will be described separately.

### Construction in any dimension

Most commonly used versions of this space partitioning technique are [quadtree](https://en.wikipedia.org/wiki/Quadtree) (2D) and [octree](https://en.wikipedia.org/wiki/Octree) (3D), but in principle it can be used in any dimension and all the internal structures will look very similar. The key observation is that basically all internal operations can be handled dimension by dimension, which is exploited in this project. The implementation uses C++ templates to instantiate a tree of any dimension based on a template parameter. This way, there is no necessity to have different implementations for different dimensions.

### Point and ND elements

A standard quadtree is used to store point elements only, but that is not always sufficient. For example, a reasonable requirement (in 2D) is to have a space populated with rectangles (axis aligned 2D objects) and then ask which of them collide with a rectangular query window. This may seem like an easy extension, but the question is what to do with rectangles that lie on quad tree nodes' boundaries. In this implementation (point and ND elements can be stored in the tree at the same time), the solution is as follows: First, a midpoint of N-dimensional element is calculated. Then, the element is used as if it was a point element. Secondly, each node stores not only its boundaries, but also its extended boundaries. When an element lies on the node's boundary, its extended boundaries are adjusted in such a way that they fully contain the stored element (nodes can be overlapping now, but that is not a problem). The updated extended boundaries' values are then propagated upwards up to the main node. When query is performed, the extended boundaries are used instead of the normal boundaries, so no elements are missed even if their boundaries stick out of the base boundaries of the node.

### Multiple elements at the same position

It is perfectly possible for multiple different objects to be stored at the same coordinates in space. Thus, the quadtree should be able to handle this situation as well. Every insert/deletion/relocation has the option for an id to be entered along with the object's coordinates. This way, the correct element can always be found.

### Relocation

In some situations, a space partitioning is needed for objects that are moving in space. Quadtrees are great for this because of the way they split space. It is predetermined and does not depend on the elements themselves, so its internal structure can never become degenerate. Relocation of elements can always be implemented naively by deleting and reinserting the element. The time complexity will be O(log(n)), because quadtrees have O(log(n)) for both insertion and deletion, but there is room for improvement and it comes from an observation that whenever an element moves, it will most likely move to a position close to the current one. In the best case scenario, the leaf node where the current coordinates belong is the same as the node where the new coordinates belong. In that case, no changes need to be done to the tree structure and the coordinates just have to be updated. Even if it is not the case, the new coordinates will probably belong to another node with the same parent or grandparent, so even in this case, it is possible to perform the move more efficiently compared to a full removal and full reinsertion of the element.

### Location queries

Standard functionality that quadtrees were designed for. In this implementation, two versions of location queries are provided. First one is a point query. All elements whose boundaries collide with a given point are returned. And the second one is N-dimensional query. All elements whose boundaries collide with N-dimensional query window (2D - rectangular query, 3D - rectangular cuboid query, ...) are returned. 

### Distance queries

Sometimes, it is not needed to find elements in a given location, but to find N closest elements to a given point. This is what [k-d trees](https://en.wikipedia.org/wiki/K-d_tree) are designed for. But it may be undesirable to construct a k-d tree in addition to quadtree just for this purpose. So this functionality is emulated in this implementation using an iterative approach. Using a given step, a location query with that radius is constructed around a given point. If enough elements are returned by this first query (searched through nodes and elements are also marked as already checked), they are sorted by distance and returned. If not, the query window is expanded and another attempt is made to find more not yet checked elements. This continues until a required number of elements is found.

### Ray query

The quad tree structure can also be used to find elements intersecting with a ray. A variety of techniques are usually used for this purpose like [bounding volume hierarchies](https://en.wikipedia.org/wiki/Bounding_volume_hierarchy) that come with their own benefits and challenges. Even though this is not the most efficient way to perform a ray query, if the number of intersecting elements is small, the resulting performance can be quite acceptable, especially when combined with a fact that it can avoid a construction of separate data structure. Implementation works by testing the intersection of a given ray with a quadtree's main node. If they collide, the test is performed recursively on the node's subnodes, otherwise the node is discarded. This happens unit a leaf node is reached and the elements themselves are tested for the intersection with the ray. This query is custom implemented only for the 2D and 3D case, because generalizing the ray intersection algorithm is not trivial.

### Memory reuse

As the elements are inserted, deleted and reallocated, the tree structure will be changing as well, which leads to a large amount of heap memory allocations and deallocations. To combat this issue a helper PointerManager class was implemented. Instead of deallocating not needed memory, the pointer is stored inside this class instead. Then, whenever a new allocation is required, the pointer is reused and new object is constructed at memory location where it points using the placement new.

## Dependencies, installation and execution

NDQuadTree implementation uses only the standard library, so no external dependencies are required. It can be used after the QuadTree.h file is included.

The implementation was compiled and tested on:
* x64 MSVC 19.41.34120 (C++20, /W4) 
* x86-64 clang 19.1.0 (-std=c++20 -Weverything -Wno-c++98-compat-pedantic -Wno-padded -Wno-float-equal)
* x86-64 gcc 14.2 (-std=c++20 -Wextra -Wall -Wno-padded -Wno-float-equal)

The equality of floating point values is being tested in this project, but not within the context of calculation results, so the -Wno-float-equal is justified here.

## Authors

Ing. Tomáš Jarůšek \[[LinkedIn](https://www.linkedin.com/in/tomáš-jarůšek-7a765284)\]

## License

This project is licensed under the GNU GPLv3 License - see the LICENSE.md file for details.
	