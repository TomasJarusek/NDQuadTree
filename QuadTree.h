
#pragma once

#include <algorithm>
#include <cmath>
#include <iostream>

#include "PointerManager.h"
#include "QuadTreeHelperFunctions.h"


// Forward declarations of internal structures.
template<size_t DIM, typename objT, typename coordT = double>
class QuadTreeElement;

template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH = -1, typename coordT = double>
class QuadTreeNode;


// N-dimensional Quad tree.
// Space partitioning data structure that performs insertion, deletion, and querying with time complexity of O(log(n)).
// https://en.wikipedia.org/wiki/Quadtree (2D), https://en.wikipedia.org/wiki/Octree (3D)
// This implementation also includes the option of querying closest elements and elements intersecting ray.
// Tree can be constructed in any dimension (DIM) higher than 0.
// Leaf nodes can hold up to MAX_CAP elements (MAX_CAP >= 1).
// Quad tree can hold objects of any type (objT).
// Maximum depth (MAX_DEPTH) can be set to limit tree division (MAX_DEPTH < 0 means there is no limit (default value)).
// Type of coordinates is specified by coordT (double by default).
template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH = -1, typename coordT = double>
class QuadTree
{
private:

	// Main node of internal quad tree structure.
	QuadTreeNode<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>* mainNode;

	// Pointer managers that facilitate the reuse of allocated memory.
	PointerManager_Unlimited<QuadTreeNode<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>> nodePointerManager;
	PointerManager_Unlimited<QuadTreeElement<DIM, objT, coordT>> elementPointerManager;

	// Element cache used for quad tree operations. It is created here to avoid its realocation for each operation.
	std::vector<QuadTreeElement<DIM, objT, coordT>*> elementCache;

	// Unique identifier for the next distance query. Starts at one.
	long long int distanceQueryId;

	// Epsilon value used in distance and ray queries.
	coordT epsilon = static_cast<coordT>(0.00000001);

public:

	// Default constructor.
	// Creates quad tree without a main node, which makes it unusable.
	QuadTree();
	// Constructor.
	// Takes in 2 arguments per axis/dimension (2*DIM) of type coordT, which represent the boundaries of the tree.
	// The arguments are bundled in pairs per axis: minX, maxX, minY, maxY, ...
	template<typename... Args>
	QuadTree(Args... boundaries);
	// Destructor.
	~QuadTree();

	// Copy/move constructor/assignment.
	QuadTree(const QuadTree& quadTree);
	QuadTree(QuadTree&& quadTree) noexcept;
	QuadTree& operator=(const QuadTree& quadTree);
	QuadTree& operator=(QuadTree&& quadTree) noexcept;

	// Inserts point element into the tree.
	// Parameters:
	// id - Unique identifier of a point element (ids for point and ND elements are handled separately).
	//      Used for situations when multiple elements occupy the same coordinates.
	//      Optional parameter. If it is not needed, then its value has to be set to 0.
	// object - Object that is going to be stored at specified position.
	// coordinates - Coordinates specifying the position of a point element.
	//               One value for each axis/dimension (x, y, z, ...).
	// Return value:
	// True when insertion was successful, false otherwise.
	template<typename T, typename... Args>
	bool InsertPointElement(long long int id, T&& object, Args... coordinates);

	// Inserts N-dimensional element into the tree.
	// Parameters:
	// id - Unique identifier of a ND element (ids for point and ND elements are handled separately).
	//      Used for situations when multiple elements occupy the same coordinates.
	//      Optional parameter. If it is not needed, then its value has to be set to 0.
	// object - Object that is going to be stored at specified position.
	// boundaries - Boundaries specifying the position of a ND element.
	//              Two values for each axis/dimension (minX, maxX, minY, maxY, ...).
	// Return value:
	// True when insertion was successful, false otherwise.
	// Additional info:
	// Internally, a ND element is stored at coordinates of its midpoint ((minX + maxX)/2, (minY + maxY)/2, ...).
	// This means that for the insertion to be successful, the midpoint has to be within the boundaries of the tree.
	// Min and max boundary for specific axis/dimension can have the same value. This way, a lower dimensional
	// structures can be inserted (even a point, but the performance is inferior to InsertPointElement()).
	template<typename T, typename... Args>
	bool InsertNDElement(long long int id, T&& object, Args... boundaries);

	// Deletes point element/s from the tree.
	// Parameters:
	// id - Unique identifier of a point element (ids for point and ND elements are handled separately).
	//      Used for situations when multiple elements occupy the same coordinates.
	//      Optional parameter. If it is not needed, then its value has to be set to 0.
	// coordinates - Coordinates specifying the position of an existing point element.
	//               One value for each axis/dimension (x, y, z, ...).
	// Return value:
	// True when deletion was successful, false otherwise.
	// Additional info:
	// When other elements occupy the same coordinates as the element that is to be deleted,
	// then the id will be used to find the correct one. If the id is not specified, then
	// EVERY POINT element will be deleted from said coordinates. If there are duplicate
	// ids, then the behaviour is undefined.
	template<typename... Args>
	bool DeletePointElement(long long int id, Args... coordinates);

	// Deletes ND element/s from the tree.
	// Parameters:
	// id - Unique identifier of a ND element (ids for point and ND elements are handled separately).
	//      Used for situations when multiple elements occupy the same coordinates.
	//      Optional parameter. If it is not needed, then its value has to be set to 0.
	// boundaries - Boundaries specifying the position of a ND element.
	//              Two values for each axis/dimension (minX, maxX, minY, maxY, ...).
	// Return value:
	// True when deletion was successful, false otherwise.
	// Additional info:
	// When other elements occupy the same coordinates as the element that is to be deleted,
	// then the id will be used to find the correct one. If the id is not specified, then
	// EVERY ND element with MATCHING BOUNDARIES will be deleted from said coordinates.
	// If there are duplicate ids, then the behaviour is undefined.
	template<typename... Args>
	bool DeleteNDElement(long long int id, Args... boundaries);

	// Deletes point element/s from the tree and returns its/their objects.
	// Additional info:
	// Same principle as DeletePointElement(). If deletion failed, then the return vector will be empty.
	template<typename... Args>
	std::vector<objT> DeleteAndReturnPointElement(long long int id, Args... coordinates);

	// Deletes ND element/s from the tree and returns its/their objects.
	// Additional info:
	// Same principle as DeleteNDElement(). If deletion failed, then the return vector will be empty.
	template<typename... Args>
	std::vector<objT> DeleteAndReturnNDElement(long long int id, Args... boundaries);

	// Relocates existing point element/s to new position.
	// Parameters:
	// id - Unique identifier of a point element (ids for point and ND elements are handled separately).
	//      Used for situations when multiple elements occupy the same coordinates.
	//      Optional parameter. If it is not needed, then its value has to be set to 0.
	// coordinates - Coordinates specifying both the existing position and the new position.
	//               One value for each axis/dimension, twice (currentX, currentY, currentZ, ..., newX, newY, newZ, ...).
	// Return value:
	// True when relocation was successful, false otherwise.
	// Additional info:
	// Deletion rules apply for existing position and insertion rules apply for new position.
	template<typename... Args>
	bool RelocatePointElement(long long int id, Args... coordinates);

	// Relocates existing ND element/s to new position.
	// Parameters:
	// id - Unique identifier of a point element (ids for point and ND elements are handled separately).
	//      Used for situations when multiple elements occupy the same coordinates.
	//      Optional parameter. If it is not needed, then its value has to be set to 0.
	// boundaries - Boundaries specifying both the existing position and the new position.
	//              Two values for each axis/dimension, twice (currentMinX, currentMaxX, currentMinY, currentMaxY, ..., newMinX, newMaxX, newMinY, newMaxY, ...).
	// Return value:
	// True when relocation was successful, false otherwise.
	// Additional info:
	// Deletion rules apply for existing position and insertion rules apply for new position.
	template<typename... Args>
	bool RelocateNDElement(long long int id, Args... boundaries);

	// Finds all elements (point or ND) that intersect the query point.
	// Parameters:
	// coordinates - Coordinates specifying the query point.
	//               One value for each axis/dimension (queryPointX, queryPointY, queryPointZ, ...).
	// Return value:
	// Vector containing the objects stored inside intersecting elements.
	// Additional info:
	// If query point lies exactly on the edge of an element, then it is still considered an intersection.
	template<typename... Args>
	std::vector<objT> PointQuery(Args... coordinates);

	// Finds all elements (point or ND) that intersect the ND query window.
	// Parameters:
	// boundaries - Boundaries specifying the ND query window.
	//              Two values for each axis/dimension (queryWindowMinX, queryWindowMaxX, queryWindowMinY, queryWindowMaxY, ...).
	// Return value:
	// Vector containing the objects stored inside intersecting elements.
	// Additional info:
	// If the ND query window touches the edge of an element, then it is still considered an intersection.
	template<typename... Args>
	std::vector<objT> NDQuery(Args... boundaries);

	// Finds all elements (point or ND) that intersect the query point and deletes them.
	// Additional info:
	// Same principle as PointQuery().
	template<typename... Args>
	std::vector<objT> PointQueryAndDelete(Args... coordinates);

	// Finds all elements (point or ND) that intersect the ND query window and deletes them.
	// Additional info:
	// Same principle as NDQuery().
	template<typename... Args>
	std::vector<objT> NDQueryAndDelete(Args... boundaries);

	// Finds closest element (point or ND) from the given point using an iterative approach.
	// Parameters:
	// step - Extension of query boundaries in a single iteration.
	// coordinates - Coordinates specifying the query point.
	//               One value for each axis/dimension (queryPointX, queryPointY, queryPointZ, ...).
	// Return value:
	// Optional (tree can be empty) pair containing the object stored inside the closest
	// element together with its squared distance to the query point.
	// Additional info:
	// When multiple elements occupy the same coordinates, then random element is chosen from them.
	// For ND elements, distance to their midpoint is considered for the calculations, not bounding box.
	template<typename... Args>
	std::optional<std::pair<coordT, objT>> QueryClosestElement(coordT step, Args... coordinates);

	// Finds N closest elements (point or ND) from the given point using an iterative approach.
	// Parameters:
	// count - Number of elements to be found.
	// step - Extension of query boundaries in a single iteration.
	// coordinates - Coordinates specifying the query point.
	//               One value for each axis/dimension (queryPointX, queryPointY, queryPointZ, ...).
	// Return value:
	// Vector containing the objects stored inside found elements together
	// with their squared distance to the query point. They are ordered from closest to farthest.
	// Additional info:
	// When multiple elements occupy the same coordinates, but the algorithm is at the cut off point, then random elements are chosen from them.
	// If the number of elements in the tree is lower than the requested amount, then the requested amount won't be fullfiled.
	// For ND elements, distance to their midpoint is considered for the calculations, not bounding box.
	template<typename... Args>
	std::vector<std::pair<coordT, objT>> QueryNClosestElements(size_t count, coordT step, Args... coordinates);

	// Finds all elements (point or ND) within given radius from the given point.
	// Parameters:
	// radius - Distance from the query point in which all elements will be found.
	// coordinates - Coordinates specifying the query point.
	//               One value for each axis/dimension (queryPointX, queryPointY, queryPointZ, ...).
	// Return value:
	// Vector containing the objects stored inside found elements together
	// with their squared distance to the query point. They are ordered from closest to farthest.
	// Additional info:
	// For ND elements, distance to their midpoint is considered for the calculations, not bounding box.
	// Elements lying on a circular boundary given by the radius are returned as well. Epsilon value is used
	// in calculations to mitigate numerical errors.
	template<typename... Args>
	std::vector<std::pair<coordT, objT>> QueryElementsWithinRadius(coordT radius, Args... coordinates);

	// Finds closest element (point or ND) from the given point using an iterative approach and deletes it.
	// Additional info:
	// Same principle as QueryClosestElement().
	template<typename... Args>
	std::optional<std::pair<coordT, objT>> QueryClosestElementAndDelete(coordT step, Args... coordinates);

	// Finds N closest elements (point or ND) from the given point using an iterative approach and deletes them.
	// Additional info:
	// Same principle as QueryNClosestElements().
	template<typename... Args>
	std::vector<std::pair<coordT, objT>> QueryNClosestElementsAndDelete(size_t count, coordT step, Args... coordinates);

	// Finds all elements (point or ND) within given radius from the given point and deletes them.
	// Additional info:
	// Same principle as QueryElementsWithinRadius().
	template<typename... Args>
	std::vector<std::pair<coordT, objT>> QueryElementsWithinRadiusAndDelete(coordT radius, Args... coordinates);

	// Finds all elements (point or ND) that intersect the query ray.
	// Parameters:
	// coordinates - Coordinates specifying two points defining the ray.
	//               One value for each axis/dimension, twice (rayFirstPointX, rayFirstPointY, ..., raySecondPointX, raySecondPointY, ...).
	// Return value:
	// Vector containing the objects stored inside intersecting elements.
	// Additional info:
	// Only implemented for 2D and 3D.
	// If ray edge of vertex touches an elements, it is still considered an intersection.
	// Epsilon value is used in calculations to mitigate the effects of numerical precision limits.
	// IMPORTANT - Works only for point elements (inserted as point elements) and non degenerate ND elements.
	template<typename... Args>
	std::vector<objT> RayQuery(Args... coordinates);

	// Finds all elements (point or ND) that intersect the query ray.
	// Additional info:
	// Same principle as RayQuery().
	template<typename... Args>
	std::vector<objT> RayQueryAndDelete(Args... coordinates);

	// Removes all elements from the quad tree.
	void Clear();

	// Returns the number of stored elements.
	// Return value:
	// Number of stored elements.
	size_t Size() const;

	// Returns the number of stored elements occupying unique coordinates.
	// Return value:
	// Number of stored elements occupying unique coordinates.
	size_t UniqueSize() const;

	// Returns a value of specific tree boundary in specific dimension.
	// Return value:
	// Value of specified boundary.
	coordT GetBoundary(size_t dimension, bool isMin) const;

	// Validates tree structure for debug purposes.
	// Return value:
	// True if tree structure is valid, false otherwise.
	bool ValidateTree() const;


private:

	// Calculates start and end radius for distace query.
	std::pair<coordT, coordT> CalculateStartAndEndRadiusForDistanceQuery(const std::array<coordT, DIM>& queryCoordinates) const;
};

template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
QuadTree<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>::QuadTree()
{
	static_assert(DIM > 0, "QuadTree's dimension has to be greater than 0.");
	static_assert(MAX_CAP > 0, "QuadTree's leaf nodes must be able to hold at least one element.");

	this->mainNode = nullptr;

	this->distanceQueryId = 1;
}

template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
template<typename... Args>
QuadTree<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>::QuadTree(Args... boundaries)
{
	static_assert(DIM > 0, "QuadTree's dimension has to be greater than 0.");
	static_assert(MAX_CAP > 0, "QuadTree's leaf nodes must be able to hold at least one element.");
	static_assert(sizeof... (boundaries) == 2*DIM, "QuadTree requires two boundaries (min, max) for each axis/dimensions.");
	static_assert((... && std::is_same_v<Args, coordT>) == true, "All arguments must be of the coordinate type (coordT) specified in class template.");

	// Boundaries are unpacked into an array.
	const std::array<coordT, 2*DIM> boundariesArray = {boundaries...};

	for (size_t i = 0; i < DIM; ++i)
	{
		// Min has to be always smaller then max.
		if (boundariesArray[2*i] > boundariesArray[2*i + 1])
		{
			throw std::logic_error("Min boundary value cannot be bigger than max boundary value.");
		}
	}

	// Boundaries are passed to the constructor of the main node. Main node does not have a parent.
	this->mainNode = this->nodePointerManager.Create(boundariesArray, &this->nodePointerManager, &this->elementPointerManager);

	this->distanceQueryId = 1;
}

template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
QuadTree<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>::~QuadTree()
{
	// Main node can be nullptr when the tree is default constructed or after a move.
	this->nodePointerManager.Delete_Nullptr(this->mainNode);
}

template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
QuadTree<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>::QuadTree(const QuadTree& quadTree)
{
	// Copy of main node has to be constructed. Special constructor is implemented for that purpose inside the node class.
	this->mainNode = this->nodePointerManager.Create(quadTree.mainNode, nullptr, &this->nodePointerManager, &this->elementPointerManager);

	this->distanceQueryId = quadTree.distanceQueryId;
}

template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
QuadTree<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>::QuadTree(QuadTree&& quadTree) noexcept : nodePointerManager(std::move(quadTree.nodePointerManager)), elementPointerManager(std::move(quadTree.elementPointerManager))
{
	this->mainNode = quadTree.mainNode;
	// Because pointer managers were moved to a different memory location, the pointers inside the main node have to be updated.
	this->mainNode->UpdatePointerManagers(&this->nodePointerManager, &this->elementPointerManager);
	quadTree.mainNode = nullptr;

	this->distanceQueryId = quadTree.distanceQueryId;
}

template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
QuadTree<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>& QuadTree<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>::operator=(const QuadTree& quadTree)
{
	// Main node can be nullptr when the tree is default constructed or after a move.
	this->nodePointerManager.Delete_Nullptr(this->mainNode);

	// Copy of main node has to be constructed. Special constructor is implemented for that purpose inside the node class.
	this->mainNode = this->nodePointerManager.Create(quadTree.mainNode, nullptr, &this->nodePointerManager, &this->elementPointerManager);

	this->distanceQueryId = quadTree.distanceQueryId;

	return *this;
}

template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
QuadTree<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>& QuadTree<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>::operator=(QuadTree&& quadTree) noexcept
{
	// Main node can be nullptr when the tree is default constructed or after a move.
	this->nodePointerManager.Delete_Nullptr(this->mainNode);

	this->nodePointerManager = std::move(quadTree.nodePointerManager);
	this->elementPointerManager = std::move(quadTree.elementPointerManager);

	this->mainNode = quadTree.mainNode;
	// Because pointer managers were moved to a different memory location, the pointers inside the main node have to be updated.
	this->mainNode->UpdatePointerManagers(&this->nodePointerManager, &this->elementPointerManager);
	quadTree.mainNode = nullptr;

	this->distanceQueryId = quadTree.distanceQueryId;

	return *this;
}

template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
template<typename T, typename... Args>
bool QuadTree<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>::InsertPointElement(long long int id, T&& object, Args... coordinates)
{
	static_assert(sizeof... (coordinates) == DIM, "Point element requires one coordinate for each axis/dimensions.");
	static_assert((... && std::is_same_v<Args, coordT>) == true, "All coordinate arguments must be of the coordinate type (coordT) specified in class template.");
	static_assert(std::is_same_v<std::remove_const_t<std::remove_reference_t<T>>, objT> == true, "Object argument must be of the object type (objT) specified in class template.");

	// For point element, midpoint is the same as point coordinates.
	const std::array<coordT, DIM> midpoint = {coordinates...};

	// For point element, min and max of boundaries are the same as corresponding coordinate.
	std::array<coordT, 2*DIM> boundaries = {};

	for (size_t i = 0; i < DIM; ++i)
	{
		boundaries[2*i] = midpoint[i];
		boundaries[2*i + 1] = midpoint[i];
	}

	// Check that the new element's midpoint is within the bounds of a main quad tree node.
	if (this->mainNode->DoesMidpointBelongToMainNode(midpoint) == false)
	{
		return false;
	}

	// New element is allocated.
	QuadTreeElement<DIM, objT, coordT>* newElement = this->elementPointerManager.Create(id, std::forward<T>(object), midpoint, boundaries, true);

	// Insert it into the internal structure.
	return this->mainNode->InsertElement(newElement);
}

template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
template<typename T, typename... Args>
bool QuadTree<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>::InsertNDElement(long long int id, T&& object, Args... boundaries)
{
	static_assert(sizeof... (boundaries) == 2*DIM, "ND element requires two boundaries (min, max) for each axis/dimensions.");
	static_assert((... && std::is_same_v<Args, coordT>) == true, "All boundaries must be of the coordinate type (coordT) specified in class template.");
	static_assert(std::is_same_v<std::remove_const_t<std::remove_reference_t<T>>, objT> == true, "Object argument must be of the object type (objT) specified in class template.");

	// Boundaries are unpacked into an array.
	const std::array<coordT, 2*DIM> boundariesArray = {boundaries...};

	// Midpoint is calculated.
	std::array<coordT, DIM> midpoint;

	for (size_t i = 0; i < DIM; ++i)
	{
		// Min has to be always smaller then max.
		if (boundariesArray[2*i] > boundariesArray[2*i + 1])
		{
			return false;
		}

		midpoint[i] = (boundariesArray[2*i] + boundariesArray[2*i + 1])/2.0;
	}

	// Check that the new element's midpoint is within the bounds of a main quad tree node.
	if (this->mainNode->DoesMidpointBelongToMainNode(midpoint) == false)
	{
		return false;
	}

	// New element is allocated.
	QuadTreeElement<DIM, objT, coordT>* newElement = this->elementPointerManager.Create(id, std::forward<T>(object), midpoint, boundariesArray, false);

	// It is inserted into the internal structure.
	return mainNode->InsertElement(newElement);
}

template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
template<typename... Args>
bool QuadTree<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>::DeletePointElement(long long int id, Args... coordinates)
{
	static_assert(sizeof... (coordinates) == DIM, "Point element requires one coordinate for each axis/dimensions.");
	static_assert((... && std::is_same_v<Args, coordT>) == true, "All coordinate arguments must be of the coordinate type (coordT) specified in class template.");

	// For point element, midpoint is the same as point coordinates.
	const std::array<coordT, DIM> midpoint = {coordinates...};
	// For point element, min and max of boundaries are the same.
	std::array<coordT, 2*DIM> boundaries = {};
	for (size_t i = 0; i < DIM; ++i)
	{
		boundaries[2*i] = midpoint[i];
		boundaries[2*i + 1] = midpoint[i];
	}

	// Check that the midpoint is within the bounds of a main quad tree node.
	if (this->mainNode->DoesMidpointBelongToMainNode(midpoint) == false)
	{
		return false;
	}

	// Reset element cache.
	this->elementCache.clear();
	// Try to delele the element from the internal structure.
	const bool deletionSuccess = this->mainNode->DeleteElement(this->elementCache, id, midpoint, boundaries, true);

	// If any elements were deleted, they are now deallocated.
	if (deletionSuccess == true)
	{
		for (size_t i = 0; i < this->elementCache.size(); ++i)
		{
			this->elementPointerManager.Delete(this->elementCache[i]);
		}
	}

	return deletionSuccess;
}

template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
template<typename... Args>
bool QuadTree<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>::DeleteNDElement(long long int id, Args... boundaries)
{
	static_assert(sizeof... (boundaries) == 2*DIM, "ND element requires two boundaries (min, max) for each axis/dimensions.");
	static_assert((... && std::is_same_v<Args, coordT>) == true, "All boundaries must be of the coordinate type (coordT) specified in class template.");

	// Boundaries are unpacked into an array.
	const std::array<coordT, 2*DIM> boundariesArray = {boundaries...};

	// Midpoint is calculated.
	std::array<coordT, DIM> midpoint;

	for (size_t i = 0; i < DIM; ++i)
	{
		midpoint[i] = (boundariesArray[2*i] + boundariesArray[2*i + 1])/2.0;
	}

	// Check that the midpoint is within the bounds of a main quad tree node.
	if (this->mainNode->DoesMidpointBelongToMainNode(midpoint) == false)
	{
		return false;
	}

	// Reset element cache.
	this->elementCache.clear();
	// Try to delele the element from the internal structure.
	const bool deletionSuccess = this->mainNode->DeleteElement(this->elementCache, id, midpoint, boundariesArray, false);

	// If any elements were deleted, they are now deallocated.
	if (deletionSuccess == true)
	{
		for (size_t i = 0; i < this->elementCache.size(); ++i)
		{
			this->elementPointerManager.Delete(this->elementCache[i]);
		}
	}

	return deletionSuccess;
}

template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
template<typename... Args>
std::vector<objT> QuadTree<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>::DeleteAndReturnPointElement(long long int id, Args... coordinates)
{
	static_assert(sizeof... (coordinates) == DIM, "Point element requires one coordinate for each axis/dimensions.");
	static_assert((... && std::is_same_v<Args, coordT>) == true, "All coordinate arguments must be of the coordinate type (coordT) specified in class template.");

	// For point element, midpoint is the same as point coordinates.
	const std::array<coordT, DIM> midpoint = {coordinates...};
	// For point element, min and max of boundaries are the same.
	std::array<coordT, 2*DIM> boundaries = {};
	for (size_t i = 0; i < DIM; ++i)
	{
		boundaries[2*i] = midpoint[i];
		boundaries[2*i + 1] = midpoint[i];
	}

	// Vector containing deleted objects.
	std::vector<objT> returnObjects;

	// Check that the midpoint is within the bounds of a main quad tree node.
	if (this->mainNode->DoesMidpointBelongToMainNode(midpoint) == false)
	{
		return returnObjects;
	}

	// Most likely, only 1 element is going to be deleted. +2 is an arbitrary overhang for when multiple elements occupy the same coordinates.
	returnObjects.reserve(1 + 2);

	// Reset element cache.
	this->elementCache.clear();
	// Try to delele the element from the internal structure.
	const bool deletionSuccess = this->mainNode->DeleteElement(this->elementCache, id, midpoint, boundaries, true);

	// If any elements were deleted, their objects are now transfered to the return vector. Then they are deallocated.
	if (deletionSuccess == true)
	{
		for (size_t i = 0; i < this->elementCache.size(); ++i)
		{
			returnObjects.push_back(std::move(this->elementCache[i]->object));

			this->elementPointerManager.Delete(this->elementCache[i]);
		}
	}

	return returnObjects;
}

template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
template<typename... Args>
std::vector<objT> QuadTree<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>::DeleteAndReturnNDElement(long long int id, Args... boundaries)
{
	static_assert(sizeof... (boundaries) == 2*DIM, "ND element requires two boundaries (min, max) for each axis/dimensions.");
	static_assert((... && std::is_same_v<Args, coordT>) == true, "All boundaries must be of the coordinate type (coordT) specified in class template.");

	// Boundaries are unpacked into an array.
	const std::array<coordT, 2*DIM> boundariesArray = {boundaries...};

	// Midpoint is calculated.
	std::array<coordT, DIM> midpoint;

	for (size_t i = 0; i < DIM; ++i)
	{
		midpoint[i] = (boundariesArray[2*i] + boundariesArray[2*i + 1])/2.0;
	}

	// Vector containing deleted objects.
	std::vector<objT> returnObjects;

	// Check that the midpoint is within the bounds of a main quad tree node.
	if (this->mainNode->DoesMidpointBelongToMainNode(midpoint) == false)
	{
		return returnObjects;
	}

	// Most likely, only 1 element is going to be deleted. +2 is an arbitrary overhang for when multiple elements occupy the same coordinates.
	returnObjects.reserve(1 + 2);

	// Reset element cache.
	this->elementCache.clear();
	// Try to delele the element from the internal structure.
	const bool deletionSuccess = this->mainNode->DeleteElement(this->elementCache, id, midpoint, boundariesArray, false);

	// If any elements were deleted, their objects are now transfered to the return vector. Then they are deallocated.
	if (deletionSuccess == true)
	{
		for (size_t i = 0; i < this->elementCache.size(); ++i)
		{
			returnObjects.push_back(std::move(this->elementCache[i]->object));

			this->elementPointerManager.Delete(this->elementCache[i]);
		}
	}

	return returnObjects;
}

template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
template<typename... Args>
bool QuadTree<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>::RelocatePointElement(long long int id, Args... coordinates)
{
	static_assert(sizeof... (coordinates) == DIM*2, "Point element relocation requires one coordinate for each axis/dimension, twice.");
	static_assert((... && std::is_same_v<Args, coordT>) == true, "All coordinate arguments must be of the coordinate type (coordT) specified in class template.");

	std::array<coordT, DIM> currentMidpoint = {};
	std::array<coordT, 2*DIM> currentBoundaries = {};
	std::array<coordT, DIM> newMidpoint = {};
	std::array<coordT, 2*DIM> newBoundaries = {};

	// First half of the parameter pack contains current coordinates and the second one contains new coordinates.
	SplitAndSaveParameterPackIntoArrays_SequenceGenerator(currentMidpoint, newMidpoint, coordinates...);

	// For point element, min and max of boundaries are the same.
	for (size_t i = 0; i < DIM; ++i)
	{
		currentBoundaries[2*i] = currentMidpoint[i];
		currentBoundaries[2*i + 1] = currentMidpoint[i];

		newBoundaries[2*i] = newMidpoint[i];
		newBoundaries[2*i + 1] = newMidpoint[i];
	}

	// Check that the new midpoint is within the bounds of a main quad tree node.
	if (this->mainNode->DoesMidpointBelongToMainNode(newMidpoint) == false)
	{
		return false;
	}

	// Reset element cache.
	this->elementCache.clear();

	// It is relocated inside the internal structure.
	return this->mainNode->RelocateElement(this->elementCache, id, currentMidpoint, currentBoundaries, newMidpoint, newBoundaries, true);
}

template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
template<typename... Args>
bool QuadTree<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>::RelocateNDElement(long long int id, Args... boundaries)
{
	static_assert(sizeof... (boundaries) == 2*DIM*2, "ND element requires two boundaries (min, max) for each axis/dimensions, twice.");
	static_assert((... && std::is_same_v<Args, coordT>) == true, "All boundaries must be of the coordinate type (coordT) specified in class template.");

	std::array<coordT, DIM> currentMidpoint = {};
	std::array<coordT, 2*DIM> currentBoundaries = {};
	std::array<coordT, DIM> newMidpoint = {};
	std::array<coordT, 2*DIM> newBoundaries = {};

	// First half of the parameter pack contains current boundaries and the second one contains new boundaries.
	SplitAndSaveParameterPackIntoArrays_SequenceGenerator(currentBoundaries, newBoundaries, boundaries...);

	// Midpoints are calculated.
	for (size_t i = 0; i < DIM; ++i)
	{
		currentMidpoint[i] = (currentBoundaries[2*i] + currentBoundaries[2*i + 1])/2.0;
		newMidpoint[i] = (newBoundaries[2*i] + newBoundaries[2*i + 1])/2.0;
	}

	// Check that the new midpoint is within the bounds of a main quad tree node.
	if (this->mainNode->DoesMidpointBelongToMainNode(newMidpoint) == false)
	{
		return false;
	}

	// Reset element cache.
	this->elementCache.clear();

	// It is relocated inside the internal structure.
	return this->mainNode->RelocateElement(this->elementCache, id, currentMidpoint, currentBoundaries, newMidpoint, newBoundaries, false);
}

template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
template<typename... Args>
std::vector<objT> QuadTree<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>::PointQuery(Args... coordinates)
{
	static_assert(sizeof... (coordinates) == DIM, "Point query requires one coordinate for each axis/dimensions.");
	static_assert((... && std::is_same_v<Args, coordT>) == true, "All coordinate arguments must be of the coordinate type (coordT) specified in class template.");

	// Coordinates are unpacked into an array.
	const std::array<coordT, DIM> queryCoordinates = {coordinates...};

	// Internally, a point query is handled as a ND query with min and max being the same.
	std::array<coordT, 2*DIM> queryBoundaries = {};

	for (size_t i = 0; i < DIM; ++i)
	{
		queryBoundaries[2*i] = queryCoordinates[i];
		queryBoundaries[2*i + 1] = queryCoordinates[i];
	}

	// Element cache is reset.
	this->elementCache.clear();

	// Querying starts from the main node.
	this->mainNode->LocationQuery_Unordered(this->elementCache, queryBoundaries);

	// Object is copied from each returned element into a return vector.
	std::vector<objT> returnVector;
	// Arbitrary reserve.
	returnVector.reserve(5);

	for (QuadTreeElement<DIM, objT, coordT>* element : this->elementCache)
	{
		returnVector.push_back(element->object);
	}

	return returnVector;
}

template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
template<typename... Args>
std::vector<objT> QuadTree<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>::NDQuery(Args... boundaries)
{
	static_assert(sizeof... (boundaries) == 2*DIM, "ND query requires two boundaries (min, max) for each axis/dimensions.");
	static_assert((... && std::is_same_v<Args, coordT>) == true, "All boundaries must be of the coordinate type (coordT) specified in class template.");

	// Boundaries are unpacked into an array.
	const std::array<coordT, 2*DIM> queryBoundaries = {boundaries...};

	for (size_t i = 0; i < DIM; ++i)
	{
		// Min has to be always smaller then max.
		if (queryBoundaries[2*i] > queryBoundaries[2*i + 1])
		{
			throw std::logic_error("Min boundary value cannot be bigger than max boundary value.");
		}
	}

	// Element cache is reset.
	this->elementCache.clear();

	// Querying starts from the main node.
	this->mainNode->LocationQuery_Unordered(this->elementCache, queryBoundaries);

	// Object is copied from each returned element into a return vector.
	std::vector<objT> returnVector;
	// Arbitrary reserve.
	returnVector.reserve(20);

	for (QuadTreeElement<DIM, objT, coordT>* element : this->elementCache)
	{
		returnVector.push_back(element->object);
	}

	return returnVector;
}

template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
template<typename... Args>
std::vector<objT> QuadTree<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>::PointQueryAndDelete(Args... coordinates)
{
	static_assert(sizeof... (coordinates) == DIM, "Point query requires one coordinate for each axis/dimensions.");
	static_assert((... && std::is_same_v<Args, coordT>) == true, "All coordinate arguments must be of the coordinate type (coordT) specified in class template.");

	// Coordinates are unpacked into an array.
	const std::array<coordT, DIM> queryCoordinates = {coordinates...};

	// Internally, a point query is handled as a ND query with min and max being the same.
	std::array<coordT, 2*DIM> queryBoundaries = {};

	for (size_t i = 0; i < DIM; ++i)
	{
		queryBoundaries[2*i] = queryCoordinates[i];
		queryBoundaries[2*i + 1] = queryCoordinates[i];
	}

	// Element cache is reset.
	this->elementCache.clear();

	// Querying starts from the main node.
	this->mainNode->LocationQuery_Unordered(this->elementCache, queryBoundaries);

	// Object is moved from each returned element into a return vector.
	std::vector<objT> returnVector;
	// Arbitrary reserve.
	returnVector.reserve(5);

	for (QuadTreeElement<DIM, objT, coordT>* element : this->elementCache)
	{
		returnVector.push_back(std::move(element->object));
	}

	// Finally, each element is deleted from the tree.

	// Internal element cache is copied, because it is used during deletion.
	std::vector<QuadTreeElement<DIM, objT, coordT>*> elementCacheCopy(this->elementCache);
	this->elementCache.clear();

	for (QuadTreeElement<DIM, objT, coordT>* element : elementCacheCopy)
	{
		// This delete should never fail, because the elements were just retrieved from the tree.

		if (this->mainNode->DeleteElement(this->elementCache, element->id, element->midpoint, element->boundaries, element->isPoint) == false)
		{
			throw std::logic_error("Internal error occured during deletion in PointQueryAndDelete().");
		}

		this->elementPointerManager.Delete(element);
	}

	return returnVector;
}

template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
template<typename... Args>
std::vector<objT> QuadTree<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>::NDQueryAndDelete(Args... boundaries)
{
	static_assert(sizeof... (boundaries) == 2*DIM, "ND query requires two boundaries (min, max) for each axis/dimensions.");
	static_assert((... && std::is_same_v<Args, coordT>) == true, "All boundaries must be of the coordinate type (coordT) specified in class template.");

	// Boundaries are unpacked into an array.
	const std::array<coordT, 2*DIM> queryBoundaries = {boundaries...};

	for (size_t i = 0; i < DIM; ++i)
	{
		// Min has to be always smaller then max.
		if (queryBoundaries[2*i] > queryBoundaries[2*i + 1])
		{
			throw std::logic_error("Min boundary value cannot be bigger than max boundary value.");
		}
	}

	// Element cache is reset.
	this->elementCache.clear();

	// Querying starts from the main node.
	this->mainNode->LocationQuery_Unordered(this->elementCache, queryBoundaries);

	// Object is moved from each returned element into a return vector.
	std::vector<objT> returnVector;
	// Arbitrary reserve.
	returnVector.reserve(20);

	for (QuadTreeElement<DIM, objT, coordT>* element : this->elementCache)
	{
		returnVector.push_back(std::move(element->object));
	}

	// Finally, each element is deleted from the tree.

	// Internal element cache is copied, because it is used during deletion.
	std::vector<QuadTreeElement<DIM, objT, coordT>*> elementCacheCopy(this->elementCache);
	this->elementCache.clear();

	for (QuadTreeElement<DIM, objT, coordT>* element : elementCacheCopy)
	{
		// This delete should never fail, because the elements were just retrieved from the tree.

		if (this->mainNode->DeleteElement(this->elementCache, element->id, element->midpoint, element->boundaries, element->isPoint) == false)
		{
			throw std::logic_error("Internal error occured during deletion in PointQueryAndDelete().");
		}

		this->elementPointerManager.Delete(element);
	}

	return returnVector;
}

template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
template<typename... Args>
std::optional<std::pair<coordT, objT>> QuadTree<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>::QueryClosestElement(coordT step, Args... coordinates)
{
	// Based on QueryNClosestElements().

	static_assert(sizeof... (coordinates) == DIM, "Distance query requires one coordinate for each axis/dimensions.");
	static_assert((... && std::is_same_v<Args, coordT>) == true, "All coordinate arguments must be of the coordinate type (coordT) specified in class template.");

	if (step <= static_cast<coordT>(0.0))
	{
		throw std::logic_error("Value of interation step has to be positive.");
	}

	const std::array<coordT, DIM> queryCoordinates = {coordinates...};

	long long int localDistanceQueryId = this->distanceQueryId++;

	auto [queryStartRadius, queryEndRadius] = this->CalculateStartAndEndRadiusForDistanceQuery(queryCoordinates);

	std::array<coordT, 2*DIM> queryBoundaries = {};

	for (size_t i = 0; i < DIM; ++i)
	{
		queryBoundaries[2*i] = queryCoordinates[i] - queryStartRadius;
		queryBoundaries[2*i + 1] = queryCoordinates[i] + queryStartRadius;
	}

	this->elementCache.clear();
	// Only one optional pair is needed.
	std::optional<std::pair<coordT, objT>> returnOptionalPair;

	coordT queryCurrentRadius = queryStartRadius;

	while (queryCurrentRadius < queryEndRadius)
	{
		queryCurrentRadius += step;

		for (size_t i = 0; i < DIM; ++i)
		{
			queryBoundaries[2*i] = queryBoundaries[2*i] - step;
			queryBoundaries[2*i + 1] = queryBoundaries[2*i + 1] + step;
		}
		
		this->mainNode->DistanceQuerySingleIteration(localDistanceQueryId, this->elementCache, queryCoordinates, queryBoundaries);

		coordT queryCurrentSquaredRadius = (queryCurrentRadius - this->epsilon)*(queryCurrentRadius - this->epsilon);

		// Single element is returned, so a simplified if statement is sufficient.
		if (this->elementCache.empty() == false && this->elementCache.front()->squaredDistance <= queryCurrentSquaredRadius)
		{
			returnOptionalPair.emplace(this->elementCache.front()->squaredDistance, this->elementCache.front()->object);

			// Element cache is untouched because it won't be used again.

			break;
		}
	}

	// This situation can still occur even for a single element.
	if (this->elementCache.empty() == false && returnOptionalPair.has_value() == false)
	{
		returnOptionalPair.emplace(this->elementCache.front()->squaredDistance, this->elementCache.front()->object);

		// Element cache is untouched because it won't be used again.
	}

	return returnOptionalPair;
}

template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
template<typename... Args>
std::vector<std::pair<coordT, objT>> QuadTree<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>::QueryNClosestElements(size_t count, coordT step, Args... coordinates)
{
	static_assert(sizeof... (coordinates) == DIM, "Distance query requires one coordinate for each axis/dimensions.");
	static_assert((... && std::is_same_v<Args, coordT>) == true, "All coordinate arguments must be of the coordinate type (coordT) specified in class template.");

	if (step <= static_cast<coordT>(0.0))
	{
		throw std::logic_error("Value of interation step has to be positive.");
	}

	// Coordinates are unpacked into an array.
	const std::array<coordT, DIM> queryCoordinates = {coordinates...};

	// New unique distance query id is generated.
	long long int localDistanceQueryId = this->distanceQueryId++;

	// Start and end value for query window radius are calculated.
	auto [queryStartRadius, queryEndRadius] = this->CalculateStartAndEndRadiusForDistanceQuery(queryCoordinates);

	// Query boundaries are moved to the start position. Query window is always square.
	std::array<coordT, 2*DIM> queryBoundaries = {};

	for (size_t i = 0; i < DIM; ++i)
	{
		queryBoundaries[2*i] = queryCoordinates[i] - queryStartRadius;
		queryBoundaries[2*i + 1] = queryCoordinates[i] + queryStartRadius;
	}

	this->elementCache.clear();
	std::vector<std::pair<coordT, objT>> returnVector;
	returnVector.reserve(count);

	coordT queryCurrentRadius = queryStartRadius;

	// Iterative expansion and execution of location queries is going to be performed
	// until the current radius is bigger than end radius (in the worst case). Only then
	// we know, that the whole tree was searched through.
	while (queryCurrentRadius < queryEndRadius)
	{
		// Query radius is incremented by a user defined value.
		queryCurrentRadius += step;

		// Query window is extended to the current radius.
		for (size_t i = 0; i < DIM; ++i)
		{
			queryBoundaries[2*i] = queryBoundaries[2*i] - step;
			queryBoundaries[2*i + 1] = queryBoundaries[2*i + 1] + step;
		}
		
		// Single location query represents one iteration. Because of distance query id, subsequent executions
		// don't consider the already searched through parts of the tree.
		this->mainNode->DistanceQuerySingleIteration(localDistanceQueryId, this->elementCache, queryCoordinates, queryBoundaries);

		// We are simulating circular query with a square one, so a distance to every single element has to be checked as well.
		// For that, the current radius value will be used (in squared form). The distance is also tightened by an epsilon
		// to make absolutely sure that no element from subsequent iteration could be closer than an element from this one.
		coordT queryCurrentSquaredRadius = (queryCurrentRadius - this->epsilon)*(queryCurrentRadius - this->epsilon);

		// Element cache contains heap data structure. Closest elements will be extracted from it one by one.
		// Extraction can proceed if:
		// 1) The requested element count hasn't been reached yet.
		// 2) There are elements in the cache.
		// 3) Next element to be extracted lies in the constrained radius.
		while (returnVector.size() < count && this->elementCache.empty() == false && this->elementCache.front()->squaredDistance <= queryCurrentSquaredRadius)
		{
			returnVector.emplace_back(this->elementCache.front()->squaredDistance, this->elementCache.front()->object);

			std::pop_heap(this->elementCache.begin(), this->elementCache.end(), [](QuadTreeElement<DIM, objT, coordT>* firstElement, QuadTreeElement<DIM, objT, coordT>* secondElement){return firstElement->squaredDistance > secondElement->squaredDistance;});
			this->elementCache.pop_back();
		}

		// If the requested element count has been reached, then the query can stop early.
		if (returnVector.size() == count)
		{
			break;
		}
	}

	// When the requested element count hasn't been reached yet and there are still
	// elements in the cache remaining, it means that the tree was fully iterated through, but
	// because of the radius constraint, the elements weren't put into the return vector.
	// But because we know now that there aren't any more elements to consider, the rest
	// can be transfered as well.
	while (this->elementCache.empty() == false && returnVector.size() < count)
	{
		returnVector.emplace_back(this->elementCache.front()->squaredDistance, this->elementCache.front()->object);

		std::pop_heap(this->elementCache.begin(), this->elementCache.end(), [](QuadTreeElement<DIM, objT, coordT>* firstElement, QuadTreeElement<DIM, objT, coordT>* secondElement){return firstElement->squaredDistance > secondElement->squaredDistance;});
		this->elementCache.pop_back();
	}

	return returnVector;
}

template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
template<typename... Args>
std::vector<std::pair<coordT, objT>>QuadTree<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>::QueryElementsWithinRadius(coordT radius, Args... coordinates)
{
	static_assert(sizeof... (coordinates) == DIM, "Distance radius query requires one coordinate for each axis/dimensions.");
	static_assert((... && std::is_same_v<Args, coordT>) == true, "All coordinate arguments must be of the coordinate type (coordT) specified in class template.");

	if (radius < static_cast<coordT>(0.0))
	{
		throw std::logic_error("Value of radius cannot be negative.");
	}

	// Coordinates are unpacked into an array.
	const std::array<coordT, DIM> queryCoordinates = {coordinates...};

	// Query boundaries are calculated based on the coordinates and the radius.
	std::array<coordT, 2*DIM> queryBoundaries = {};

	for (size_t i = 0; i < DIM; ++i)
	{
		// Epsilon is used here to make sure that even the elements that are
		// on the boundary of the circle are within the boundary of the query.
		queryBoundaries[2*i] = queryCoordinates[i] - radius - this->epsilon;
		queryBoundaries[2*i + 1] = queryCoordinates[i] + radius + this->epsilon;
	}

	// Element cache is reset.
	this->elementCache.clear();

	// Querying starts from the main node.
	this->mainNode->LocationQuery_Ordered(this->elementCache, queryCoordinates, queryBoundaries);

	// Elements are extracted from the cache from closest to farthest.
	// Their objects are copied into the return vector.
	std::vector<std::pair<coordT,objT>> returnVector;
	// Arbitrary reserve.
	returnVector.reserve(20);

	// Distance to the found elements will be compared against the given radius
	// extended by an epsilon (squared value). This way, we make sure that even
	// the elements on the border are going to be returned.
	coordT squaredRadius = (radius + this->epsilon)*(radius + this->epsilon);

	while (this->elementCache.empty() == false && this->elementCache.front()->squaredDistance <= squaredRadius)
	{
		returnVector.emplace_back(this->elementCache.front()->squaredDistance, this->elementCache.front()->object);

		std::pop_heap(this->elementCache.begin(), this->elementCache.end(), [](QuadTreeElement<DIM, objT, coordT>* firstElement, QuadTreeElement<DIM, objT, coordT>* secondElement){return firstElement->squaredDistance > secondElement->squaredDistance;});
		this->elementCache.pop_back();
	}

	return returnVector;
}

template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
template<typename... Args>
std::optional<std::pair<coordT, objT>> QuadTree<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>::QueryClosestElementAndDelete(coordT step, Args... coordinates)
{
	// Based on QueryClosestElement().

	static_assert(sizeof... (coordinates) == DIM, "Distance query requires one coordinate for each axis/dimensions.");
	static_assert((... && std::is_same_v<Args, coordT>) == true, "All coordinate arguments must be of the coordinate type (coordT) specified in class template.");

	if (step <= static_cast<coordT>(0.0))
	{
		throw std::logic_error("Value of interation step has to be positive.");
	}

	const std::array<coordT, DIM> queryCoordinates = {coordinates...};

	long long int localDistanceQueryId = this->distanceQueryId++;

	auto [queryStartRadius, queryEndRadius] = this->CalculateStartAndEndRadiusForDistanceQuery(queryCoordinates);

	std::array<coordT, 2*DIM> queryBoundaries = {};

	for (size_t i = 0; i < DIM; ++i)
	{
		queryBoundaries[2*i] = queryCoordinates[i] - queryStartRadius;
		queryBoundaries[2*i + 1] = queryCoordinates[i] + queryStartRadius;
	}

	this->elementCache.clear();
	std::optional<std::pair<coordT, objT>> returnOptionalPair;
	// Closest element found will be stored.
	QuadTreeElement<DIM, objT, coordT>* foundClosestElement = nullptr;

	coordT queryCurrentRadius = queryStartRadius;

	while (queryCurrentRadius < queryEndRadius)
	{
		queryCurrentRadius += step;

		for (size_t i = 0; i < DIM; ++i)
		{
			queryBoundaries[2*i] = queryBoundaries[2*i] - step;
			queryBoundaries[2*i + 1] = queryBoundaries[2*i + 1] + step;
		}
		
		this->mainNode->DistanceQuerySingleIteration(localDistanceQueryId, this->elementCache, queryCoordinates, queryBoundaries);

		coordT queryCurrentSquaredRadius = (queryCurrentRadius - this->epsilon)*(queryCurrentRadius - this->epsilon);

		if (this->elementCache.empty() == false && this->elementCache.front()->squaredDistance <= queryCurrentSquaredRadius)
		{
			returnOptionalPair.emplace(this->elementCache.front()->squaredDistance, std::move(this->elementCache.front()->object));
			// Closest found element is stored.
			foundClosestElement = this->elementCache.front();

			break;
		}
	}

	if (this->elementCache.empty() == false && returnOptionalPair.has_value() == false)
	{
		returnOptionalPair.emplace(this->elementCache.front()->squaredDistance, std::move(this->elementCache.front()->object));
		// Closest found element is stored.
		foundClosestElement = this->elementCache.front();
	}

	// Closest found element is deleted if it exists.
	if (foundClosestElement != nullptr)
	{
		if (this->mainNode->DeleteElement(this->elementCache, foundClosestElement->id, foundClosestElement->midpoint, foundClosestElement->boundaries, foundClosestElement->isPoint) == false)
		{
			throw std::logic_error("Internal error occured during deletion in QueryClosestElementAndDelete().");
		}

		this->elementPointerManager.Delete(foundClosestElement);
	}

	return returnOptionalPair;
}

template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
template<typename... Args>
std::vector<std::pair<coordT, objT>> QuadTree<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>::QueryNClosestElementsAndDelete(size_t count, coordT step, Args... coordinates)
{
	// Based on QueryNClosestElements().

	static_assert(sizeof... (coordinates) == DIM, "Distance query requires one coordinate for each axis/dimensions.");
	static_assert((... && std::is_same_v<Args, coordT>) == true, "All coordinate arguments must be of the coordinate type (coordT) specified in class template.");

	if (step <= static_cast<coordT>(0.0))
	{
		throw std::logic_error("Value of interation step has to be positive.");
	}

	const std::array<coordT, DIM> queryCoordinates = {coordinates...};

	long long int localDistanceQueryId = this->distanceQueryId++;

	auto [queryStartRadius, queryEndRadius] = this->CalculateStartAndEndRadiusForDistanceQuery(queryCoordinates);

	std::array<coordT, 2*DIM> queryBoundaries = {};

	for (size_t i = 0; i < DIM; ++i)
	{
		queryBoundaries[2*i] = queryCoordinates[i] - queryStartRadius;
		queryBoundaries[2*i + 1] = queryCoordinates[i] + queryStartRadius;
	}

	this->elementCache.clear();
	std::vector<std::pair<coordT, objT>> returnVector;
	returnVector.reserve(count);
	
	// Secondary element cache is created to keep track of elements to be deleted.
	std::vector<QuadTreeElement<DIM, objT, coordT>*> secondaryElementCache;
	secondaryElementCache.reserve(count);

	coordT queryCurrentRadius = queryStartRadius;

	while (queryCurrentRadius < queryEndRadius)
	{
		queryCurrentRadius += step;

		for (size_t i = 0; i < DIM; ++i)
		{
			queryBoundaries[2*i] = queryBoundaries[2*i] - step;
			queryBoundaries[2*i + 1] = queryBoundaries[2*i + 1] + step;
		}
		
		this->mainNode->DistanceQuerySingleIteration(localDistanceQueryId, this->elementCache, queryCoordinates, queryBoundaries);

		coordT queryCurrentSquaredRadius = (queryCurrentRadius - this->epsilon)*(queryCurrentRadius - this->epsilon);

		while (returnVector.size() < count && this->elementCache.empty() == false && this->elementCache.front()->squaredDistance <= queryCurrentSquaredRadius)
		{
			// Object are moved out of the element cache.
			returnVector.emplace_back(this->elementCache.front()->squaredDistance, std::move(this->elementCache.front()->object));
			// The used up element pointer is stored in the secondary element cache.
			secondaryElementCache.push_back(this->elementCache.front());

			std::pop_heap(this->elementCache.begin(), this->elementCache.end(), [](QuadTreeElement<DIM, objT, coordT>* firstElement, QuadTreeElement<DIM, objT, coordT>* secondElement){return firstElement->squaredDistance > secondElement->squaredDistance;});
			this->elementCache.pop_back();
		}

		if (returnVector.size() == count)
		{
			break;
		}
	}

	while (this->elementCache.empty() == false && returnVector.size() < count)
	{
		// Object are moved out of the element cache.
		returnVector.emplace_back(this->elementCache.front()->squaredDistance, std::move(this->elementCache.front()->object));
		// The used up element pointer is stored in the secondary element cache.
		secondaryElementCache.push_back(this->elementCache.front());

		std::pop_heap(this->elementCache.begin(), this->elementCache.end(), [](QuadTreeElement<DIM, objT, coordT>* firstElement, QuadTreeElement<DIM, objT, coordT>* secondElement){return firstElement->squaredDistance > secondElement->squaredDistance;});
		this->elementCache.pop_back();
	}

	// Each element inside the secondary element cache is deleted from the tree.
	for (QuadTreeElement<DIM, objT, coordT>* element : secondaryElementCache)
	{
		// This delete should never fail, because the elements were just retrieved from the tree.

		if (this->mainNode->DeleteElement(this->elementCache, element->id, element->midpoint, element->boundaries, element->isPoint) == false)
		{
			throw std::logic_error("Internal error occured during deletion in QueryNClosestElementsAndDelete().");
		}

		this->elementPointerManager.Delete(element);
	}

	return returnVector;
}

template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
template<typename... Args>
std::vector<std::pair<coordT, objT>> QuadTree<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>::QueryElementsWithinRadiusAndDelete(coordT radius, Args... coordinates)
{
	// Based on QueryElementsWithinRadius().

	static_assert(sizeof... (coordinates) == DIM, "Distance radius query requires one coordinate for each axis/dimensions.");
	static_assert((... && std::is_same_v<Args, coordT>) == true, "All coordinate arguments must be of the coordinate type (coordT) specified in class template.");

	if (radius < static_cast<coordT>(0.0))
	{
		throw std::logic_error("Value of radius cannot be negative.");
	}

	const std::array<coordT, DIM> queryCoordinates = {coordinates...};

	std::array<coordT, 2*DIM> queryBoundaries = {};

	for (size_t i = 0; i < DIM; ++i)
	{
		queryBoundaries[2*i] = queryCoordinates[i] - radius - this->epsilon;
		queryBoundaries[2*i + 1] = queryCoordinates[i] + radius + this->epsilon;
	}

	this->elementCache.clear();

	this->mainNode->LocationQuery_Ordered(this->elementCache, queryCoordinates, queryBoundaries);

	// Secondary element cache is created to keep track of elements to be deleted.
	std::vector<QuadTreeElement<DIM, objT, coordT>*> secondaryElementCache;
	secondaryElementCache.reserve(this->elementCache.size());

	std::vector<std::pair<coordT,objT>> returnVector;
	// Arbitrary reserve.
	returnVector.reserve(20);

	coordT squaredRadius = (radius + this->epsilon)*(radius + this->epsilon);

	while (this->elementCache.empty() == false && this->elementCache.front()->squaredDistance <= squaredRadius)
	{
		returnVector.emplace_back(this->elementCache.front()->squaredDistance, std::move(this->elementCache.front()->object));
		// The used up element pointer is stored in the secondary element cache.
		secondaryElementCache.push_back(this->elementCache.front());

		std::pop_heap(this->elementCache.begin(), this->elementCache.end(), [](QuadTreeElement<DIM, objT, coordT>* firstElement, QuadTreeElement<DIM, objT, coordT>* secondElement){return firstElement->squaredDistance > secondElement->squaredDistance;});
		this->elementCache.pop_back();
	}

	// Each element inside the secondary element cache is deleted from the tree.
	for (QuadTreeElement<DIM, objT, coordT>* element : secondaryElementCache)
	{
		// This delete should never fail, because the elements were just retrieved from the tree.

		if (this->mainNode->DeleteElement(this->elementCache, element->id, element->midpoint, element->boundaries, element->isPoint) == false)
		{
			throw std::logic_error("Internal error occured during deletion in QueryNClosestElementsAndDelete().");
		}

		this->elementPointerManager.Delete(element);
	}

	return returnVector;
}

template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
template<typename... Args>
std::vector<objT> QuadTree<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>::RayQuery(Args... coordinates)
{
	static_assert(sizeof... (coordinates) == DIM*2, "Ray query requires one coordinate for each axis/dimensions, twice.");
	static_assert(sizeof... (coordinates) == 2*2 || sizeof... (coordinates) == 3*2, "Ray query is only implemented for 2D and 3D.");
	static_assert((... && std::is_same_v<Args, coordT>) == true, "All coordinate arguments must be of the coordinate type (coordT) specified in class template.");

	// Coordinates of both points defining ray are unpacked into a single array.
	const std::array<coordT, 2*DIM> rayPointsCoordinates = {coordinates...};

	// Points defining ray cannot identical.
	bool pointsIdentical = true;
	for (size_t i = 0; i < DIM; ++i)
	{
		pointsIdentical &= rayPointsCoordinates[i] >= rayPointsCoordinates[DIM + i] - epsilon && rayPointsCoordinates[i] <= rayPointsCoordinates[DIM + i] + epsilon;
	}

	if (pointsIdentical == true)
	{
		throw std::logic_error("Points defining ray cannot be identical.");
	}

	// Element cache is reset.
	this->elementCache.clear();

	// 2D or 3D implementation is invoked.
	if constexpr (sizeof... (coordinates) == 2*2)
	{
		this->mainNode->RayQuery_2D(this->elementCache, rayPointsCoordinates[0], rayPointsCoordinates[1], rayPointsCoordinates[2], rayPointsCoordinates[3], this->epsilon);
	}
	else if constexpr (sizeof... (coordinates) == 3*2)
	{
		this->mainNode->RayQuery_3D(this->elementCache, rayPointsCoordinates[0], rayPointsCoordinates[1], rayPointsCoordinates[2], rayPointsCoordinates[3], rayPointsCoordinates[4], rayPointsCoordinates[5], this->epsilon);
	}
	else
	{
		// This should never be reached due to static_assert.
		return {};
	}

	// Object is copied from each returned element into a return vector.
	std::vector<objT> returnVector;
	// Arbitrary reserve.
	returnVector.reserve(20);

	for (QuadTreeElement<DIM, objT, coordT>* element : this->elementCache)
	{
		returnVector.push_back(element->object);
	}

	return returnVector;
}

template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
template<typename... Args>
std::vector<objT> QuadTree<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>::RayQueryAndDelete(Args... coordinates)
{
	static_assert(sizeof... (coordinates) == DIM*2, "Ray query requires one coordinate for each axis/dimensions, twice.");
	static_assert(sizeof... (coordinates) == 2*2 || sizeof... (coordinates) == 3*2, "Ray query is only implemented for 2D and 3D.");
	static_assert((... && std::is_same_v<Args, coordT>) == true, "All coordinate arguments must be of the coordinate type (coordT) specified in class template.");

	// Coordinates of both points defining ray are unpacked into a single array.
	const std::array<coordT, 2*DIM> rayPointsCoordinates = {coordinates...};

	// Points defining ray cannot identical.
	bool pointsIdentical = true;
	for (size_t i = 0; i < DIM; ++i)
	{
		pointsIdentical &= rayPointsCoordinates[i] >= rayPointsCoordinates[DIM + i] - epsilon && rayPointsCoordinates[i] <= rayPointsCoordinates[DIM + i] + epsilon;
	}

	if (pointsIdentical == true)
	{
		throw std::logic_error("Points defining ray cannot be identical.");
	}

	// Element cache is reset.
	this->elementCache.clear();

	// 2D or 3D implementation is invoked.
	if constexpr (sizeof... (coordinates) == 2*2)
	{
		this->mainNode->RayQuery_2D(this->elementCache, rayPointsCoordinates[0], rayPointsCoordinates[1], rayPointsCoordinates[2], rayPointsCoordinates[3], this->epsilon);
	}
	else if constexpr (sizeof... (coordinates) == 3*2)
	{
		this->mainNode->RayQuery_3D(this->elementCache, rayPointsCoordinates[0], rayPointsCoordinates[1], rayPointsCoordinates[2], rayPointsCoordinates[3], rayPointsCoordinates[4], rayPointsCoordinates[5], this->epsilon);
	}
	else
	{
		// This should never be reached due to static_assert.
		return {};
	}

	// Object is moved from each returned element into a return vector.
	std::vector<objT> returnVector;
	// Arbitrary reserve.
	returnVector.reserve(20);

	for (QuadTreeElement<DIM, objT, coordT>* element : this->elementCache)
	{
		returnVector.push_back(std::move(element->object));
	}

	// Finally, each element is deleted from the tree.

	// Internal element cache is copied, because it is used during deletion.
	std::vector<QuadTreeElement<DIM, objT, coordT>*> elementCacheCopy(this->elementCache);
	this->elementCache.clear();

	for (QuadTreeElement<DIM, objT, coordT>* element : elementCacheCopy)
	{
		// This delete should never fail, because the elements were just retrieved from the tree.

		if (this->mainNode->DeleteElement(this->elementCache, element->id, element->midpoint, element->boundaries, element->isPoint) == false)
		{
			throw std::logic_error("Internal error occured during deletion in RayQueryAndDelete().");
		}

		this->elementPointerManager.Delete(element);
	}

	return returnVector;
}

template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
void QuadTree<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>::Clear()
{
	// Boundaries are extracted from the existing main node.
	std::array<coordT, 2*DIM> boundariesArray = {};

	for (size_t i = 0; i < DIM; ++i)
	{
		boundariesArray[2*i] = this->mainNode->GetBoundary(i + 1, true);
		boundariesArray[2*i + 1] = this->mainNode->GetBoundary(i + 1, false);
	}

	// Main node is destructed.
	// Main node can be nullptr when the tree is default constructed or after a move.
	this->nodePointerManager.Delete_Nullptr(this->mainNode);

	// Main node is reconstructed.
	this->mainNode = this->nodePointerManager.Create(boundariesArray, &this->nodePointerManager, &this->elementPointerManager);
}

template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
size_t QuadTree<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>::Size() const
{
	return this->mainNode->GetElementCount();
}

template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
size_t QuadTree<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>::UniqueSize() const
{
	return this->mainNode->GetUniqueElementCount();
}

template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
coordT QuadTree<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>::GetBoundary(size_t dimension, bool isMin) const
{
	return this->mainNode->GetBoundary(dimension, isMin);
}

template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
bool QuadTree<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>::ValidateTree() const
{
	// Main node has the same properties as node on min side in all dimensions.
	std::array<bool, DIM> isOnMaxSide;
	isOnMaxSide.fill(false);

	// Recursively validate the node structructe starting from the main node.
	return this->mainNode->ValidateNode(isOnMaxSide);
}

template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
std::pair<coordT, coordT> QuadTree<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>::CalculateStartAndEndRadiusForDistanceQuery(const std::array<coordT, DIM>& queryCoordinates) const
{
	// Both variables start at zero, which is the minimum value.
	coordT queryStartRadius = static_cast<coordT>(0.0);
	coordT queryEndRadius = static_cast<coordT>(0.0);

	// Query coordinates can either be inside or outside the tree boundary.
	// The calculations will be different for each case.
	if (this->mainNode->DoesMidpointBelongToMainNode(queryCoordinates) == false)
	{
		// Query coordinates are outside the tree boundary. In this case, we want the first query to start
		// at the position where it touches the closest boundary of the tree and we want the last query to
		// end at the position where it touches the farthest boundary of the tree. This way, we are not
		// wasting time performing queries that don't interset the tree's main node.

		// Values are going to be calculated dimension by dimension.
		for (size_t i = 0; i < DIM; ++i)
		{
			// Both the min and max boundary are extracted.
			coordT minBoundary = this->mainNode->GetBoundary(i + 1, true);
			coordT maxBoundary = this->mainNode->GetBoundary(i + 1, false);

			// We are only going to consider dimensions, where the queryCoordinates are outside of the tree
			// because otherwise we would take into account values that don't make sense geometrically.
			// (We want distances to the boundary line segment, not to the line specified by the boundary line segment).
			if (queryCoordinates[i] < minBoundary || queryCoordinates[i] > maxBoundary)
			{
				// Squared distance to both boundaries is calculated.
				coordT minBoundarySquaredDistance = (minBoundary - queryCoordinates[i])*(minBoundary - queryCoordinates[i]);
				coordT maxBoundarySquaredDistance = (maxBoundary - queryCoordinates[i])*(maxBoundary - queryCoordinates[i]);

				// The squared distances are sorted.
				coordT closerBoundarySquaredDistance = static_cast<coordT>(0.0);
				coordT fartherBoundarySquaredDistance = static_cast<coordT>(0.0);

				if (minBoundarySquaredDistance < maxBoundarySquaredDistance)
				{
					closerBoundarySquaredDistance = minBoundarySquaredDistance;
					fartherBoundarySquaredDistance = maxBoundarySquaredDistance;
				}
				else
				{
					closerBoundarySquaredDistance = maxBoundarySquaredDistance;
					fartherBoundarySquaredDistance = minBoundarySquaredDistance;
				}

				// We are trying to find the biggest value for each variable, but for start radius, only
				// the closer boundary is considered and for end radius only the farther boundary is considered.
				queryStartRadius = closerBoundarySquaredDistance > queryStartRadius ? closerBoundarySquaredDistance : queryStartRadius;
				queryEndRadius = fartherBoundarySquaredDistance > queryEndRadius ? fartherBoundarySquaredDistance : queryEndRadius;
			}
		}

		// Actual distance is calculated.
		queryStartRadius = std::sqrt(queryStartRadius);
		queryEndRadius = std::sqrt(queryEndRadius);

		// Because the distance query is working with circles and touch is considered as an intersection,
		// there has to be a handling of numerical errors. Each boundary will be moved by a small epsilon value
		// to make sure that no element is missed.
		queryStartRadius = queryStartRadius - this->epsilon < static_cast<coordT>(0.0) ? static_cast<coordT>(0.0) : queryStartRadius - this->epsilon;
		queryEndRadius = queryEndRadius + this->epsilon;

	}
	else
	{
		// Query coordinates are inside the tree boundary. In this case, the query start radius is
		// always zero. The query end radius is going to be the farthest tree boundary in any direction.

		for (size_t i = 0; i < DIM; ++i)
		{
			coordT minBoundary = this->mainNode->GetBoundary(i + 1, true);
			coordT maxBoundary = this->mainNode->GetBoundary(i + 1, false);

			coordT minBoundarySquaredDistance = (minBoundary - queryCoordinates[i])*(minBoundary - queryCoordinates[i]);
			coordT maxBoundarySquaredDistance = (maxBoundary - queryCoordinates[i])*(maxBoundary - queryCoordinates[i]);

			queryEndRadius = minBoundarySquaredDistance > queryEndRadius ? minBoundarySquaredDistance : queryEndRadius;
			queryEndRadius = maxBoundarySquaredDistance > queryEndRadius ? maxBoundarySquaredDistance : queryEndRadius;
		}

		// Actual distance is calculated.
		queryEndRadius = std::sqrt(queryEndRadius);

		// Query start radius is already at a minimum, but end radius is still moved by epsilon.
		queryEndRadius = queryEndRadius + this->epsilon;
	}

	return {queryStartRadius, queryEndRadius};
}

//---------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------

// Single element contained in the quad tree structure node.
template<size_t DIM, typename objT, typename coordT>
class QuadTreeElement
{
public:

	// Id that uniquely identifies stored object.
	long long int id;

	// Flag that indicates that element represents point object.
	bool isPoint;

	// Point: coordinates. ND structure: coordinates of a midpoint.
	// x, y, z, ...
	std::array<coordT, DIM> midpoint;
	// Point: min and max are the same as midpoint for specific axis/dimension. ND structure: boundaries of the element. Two for each axis/dimension.
	// minX, maxX, minY, maxY, ...
	std::array<coordT, 2*DIM> boundaries;

	// Stored object.
	objT object;

	// Id of last distance query performed on this element. Starts at zero.
	long long int distanceQueryId;

	// Squared distance to this element. Used in distance querying.
	coordT squaredDistance;

	// Default constructor.
	QuadTreeElement();
	// Constructor.
	template<typename T>
	QuadTreeElement(long long int paramId, T&& paramObject, const std::array<coordT, DIM>& paramMidpoint, const std::array<coordT, 2*DIM>& paramBoundaries, bool paramIsPoint);
	// Destructor.
	~QuadTreeElement();

	// Copy/move constructor/assignment is not needed.
	QuadTreeElement(const QuadTreeElement& edge) = delete;
	QuadTreeElement(QuadTreeElement&& edge) noexcept = delete;
	QuadTreeElement& operator=(const QuadTreeElement& edge) = delete;
	QuadTreeElement& operator=(QuadTreeElement&& edge) noexcept = delete;
};

template<size_t DIM, typename objT, typename coordT>
QuadTreeElement<DIM, objT, coordT>::QuadTreeElement()
{
	this->id = 0;

	this->isPoint = true;

	this->midpoint = {};
	this->boundaries = {};

	this->distanceQueryId = 0;
	this->squaredDistance = static_cast<coordT>(0.0);
}

template<size_t DIM, typename objT, typename coordT>
template<typename T>
QuadTreeElement<DIM, objT, coordT>::QuadTreeElement(long long int paramId, T&& paramObject, const std::array<coordT, DIM>& paramMidpoint, const std::array<coordT, 2*DIM>& paramBoundaries, bool paramIsPoint) : midpoint(paramMidpoint), boundaries(paramBoundaries), object(std::forward<T>(paramObject))
{
	this->id = paramId;

	this->isPoint = paramIsPoint;

	this->distanceQueryId = 0;
	this->squaredDistance = static_cast<coordT>(0.0);
}

template<size_t DIM, typename objT, typename coordT>
QuadTreeElement<DIM, objT, coordT>::~QuadTreeElement()
{
}

//---------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------

// Quad tree node.
template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
class QuadTreeNode
{
private:

	// Pointers to pointer managers.
	PointerManager_Unlimited<QuadTreeNode<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>>* nodePointerManager;
	PointerManager_Unlimited<QuadTreeElement<DIM, objT, coordT>>* elementPointerManager;

	// Depth of this node.
	int depth;

	// Base boundaries of the node. Two for each axis/dimension.
	// minX, maxX, minY, maxY, ...
	std::array<coordT, 2*DIM> boundaries;

	// Boundaries extended by the elements that are stored inside.
	std::array<coordT, 2*DIM> extendedBoundaries;

	// Coordinates of a midpoint (based on boundaries).
	// x, y, z, ...
	std::array<coordT, DIM> midpoint;

	// False when node is a leaf node.
	bool divided;

	// Number of stored elements in this node.
	// If divided == false, then elementCount = storedElements.size().
	// If divided == true, then elementCount is the sum of all elements in subnodes.
	size_t elementCount;

	// Number of stored elements with unique coordinates.
	// Works in the similar way as elementCount and is updated at the same time.
	size_t uniqueElementCount;

	// Elements stored in this node. Empty if divided == true.
	// Represented as a vector because if coordinates are not unique, then maximumCapacity can be exceeded.
	std::vector<QuadTreeElement<DIM, objT, coordT>*> storedElements;

	// Pointer to the node above this one.
	QuadTreeNode<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>* parentNode;

	// Array of pointers to subnodes. The number of subnodes is based on the dimension (2^DIM).
	// Ordering is the following:
	// 1D)
	// (minX, midpointX)
	// (midpointX, maxX)
	// 2D)
	// (minX, midpointX), (minY, midpointY)
	// (midpointX, maxX), (minY, midpointY)
	// (minX, midpointX), (midpointY, maxY)
	// (midpointX, maxX), (midpointY, maxY)
	// ...
	std::array<QuadTreeNode<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>*, Pow<2, DIM>::value> subnodes;

	// Id of last distance query performed on this node. Starts at zero.
	long long int distanceQueryId;

public:

	// Default constructor.
	QuadTreeNode();
	// Main node constructor.
	QuadTreeNode(const std::array<coordT, 2*DIM>& paramBoundaries, PointerManager_Unlimited<QuadTreeNode<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>>* paramNodePointerManager, PointerManager_Unlimited<QuadTreeElement<DIM, objT, coordT>>* paramElementPointerManager);
	// Not main node constructor.
	QuadTreeNode(const std::array<coordT, 2*DIM>& paramBoundaries, QuadTreeNode<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>* paramParentNode, PointerManager_Unlimited<QuadTreeNode<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>>* paramNodePointerManager, PointerManager_Unlimited<QuadTreeElement<DIM, objT, coordT>>* paramElementPointerManager);
	// Special constructor used inside copy constructor of QuadTree class.
	// Constructed node will be the same as nodeToCopy.
	// The reason that this isn't a copy constructor as well is because we need to use pointer managers during construction.
	QuadTreeNode(QuadTreeNode<DIM, MAX_CAP, objT, MAX_DEPTH, coordT> const* nodeToCopy, QuadTreeNode<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>* parentNodeForCopiedNode, PointerManager_Unlimited<QuadTreeNode<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>>* paramNodePointerManager, PointerManager_Unlimited<QuadTreeElement<DIM, objT, coordT>>* paramElementPointerManager);
	// Destructor.
	~QuadTreeNode();

	// Copy/move constructor/assignment is not needed.
	QuadTreeNode(const QuadTreeNode& edge) = delete;
	QuadTreeNode(QuadTreeNode&& edge) noexcept = delete;
	QuadTreeNode& operator=(const QuadTreeNode& edge) = delete;
	QuadTreeNode& operator=(QuadTreeNode&& edge) noexcept = delete;

	// Updates this node's and its subnodes' pointer managers.
	void UpdatePointerManagers(PointerManager_Unlimited<QuadTreeNode<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>>* paramNodePointerManager, PointerManager_Unlimited<QuadTreeElement<DIM, objT, coordT>>* paramElementPointerManager);

	// Test if a midpoint belongs to this node, but only if this node is the main node.
	bool DoesMidpointBelongToMainNode(const std::array<coordT, DIM>& paramMidpoint) const;

	// Inserts an element into the tree structure belonging to this node.
	bool InsertElement(QuadTreeElement<DIM, objT, coordT>* newElement);

	// Deletes an element from the tree structure belonging to this node.
	bool DeleteElement(std::vector<QuadTreeElement<DIM, objT, coordT>*>& elementCache, long long int id, const std::array<coordT, DIM>& paramMidpoint, const std::array<coordT, 2*DIM>& paramBoundaries, bool isPoint);

	// Relocates an element inside the tree structure belonging to this node.
	bool RelocateElement(std::vector<QuadTreeElement<DIM, objT, coordT>*>& elementCache, long long int id, const std::array<coordT, DIM>& currentMidpoint, const std::array<coordT, 2*DIM>& currentBoundaries, const std::array<coordT, DIM>& newMidpoint, const std::array<coordT, 2*DIM>& newBoundaries, bool isPoint);

	// Finds all elements intersecting location specified by query window. Unordered version.
	void LocationQuery_Unordered(std::vector<QuadTreeElement<DIM, objT, coordT>*>& elementCache, const std::array<coordT, 2*DIM>& queryBoundaries) const;

	// Finds all elements intersecting location specified by query window. Ordered using heap.
	void LocationQuery_Ordered(std::vector<QuadTreeElement<DIM, objT, coordT>*>& elementCache, const std::array<coordT, DIM>& queryCoordinates, const std::array<coordT, 2*DIM>& queryBoundaries) const;

	// Finds every not yet found element (based on distanceSearchId) that is inside the query window.
	// elementCache has to represent a min heap (based on squared distances). When elements are added into it, the heap is maintained.
	void DistanceQuerySingleIteration(long long int paramDistanceQueryId, std::vector<QuadTreeElement<DIM, objT, coordT>*>& elementCache, const std::array<coordT, DIM>& queryCoordinates, const std::array<coordT, 2*DIM>& queryBoundaries);

	// Finds all elements intersecting 2D ray.
	void RayQuery_2D(std::vector<QuadTreeElement<DIM, objT, coordT>*>& elementCache, coordT rayFirstVertexX, coordT rayFirstVertexY, coordT raySecondVertexX, coordT raySecondVertexY, coordT epsilon) const;

	// Finds all elements intersecting 3D ray.
	void RayQuery_3D(std::vector<QuadTreeElement<DIM, objT, coordT>*>& elementCache, coordT rayFirstVertexX, coordT rayFirstVertexY, coordT rayFirstVertexZ, coordT raySecondVertexX, coordT raySecondVertexY, coordT raySecondVertexZ, coordT epsilon) const;

	// Returns a value of a specific boundary of a specific dimension.
	coordT GetBoundary(size_t dimension, bool isMin) const;

	// Returns the element count.
	size_t GetElementCount() const;

	// Returns the unique element count.
	size_t GetUniqueElementCount() const;

	// Validates tree structure belonging to this node for debug purposes.
	bool ValidateNode(std::array<bool, DIM> isOnMaxSide) const;

private:

	// Calculates the midpoint based on boundaries.
	void CalculateMidpoint();

	// Finds subnode index for element belonging to this node.
	size_t FindSubnodeIndexForCoordinates(const std::array<coordT, DIM>& coordinates) const;

	// Finds already existing leaf node that the coordinates belongs to.
	QuadTreeNode<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>* FindExistingLeafNodeForCoordinates(const std::array<coordT, DIM>& coordinates);

	// Test if there already is an element inside this node at the coordinates of the given element.
	bool DoesElementAlreadyExistAtCoordinates(QuadTreeElement<DIM, objT, coordT>* element) const;

	// Splits this node and moves its element into their respective subnodes.
	// After split, all boundaries for this node and its subnodes are valid.
	void SplitNode();

	// Recalculates extended boundaries after new element is inserted into this node or its subnodes.
	void RecalculateExtendedBoundariesAfterElementInsertion(QuadTreeElement<DIM, objT, coordT>* element);

	// Inserts an element into this leaf node. Leaf node can be split in the process.
	// All variables inside this node and its potential subnotes are valid after the insertion.
	void InsertElementIntoThisLeafNode(QuadTreeElement<DIM, objT, coordT>* element, int& elementCountChange, int& uniqueElementCountChange);

	// After element is inserted into a (before insertion) leaf node, this function then can be called
	// on its parent to update nodes above this one.
	void PropagateChangesUpAfterElementInsertion(QuadTreeNode<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>* stopNode, QuadTreeElement<DIM, objT, coordT>* element, const int& elementCountChange, const int& uniqueElementCountChange);

	// Recalculates extended boundaries after an element is removed.
	void RecalculateExtendedBoundariesAfterElementRemoval(bool isPoint);

	// Removes an element from this leaf node. All variables inside this node are valid after removal.
	bool RemoveElementFromThisLeafNode(std::vector<QuadTreeElement<DIM, objT, coordT>*>& elementCache, long long int id, const std::array<coordT, DIM>& paramMidpoint, const std::array<coordT, 2*DIM>& paramBoundaries, bool isPoint, int& elementCountChange, int& uniqueElementCountChange);

	// Tests if not a leaf node can be merged.
	bool CanNodeBeMerged() const;

	// Recalculates extended bundaries after a node is merged.
	void RecalculateExtendedBoundariesAfterMerge();

	// Merges not a leaf node. Subnodes are dealocated.
	void MergeNode();

	// After an element is removed and merging of nodes is done, this then
	// can be called to update nodes above up to the stop node.
	void PropagateChangesUpAfterElementRemoval(QuadTreeNode<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>* stopNode, const int& elementCountChange, const int& uniqueElementCountChange, bool isPoint);

	// Finds the leaf node of current coordinates together with the lowest node shared by the current and new coordinates.
	void FindExistingLeafNodeAndReinsertNodeForCoordinates(QuadTreeNode<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>*& existingLeafNode, QuadTreeNode<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>*& reinsertNode, const std::array<coordT, DIM>& currentCoordinates, const std::array<coordT, DIM>& newCoordinates);
};

template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
QuadTreeNode<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>::QuadTreeNode()
{
	this->nodePointerManager = nullptr;
	this->elementPointerManager = nullptr;

	this->depth = -1;

	this->boundaries = {};
	this->extendedBoundaries = {};

	this->CalculateMidpoint();

	this->divided = false;

	this->elementCount = 0;
	this->uniqueElementCount = 0;

	this->parentNode = nullptr;

	this->subnodes = {};

	this->distanceQueryId = 0;
}

template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
QuadTreeNode<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>::QuadTreeNode(const std::array<coordT, 2*DIM>& paramBoundaries, PointerManager_Unlimited<QuadTreeNode<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>>* paramNodePointerManager, PointerManager_Unlimited<QuadTreeElement<DIM, objT, coordT>>* paramElementPointerManager) : boundaries(paramBoundaries), extendedBoundaries(paramBoundaries)
{
	this->nodePointerManager = paramNodePointerManager;
	this->elementPointerManager = paramElementPointerManager;

	// Main node has depth equal to 0.
	this->depth = 0;

	this->CalculateMidpoint();

	// Main node is not divided by default.
	this->divided = false;

	this->elementCount = 0;
	this->uniqueElementCount = 0;

	// Parent of main node is nullptr.
	this->parentNode = nullptr;

	// No subnodes by default.
	this->subnodes = {};

	this->distanceQueryId = 0;
}

template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
QuadTreeNode<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>::QuadTreeNode(const std::array<coordT, 2*DIM>& paramBoundaries, QuadTreeNode<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>* paramParentNode, PointerManager_Unlimited<QuadTreeNode<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>>* paramNodePointerManager, PointerManager_Unlimited<QuadTreeElement<DIM, objT, coordT>>* paramElementPointerManager) : boundaries(paramBoundaries), extendedBoundaries(paramBoundaries)
{
	this->nodePointerManager = paramNodePointerManager;
	this->elementPointerManager = paramElementPointerManager;

	// Depth of new node is the parent's value + 1.
	this->depth = paramParentNode->depth + 1;

	this->CalculateMidpoint();

	// Node is not divided by default.
	this->divided = false;

	this->elementCount = 0;
	this->uniqueElementCount = 0;

	this->parentNode = paramParentNode;

	// No subnodes by default.
	this->subnodes = {};

	this->distanceQueryId = 0;
}

template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
QuadTreeNode<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>::QuadTreeNode(QuadTreeNode<DIM, MAX_CAP, objT, MAX_DEPTH, coordT> const* nodeToCopy, QuadTreeNode<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>* parentNodeForCopiedNode, PointerManager_Unlimited<QuadTreeNode<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>>* paramNodePointerManager, PointerManager_Unlimited<QuadTreeElement<DIM, objT, coordT>>* paramElementPointerManager) : boundaries(nodeToCopy->boundaries), extendedBoundaries(nodeToCopy->extendedBoundaries), midpoint(nodeToCopy->midpoint)
{
	this->nodePointerManager = paramNodePointerManager;
	this->elementPointerManager = paramElementPointerManager;

	this->depth = nodeToCopy->depth;
	this->divided = nodeToCopy->divided;
	this->elementCount = nodeToCopy->elementCount;
	this->uniqueElementCount = nodeToCopy->uniqueElementCount;

	this->parentNode = parentNodeForCopiedNode;

	if (nodeToCopy->divided == false)
	{
		for (size_t i = 0; i < nodeToCopy->storedElements.size(); ++i)
		{
			this->storedElements.push_back(this->elementPointerManager->Create(nodeToCopy->storedElements[i]->id, nodeToCopy->storedElements[i]->object, nodeToCopy->storedElements[i]->midpoint, nodeToCopy->storedElements[i]->boundaries, nodeToCopy->storedElements[i]->isPoint));
		}

		this->subnodes = {};
	}
	else
	{
		for (size_t i = 0; i < Pow<2, DIM>::value; ++i)
		{
			this->subnodes[i] = this->nodePointerManager->Create(nodeToCopy->subnodes[i], this, this->nodePointerManager, this->elementPointerManager);
		}
	}

	this->distanceQueryId = nodeToCopy->distanceQueryId;
}

template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
QuadTreeNode<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>::~QuadTreeNode()
{
	if (this->divided == true)
	{
		// All subnodes are deallocated for divided nodes.

		for (size_t i = 0; i < Pow<2, DIM>::value; ++i)
		{
			this->nodePointerManager->Delete(this->subnodes[i]);
		}
	}
	else
	{
		// Elements are deallocated for leaf nodes.

		for (size_t i = 0; i < this->storedElements.size(); ++i)
		{
			this->elementPointerManager->Delete(this->storedElements[i]);
		}
	}
}

template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
void QuadTreeNode<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>::UpdatePointerManagers(PointerManager_Unlimited<QuadTreeNode<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>>* paramNodePointerManager, PointerManager_Unlimited<QuadTreeElement<DIM, objT, coordT>>* paramElementPointerManager)
{
	this->nodePointerManager = paramNodePointerManager;
	this->elementPointerManager = paramElementPointerManager;

	if (this->divided == true)
	{
		for (size_t i = 0; i < Pow<2, DIM>::value; ++i)
		{
			this->subnodes[i]->UpdatePointerManagers(paramNodePointerManager, paramElementPointerManager);
		}
	}
}

template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
bool QuadTreeNode<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>::DoesMidpointBelongToMainNode(const std::array<coordT, DIM>& paramMidpoint) const
{
	if (this->parentNode == nullptr)
	{
		for (size_t i = 0; i < DIM; ++i)
		{
			if (paramMidpoint[i] < this->boundaries[2*i] || paramMidpoint[i] > this->boundaries[2*i + 1])
			{
				return false;
			}
		}

		return true;
	}

	// Using this function on anything other than the main node is invalid because of elements laying
	// exactly on the cross boundaries between the nodes. False is returned by default.
	return false;
}

template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
bool QuadTreeNode<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>::InsertElement(QuadTreeElement<DIM, objT, coordT>* newElement)
{
	// First, an already existing leaf node where the new element belongs is found.
	QuadTreeNode<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>* existingNode = this->FindExistingLeafNodeForCoordinates(newElement->midpoint);

	// Insert the element into the found leaf node.
	int elementCountChange = 0;
	int uniqueElementCountChange = 0;
	existingNode->InsertElementIntoThisLeafNode(newElement, elementCountChange, uniqueElementCountChange);

	// Propagate the changes from the found leaf node (by now it could be split) upwards starting from its parent node.
	// IMPORTANT - The propagation has to be performed up to the main node, not just to this one (the original insertion node).
	// Otherwise the node boundaries could become invalid.
	existingNode->PropagateChangesUpAfterElementInsertion(nullptr, newElement, elementCountChange, uniqueElementCountChange);

	// Inserting an element into a node cannot fail.
	return true;
}

template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
bool QuadTreeNode<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>::DeleteElement(std::vector<QuadTreeElement<DIM, objT, coordT>*>& elementCache, long long int id, const std::array<coordT, DIM>& paramMidpoint, const std::array<coordT, 2*DIM>& paramBoundaries, bool isPoint)
{
	// First, an already existing leaf node where the element should be located is found.
	QuadTreeNode<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>* existingNode = this->FindExistingLeafNodeForCoordinates(paramMidpoint);

	// Element/s are deleted from the found leaf node.
	int elementCountChange = 0;
	int uniqueElementCountChange = 0;
	const bool removalSuccessful = existingNode->RemoveElementFromThisLeafNode(elementCache, id, paramMidpoint, paramBoundaries, isPoint, elementCountChange, uniqueElementCountChange);

	// If removal was not successful, false is returned.
	if (removalSuccessful == false)
	{
		return false;
	}

	// Now the merging phase can start.

	// We start with the parent node of existing leaf node from which the element was removed.
	QuadTreeNode<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>* currentNode = existingNode->parentNode;

	// Nodes can be merged up to the main node.
	while (currentNode != nullptr)
	{
		// If node can be merged it will be. Otherwise, the merging is stopped.
		if (currentNode->CanNodeBeMerged() == true)
		{
			currentNode->MergeNode();
		}
		else
		{
			break;
		}

		currentNode = currentNode->parentNode;
	}

	// Propagation phase.

	// Merging phase most likely stops before the main node, but the element count changes
	// have to be propagated up to the main node.
	if (currentNode != nullptr)
	{
		currentNode->PropagateChangesUpAfterElementRemoval(nullptr, elementCountChange, uniqueElementCountChange, isPoint);
	}

	return true;
}

template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
bool QuadTreeNode<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>::RelocateElement(std::vector<QuadTreeElement<DIM, objT, coordT>*>& elementCache, long long int id, const std::array<coordT, DIM>& currentMidpoint, const std::array<coordT, 2*DIM>& currentBoundaries, const std::array<coordT, DIM>& newMidpoint, const std::array<coordT, 2*DIM>& newBoundaries, bool isPoint)
{
	// Current element count changes for removal.
	int removalElementCountChange = 0;
	int removalUniqueElementCountChange = 0;

	// Existing leaf node that should contain element with current midpoint.
	QuadTreeNode<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>* existingLeafNode = nullptr;
	// Lowest shared node containing both the current and new midpoint.
	QuadTreeNode<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>* reinsertNode = nullptr;

	// Above mentioned nodes are found. Both will always exist.
	this->FindExistingLeafNodeAndReinsertNodeForCoordinates(existingLeafNode, reinsertNode, currentMidpoint, newMidpoint);

	// Element is removed from the found leaf node.
	const bool removalSuccessful = existingLeafNode->RemoveElementFromThisLeafNode(elementCache, id, currentMidpoint, currentBoundaries, isPoint, removalElementCountChange, removalUniqueElementCountChange);

	// If removal was unsuccessful, then the relocation is unsuccessful as well.
	if (removalSuccessful == false)
	{
		return false;
	}

	// Procsessing is different if the current and new midpoint belong to the same leaf node.
	if (existingLeafNode == reinsertNode)
	{
		// No postprocessing after removal is necessary.

		// Reinsert is done in the same leaf node as removal.
		QuadTreeNode<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>* existingReinsertLeafNode = existingLeafNode;

		// Insertion is done in similar way as in the other general case.

		int insertionElementCountChange = 0;
		int insertionUniqueElementCountChange = 0;

		for (size_t i = 0; i < elementCache.size(); ++i)
		{
			int insertionElementSubcountChange = 0;
			int insertionUniqueElementSubcountChange = 0;

			elementCache[i]->midpoint = newMidpoint;
			elementCache[i]->boundaries = newBoundaries;

			if (existingReinsertLeafNode->divided == true)
			{
				existingReinsertLeafNode = existingReinsertLeafNode->FindExistingLeafNodeForCoordinates(newMidpoint);
			}

			existingReinsertLeafNode->InsertElementIntoThisLeafNode(elementCache[i], insertionElementSubcountChange, insertionUniqueElementSubcountChange);
			
			// Propagation after insertion still has to be done, but only for insertions that
			// come after the first one which has split the original leaf node.
			if (existingReinsertLeafNode != reinsertNode)
			{
				// Even reinsertNode should be valid, so the stop node is set to its parent node.
				existingReinsertLeafNode->PropagateChangesUpAfterElementInsertion(reinsertNode->parentNode, elementCache[i], insertionElementSubcountChange, insertionUniqueElementSubcountChange);
			}

			insertionElementCountChange += insertionElementSubcountChange;
			insertionUniqueElementCountChange += insertionUniqueElementSubcountChange;
		}

		// Leaf node is now in a valid state, so the merging and propagation starts from its parent.
		QuadTreeNode<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>* currentNode = existingLeafNode->parentNode;

		// Merging and propagation is done in the same way as in the other general case.

		int relocationElementCountChange = removalElementCountChange + insertionElementCountChange;
		int relocationUniqueElementCountChange = removalUniqueElementCountChange + insertionUniqueElementCountChange;

		while (currentNode != nullptr)
		{
			if (currentNode->CanNodeBeMerged() == true)
			{
				currentNode->MergeNode();
			}
			else
			{
				break;
			}

			currentNode = currentNode->parentNode;
		}

		if (currentNode != nullptr)
		{
			currentNode->PropagateChangesUpAfterElementRemoval(nullptr, relocationElementCountChange, relocationUniqueElementCountChange, isPoint);
		}
	}
	else
	{
		// Leaf node was already handled, so now we start the processing with its parent.
		QuadTreeNode<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>* currentNode = existingLeafNode->parentNode;

		// Element was removed. Nodes can be merged up to the reinsert node.
		while (currentNode != reinsertNode)
		{
			// If node can be merged, it will be. Otherwise, the merging stops.
			if (currentNode->CanNodeBeMerged() == true)
			{
				currentNode->MergeNode();
			}
			else
			{
				break;
			}

			currentNode = currentNode->parentNode;
		}

		// Merging phase most likely stops before the reinsert node, but the element count changes
		// have to be propagated up to the reinsert node. The reinsert node is untouched.
		if (currentNode != reinsertNode)
		{
			currentNode->PropagateChangesUpAfterElementRemoval(reinsertNode, removalElementCountChange, removalUniqueElementCountChange, isPoint);
		}

		// Now the reinsertion can begin.

		// Existing leaf node for the new midpoint is found. Search starts from the reinsert node.
		QuadTreeNode<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>* existingReinsertLeafNode = reinsertNode->FindExistingLeafNodeForCoordinates(newMidpoint);

		// Multiple elements can possibly be moved at once if id was not given.
		// All of them are going to be reinserted one by one.

		int insertionElementCountChange = 0;
		int insertionUniqueElementCountChange = 0;

		for (size_t i = 0; i < elementCache.size(); ++i)
		{
			int insertionElementSubcountChange = 0;
			int insertionUniqueElementSubcountChange = 0;

			// Existing element's midpoint and boundaries will be updated to the new values.
			elementCache[i]->midpoint = newMidpoint;
			elementCache[i]->boundaries = newBoundaries;

			// What could happen is that after the first element is inserted, the leaf node splits.
			// That can only happen once because all of the moved elements must lie at the same coordinates.
			// In that case, the leaf node search is performed again on the now split leaf node.
			if (existingReinsertLeafNode->divided == true)
			{
				existingReinsertLeafNode = existingReinsertLeafNode->FindExistingLeafNodeForCoordinates(newMidpoint);
			}

			// Element is inserted.
			existingReinsertLeafNode->InsertElementIntoThisLeafNode(elementCache[i], insertionElementSubcountChange, insertionUniqueElementSubcountChange);

			// The element counts are propagated up to the reinsert node. The reinsert node is untouched.
			existingReinsertLeafNode->PropagateChangesUpAfterElementInsertion(reinsertNode, elementCache[i], insertionElementSubcountChange, insertionUniqueElementSubcountChange);

			// Subcounts are added to the main insertion counts.
			insertionElementCountChange += insertionElementSubcountChange;
			insertionUniqueElementCountChange += insertionUniqueElementSubcountChange;
		}

		// At this point, the reinsert node is still untouched, but the subnode containing old midpoint
		// and the subnode containing new midpoint are fully handled and valid.
		// Several things to mention. Even though the total number of unique elements in the reinsert node
		// can decrease during relocation, it is not necessary to try merging from the leaf node
		// where the element was reinserted. Let's say that the reinsert node has two elements (MAX_CAP == 1)
		// and the subnodes are further divided because the two elements are close to each other.
		// If one element is going to be relocated on top the other one, the whole tree is going to collapse.
		// But this can only happen if both elements are in the leafs next to each other (they share parent node),
		// so it is going to be handled by the merge starting from the leaf node containing current midpoint.
		// They cannot be further apart, because then the nodes wouldn't be split in the first place.
		// So now the handling of the reinsert node (and the nodes above) can begin.
		// Because an element was removed from the reinsert node, the whole process is going to be
		// treated as a removal. First a merging will be tried followed by a propagation phase
		// (removal -> boundaries are fully recalculated based on the subnodes).

		// Merging starts at the reinsert node.
		currentNode = reinsertNode;

		// Total count changes after relocation.
		int relocationElementCountChange = removalElementCountChange + insertionElementCountChange;
		int relocationUniqueElementCountChange = removalUniqueElementCountChange + insertionUniqueElementCountChange;

		// Nodes can be merged up to the main node.
		while (currentNode != nullptr)
		{
			// If node can be merged, it will be. Otherwise, the merging stops.
			if (currentNode->CanNodeBeMerged() == true)
			{
				currentNode->MergeNode();
			}
			else
			{
				break;
			}

			currentNode = currentNode->parentNode;
		}

		// Merging phase most likely stops before the main node, but the element count changes
		// have to be propagated up to the main node.
		if (currentNode != nullptr)
		{
			currentNode->PropagateChangesUpAfterElementRemoval(nullptr, relocationElementCountChange, relocationUniqueElementCountChange, isPoint);
		}
	}

	// Removal can fail, but reinsertion cannot, so true is always returned.
	return true;
}

template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
void QuadTreeNode<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>::LocationQuery_Unordered(std::vector<QuadTreeElement<DIM, objT, coordT>*>& elementCache, const std::array<coordT, 2*DIM>& queryBoundaries) const
{
	if (DoAxisAlignedNDVolumesIntersect<DIM>(queryBoundaries, this->extendedBoundaries) == true)
	{
		// Query window collides with the extended boundaries of this node.

		if (this->divided == true)
		{
			// Node is divided. Query is going to be delegated to its subnodes.

			for (size_t i = 0; i < Pow<2, DIM>::value; ++i)
			{
				this->subnodes[i]->LocationQuery_Unordered(elementCache, queryBoundaries);
			}
		}
		else
		{
			// Node is not divided. Query window is going to be checked against stored elements.

			for (size_t i = 0; i < this->storedElements.size(); ++i)
			{
				// Even if element is a point, the check has the same complexity (two comparisons per dimension) as ND element.
				// And because point element's boundaries field is filled, we can use the same funtion as for ND element.
				if (DoAxisAlignedNDVolumesIntersect<DIM>(queryBoundaries, this->storedElements[i]->boundaries) == true)
				{
					elementCache.push_back(this->storedElements[i]);
				}
			}
		}
	}
}

template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
void QuadTreeNode<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>::LocationQuery_Ordered(std::vector<QuadTreeElement<DIM, objT, coordT>*>& elementCache, const std::array<coordT, DIM>& queryCoordinates, const std::array<coordT, 2*DIM>& queryBoundaries) const
{
	// Based on LocationQuery_Unordered().

	if (DoAxisAlignedNDVolumesIntersect<DIM>(queryBoundaries, this->extendedBoundaries) == true)
	{
		if (this->divided == true)
		{
			for (size_t i = 0; i < Pow<2, DIM>::value; ++i)
			{
				this->subnodes[i]->LocationQuery_Ordered(elementCache, queryCoordinates, queryBoundaries);
			}
		}
		else
		{
			for (size_t i = 0; i < this->storedElements.size(); ++i)
			{
				if (DoAxisAlignedNDVolumesIntersect<DIM>(queryBoundaries, this->storedElements[i]->boundaries) == true)
				{
					elementCache.push_back(this->storedElements[i]);

					// Element cache heap structure is maintained.
					this->storedElements[i]->squaredDistance = CalculateSquaredEuclidianDistanceBetweenPoints(this->storedElements[i]->midpoint, queryCoordinates);
					std::push_heap(elementCache.begin(), elementCache.end(), [](QuadTreeElement<DIM, objT, coordT>* firstElement, QuadTreeElement<DIM, objT, coordT>* secondElement){return firstElement->squaredDistance > secondElement->squaredDistance;});
				}
			}
		}
	}
}

template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
void QuadTreeNode<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>::DistanceQuerySingleIteration(long long int paramDistanceQueryId, std::vector<QuadTreeElement<DIM, objT, coordT>*>& elementCache, const std::array<coordT, DIM>& queryCoordinates, const std::array<coordT, 2*DIM>& queryBoundaries)
{
	if (this->distanceQueryId != paramDistanceQueryId && DoAxisAlignedNDVolumesIntersect<DIM>(queryBoundaries, this->extendedBoundaries) == true)
	{
		// This node was not yet (fully) searched through (distance query id is not the same) AND
		// the node intersets the query window. This means that the search will be performed in this node.

		if (this->divided == true)
		{
			// Node is divided. Query is going to be delegated to its subnodes.

			// Assumme that all subnodes were already fully searched through.
			bool allSubnodesSearchedThrough = true;

			for (size_t i = 0; i < Pow<2, DIM>::value; ++i)
			{
				this->subnodes[i]->DistanceQuerySingleIteration(paramDistanceQueryId, elementCache, queryCoordinates, queryBoundaries);

				// If distance query id of a subnode equals the current value, then it was searched through (variable stays true).
				allSubnodesSearchedThrough &= this->subnodes[i]->distanceQueryId == paramDistanceQueryId;
			}

			if (allSubnodesSearchedThrough == true)
			{
				// All subnodes were already fully searched through, which means that this node can be marked
				// as fully searched through as well (its distance query id is set to the current value).

				this->distanceQueryId = paramDistanceQueryId;
			}
		}
		else
		{
			// Node is not divided. Query window will be checked against stored elements.

			// Total number of already processed elements in this node.
			size_t processedElementsCount = 0;

			for (size_t i = 0; i < this->storedElements.size(); ++i)
			{
				if (this->storedElements[i]->distanceQueryId != paramDistanceQueryId)
				{
					// This element was not yet processed (distance query id is not the same).
					// So it will be checked now to see if it intersects the current query window.

					// Even if element is a point, the check has the same complexity (two comparisons per dimension) as ND element.
					// And because point element's boundaries field is filled, we can use the same funtion as for ND element.
					if (DoAxisAlignedNDVolumesIntersect<DIM>(queryBoundaries, this->storedElements[i]->boundaries) == true)
					{
						// Element intersects the current query window, so it will be pushed into the result cache.
						// But the element cache contains heap data structure and it has to be maintained.

						elementCache.push_back(this->storedElements[i]);
			
						this->storedElements[i]->squaredDistance = CalculateSquaredEuclidianDistanceBetweenPoints(this->storedElements[i]->midpoint, queryCoordinates);
						std::push_heap(elementCache.begin(), elementCache.end(), [](QuadTreeElement<DIM, objT, coordT>* firstElement, QuadTreeElement<DIM, objT, coordT>* secondElement){return firstElement->squaredDistance > secondElement->squaredDistance;});

						// Element is now processed, so the distance query id is set and the counter is incremented.
						this->storedElements[i]->distanceQueryId = paramDistanceQueryId;
						++processedElementsCount;
					}
				}
				else
				{
					// This element was already processed (distance query id is the same),
					// but the counter is local so it has to be incremented even in this case.

					++processedElementsCount;
				}
			}
		
			// If every element from this node was processed, then this whole node is marked
			// as fully searched through (its distance query id is set to the current value).
			if (processedElementsCount == this->elementCount)
			{
				this->distanceQueryId = paramDistanceQueryId;
			}
		}
	}
}

template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
void QuadTreeNode<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>::RayQuery_2D(std::vector<QuadTreeElement<DIM, objT, coordT>*>& elementCache, coordT rayFirstPointX, coordT rayFirstPointY, coordT raySecondPointX, coordT raySecondPointY, coordT epsilon) const
{
	// This could be optimized if necessary.
	// First idea: We could first construct a bounding circle and test the node against it first, because it is simpler.
	// Squared diameter of that circle would be the squared length of rectangle's diagonal.
	// Second idea: This general condition is calculated from scratch for each node. But that is not efficient,
	// because from the view of the parent node, the subnodes share sides and that could be exploited.
	if (DoesLineIntersectRectangle_2D(rayFirstPointX, rayFirstPointY, raySecondPointX, raySecondPointY, this->extendedBoundaries[0], this->extendedBoundaries[1], this->extendedBoundaries[2], this->extendedBoundaries[3], epsilon) == true)
	{
		// Query ray collides with the extended boundaries of this node.

		if (this->divided == true)
		{
			// Node is divided. Query will be delegated to its subnodes.

			for (size_t i = 0; i < Pow<2, DIM>::value; ++i)
			{
				this->subnodes[i]->RayQuery_2D(elementCache, rayFirstPointX, rayFirstPointY, raySecondPointX, raySecondPointY, epsilon);
			}
		}
		else
		{
			// Node is not divided. Query ray will be checked against stored elements.

			for (size_t i = 0; i < this->storedElements.size(); ++i)
			{
				// Test is different for point and ND elements.

				if (this->storedElements[i]->isPoint == true)
				{
					if (DoesLineIntersectPoint_2D(rayFirstPointX, rayFirstPointY, raySecondPointX, raySecondPointY, this->storedElements[i]->midpoint[0], this->storedElements[i]->midpoint[1], epsilon) == true)
					{
						elementCache.push_back(this->storedElements[i]);
					}
				}
				else
				{
					if (DoesLineIntersectRectangle_2D(rayFirstPointX, rayFirstPointY, raySecondPointX, raySecondPointY, this->storedElements[i]->boundaries[0], this->storedElements[i]->boundaries[1], this->storedElements[i]->boundaries[2], this->storedElements[i]->boundaries[3], epsilon) == true)
					{
						elementCache.push_back(this->storedElements[i]);
					}
				}
			}
		}
	}
}

template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
void QuadTreeNode<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>::RayQuery_3D(std::vector<QuadTreeElement<DIM, objT, coordT>*>& elementCache, coordT rayFirstPointX, coordT rayFirstPointY, coordT rayFirstPointZ, coordT raySecondPointX, coordT raySecondPointY, coordT raySecondPointZ, coordT epsilon) const
{
	// This could be optimized if necessary.
	// First idea: We could first construct a bounding ball and test the node against it first, because it is simpler.
	// Squared diameter of that ball would be the squared length of rectangular cuboid's diagonal.
	// Second idea: This general condition is calculated from scratch for each node. But that is not efficient,
	// because from the view of the parent node, the subnodes share sides and that could be exploited.
	if (DoesLineIntersectRectangularCuboid_3D(rayFirstPointX, rayFirstPointY, rayFirstPointZ, raySecondPointX, raySecondPointY, raySecondPointZ, this->extendedBoundaries[0], this->extendedBoundaries[1], this->extendedBoundaries[2], this->extendedBoundaries[3], this->extendedBoundaries[4], this->extendedBoundaries[5], epsilon) == true)
	{
		// Query ray collides with the extended boundaries of this node.

		if (this->divided == true)
		{
			// Node is divided. Query will be delegated to its subnodes.

			for (size_t i = 0; i < Pow<2, DIM>::value; ++i)
			{
				this->subnodes[i]->RayQuery_3D(elementCache, rayFirstPointX, rayFirstPointY, rayFirstPointZ, raySecondPointX, raySecondPointY, raySecondPointZ, epsilon);
			}
		}
		else
		{
			// Node is not divided. Query ray will be checked against stored elements.

			for (size_t i = 0; i < this->storedElements.size(); ++i)
			{
				// Test is different for point and ND elements.

				if (this->storedElements[i]->isPoint == true)
				{
					if (DoesLineIntersectPoint_3D(rayFirstPointX, rayFirstPointY, rayFirstPointZ, raySecondPointX, raySecondPointY, raySecondPointZ, this->storedElements[i]->midpoint[0], this->storedElements[i]->midpoint[1], this->storedElements[i]->midpoint[2], epsilon) == true)
					{
						elementCache.push_back(this->storedElements[i]);
					}
				}
				else
				{
					if (DoesLineIntersectRectangularCuboid_3D(rayFirstPointX, rayFirstPointY, rayFirstPointZ, raySecondPointX, raySecondPointY, raySecondPointZ, this->storedElements[i]->boundaries[0], this->storedElements[i]->boundaries[1], this->storedElements[i]->boundaries[2], this->storedElements[i]->boundaries[3], this->storedElements[i]->boundaries[4], this->storedElements[i]->boundaries[5], epsilon) == true)
					{
						elementCache.push_back(this->storedElements[i]);
					}
				}
			}
		}
	}

	return;
}

template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
coordT QuadTreeNode<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>::GetBoundary(size_t dimension, bool isMin) const
{
	return this->boundaries[isMin == true ? 2*(dimension - 1) : 2*(dimension - 1) + 1];
}

template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
size_t QuadTreeNode<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>::GetElementCount() const
{
	return this->elementCount;
}

template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
size_t QuadTreeNode<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>::GetUniqueElementCount() const
{
	return this->uniqueElementCount;
}

template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
bool QuadTreeNode<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>::ValidateNode(std::array<bool, DIM> isOnMaxSide) const
{
	bool nodeValid = true;

	// General validation.

	// 1) Extendd boundaries are always bigger then base boundaries.
	for (size_t i = 0; i < DIM; ++i)
	{
		if (this->boundaries[2*i] < extendedBoundaries[2*i])
		{
			nodeValid &= false;
			std::cout << "Min boundary < min extended boundary." << "\n";
		}

		if (this->boundaries[2*i + 1] > extendedBoundaries[2*i + 1])
		{
			nodeValid &= false;
			std::cout << "Max boundary > max extended boundary." << "\n";
		}
	}

	// 2) Unique element count must be smaller than or equal to the standard element count.
	if (this->uniqueElementCount > this->elementCount)
	{
		nodeValid &= false;
		std::cout << "The number of unique elements is bigger then the number of elements." << "\n";
	}

	if (this->divided == true)
	{
		// Divided validation.
		
		// 3) Number of stored elements has to be zero.
		if (this->storedElements.size() != 0)
		{
			nodeValid &= false;
			std::cout << "Node is divided, but contains elements." << "\n";
		}

		// 4) Sum of the element counts inside subnodes must be equal to count in this one.
		size_t subnodesElementCount = 0;
		for (size_t i = 0; i < Pow<2, DIM>::value; ++i)
		{
			subnodesElementCount += this->subnodes[i]->elementCount;
		}

		if (subnodesElementCount != this->elementCount)
		{
			nodeValid &= false;
			std::cout << "Node is divided, but the number of elements in the subnodes does not match the current value." << "\n";
		}

		// 5) Sum of the unique element counts inside subnodes must be equal to count in this one.
		size_t subnodesUniqueElementCount = 0;
		for (size_t i = 0; i < Pow<2, DIM>::value; ++i)
		{
			subnodesUniqueElementCount += this->subnodes[i]->uniqueElementCount;
		}

		if (subnodesUniqueElementCount != this->uniqueElementCount)
		{
			nodeValid &= false;
			std::cout << "Node is divided, but the number of unique elements in the subnodes does not match the current value." << "\n";
		}

		// 6) All subnodes of divided node can't be nullptr.
		for (size_t i = 0; i < Pow<2, DIM>::value; ++i)
		{
			if (this->subnodes[i] == nullptr)
			{
				nodeValid &= false;
				std::cout << "Node is divided, but subnode [" << i << "] is nullptr." << "\n";
			}
		}

		// 7) AABB of extended boundaries of subnodes must be qual to the extended boundaries of this node.
		std::array<coordT, 2*DIM> recalculatedExtendedBoundaries = this->boundaries;
		for (size_t i = 0; i < Pow<2, DIM>::value; ++i)
		{
			for (size_t j = 0; j < DIM; ++j)
			{
				recalculatedExtendedBoundaries[2*j] = this->subnodes[i]->extendedBoundaries[2*j] < recalculatedExtendedBoundaries[2*j] ? this->subnodes[i]->extendedBoundaries[2*j] : recalculatedExtendedBoundaries[2*j];
				recalculatedExtendedBoundaries[2*j + 1] = this->subnodes[i]->extendedBoundaries[2*j + 1] > recalculatedExtendedBoundaries[2*j + 1] ? this->subnodes[i]->extendedBoundaries[2*j + 1] : recalculatedExtendedBoundaries[2*j + 1];
			}
		}

		for (size_t i = 0; i < DIM; ++i)
		{
			if (this->extendedBoundaries[2*i] != recalculatedExtendedBoundaries[2*i] || this->extendedBoundaries[2*i + 1] != recalculatedExtendedBoundaries[2*i + 1])
			{
				nodeValid &= false;
				std::cout << "Extended boundaries are wrong based on subnodes in dimension [" << i << "]." << "\n";
			}
		}
	}
	else
	{
		// Not divided validation.

		// 8) Element count must be the same as the size of the element vector.
		if (this->storedElements.size() != this->elementCount)
		{
			nodeValid &= false;
			std::cout << "Node is not divided, but the element count does not match the vector size." << "\n";
		}

		// 9) Unique element count must be the same as the number of elements inside the vector with unique coordinates.
		size_t recalculatedUniqueElementCount = 0;
		std::vector<QuadTreeElement<DIM, objT, coordT>*> storedElementsCopy = this->storedElements;

		while (storedElementsCopy.empty() == false)
		{
			QuadTreeElement<DIM, objT, coordT>* currentElement = storedElementsCopy.front();

			std::erase_if(storedElementsCopy, [&currentElement](QuadTreeElement<DIM, objT, coordT>* element){return currentElement->midpoint == element->midpoint;});

			++recalculatedUniqueElementCount;
		}

		if (recalculatedUniqueElementCount != this->uniqueElementCount)
		{
			nodeValid &= false;
			std::cout << "Node is not divided, but the unique element count does not match the number of unique elements in the vector." << "\n";
		}

		// 10) Not divided node must have all subnodes equal to nullptr.
		for (size_t i = 0; i < Pow<2, DIM>::value; ++i)
		{
			if (this->subnodes[i] != nullptr)
			{
				nodeValid &= false;
				std::cout << "Node is not divided, but subnode [" << i << "] is not nullptr." << "\n";
			}
		}

		// 11) Not divided node's extended boundaries have to be equal to the AABB of stored elements.
		std::array<coordT, 2*DIM> recalculatedExtendedBoundaries = this->boundaries;
		for (size_t i = 0; i < this->storedElements.size(); ++i)
		{
			for (size_t j = 0; j < DIM; ++j)
			{
				recalculatedExtendedBoundaries[2*j] = this->storedElements[i]->boundaries[2*j] < recalculatedExtendedBoundaries[2*j] ? this->storedElements[i]->boundaries[2*j] : recalculatedExtendedBoundaries[2*j];
				recalculatedExtendedBoundaries[2*j + 1] = this->storedElements[i]->boundaries[2*j + 1] > recalculatedExtendedBoundaries[2*j + 1] ? this->storedElements[i]->boundaries[2*j + 1] : recalculatedExtendedBoundaries[2*j + 1];
			}
		}

		for (size_t i = 0; i < DIM; ++i)
		{
			if (this->extendedBoundaries[2*i] != recalculatedExtendedBoundaries[2*i] || this->extendedBoundaries[2*i + 1] != recalculatedExtendedBoundaries[2*i + 1])
			{
				nodeValid &= false;
				std::cout << "Extended boundaries are wrong based on stored elements in dimension [" << i << "]." << "\n";
			}
		}

		// 12) All elements must be within the  bounds of the current node.
		for (size_t i = 0; i < this->storedElements.size(); ++i)
		{
			for (size_t j = 0; j < DIM; ++j)
			{
				if ((isOnMaxSide[j] == false && (this->storedElements[i]->midpoint[j] < this->boundaries[2*j] || this->storedElements[i]->midpoint[j] > this->boundaries[2*j + 1])) ||
					(isOnMaxSide[j] == true && (this->storedElements[i]->midpoint[j] <= this->boundaries[2*j] || this->storedElements[i]->midpoint[j] > this->boundaries[2*j + 1])))
				{
					nodeValid &= false;
					std::cout << "Element at index [" << i << "] does not belong to this tree in dimension [" << j << "]." << "\n";
				}
			}
		}
	}

	// Subnodes are validated if the current node is divided.
	if (this->divided == true)
	{
		// Correct side must be calculated for each dimension (taken from SplitNode() function).
		std::array<bool, DIM> subnodeIsOnMaxSide = {};
		subnodeIsOnMaxSide.fill(true);

		for (size_t i = 0; i < Pow<2, DIM>::value; ++i)
		{
			bool previousCarry = true;
			bool currentCarry = true;

			for (size_t j = 0; j < DIM; ++j)
			{
				currentCarry = subnodeIsOnMaxSide[j] && previousCarry;
				subnodeIsOnMaxSide[j] = (subnodeIsOnMaxSide[j] != previousCarry);
				previousCarry = currentCarry;
			}

			nodeValid &= this->subnodes[i]->ValidateNode(subnodeIsOnMaxSide);
		}
	}

	return nodeValid;
}

template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
void QuadTreeNode<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>::CalculateMidpoint()
{
	for (size_t i = 0; i < DIM; ++i)
	{
		this->midpoint[i] = (this->boundaries[2*i] + this->boundaries[2*i + 1])/static_cast<coordT>(2.0);
	}
}

template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
size_t QuadTreeNode<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>::FindSubnodeIndexForCoordinates(const std::array<coordT, DIM>& coordinates) const
{
	// Subnode array is ordered in a way similar to little endian binary number.
	// Each dimension is assigned a digit (x -> 2^0, y-> 2^1, z -> 2^2, ...).
	// Then, the specific digit contains either 0 or 1 based on if the subnode is on the min side or the max side.
	// In this way, we can read of the array index directly (range 0 - 2^DIM - 1).

	// We start with zero index (value of binary number is zero).
	size_t subnodeIndex = 0;

	// Least significant digit has multiplier equal to one.
	size_t currentDigitMultiplier = 1;

	// We are going to add up the digits (dimensions) from least significant to most significant.
	for (size_t i = 0; i < DIM; ++i)
	{
		// If coordinate is on the min side, the digit stays zero. If not, the weight of the current digit is added.
		// Border is included in the min side.
		subnodeIndex += coordinates[i] <= this->midpoint[i] ? 0 : currentDigitMultiplier;

		// Weight of the next digit is twice that of the current one.
		currentDigitMultiplier *= 2;
	}

	// Chosen subnode index is returned.
	return subnodeIndex;
}

template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
QuadTreeNode<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>* QuadTreeNode<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>::FindExistingLeafNodeForCoordinates(const std::array<coordT, DIM>& coordinates)
{
	if (this->divided == true)
	{
		// Node is divided. Find correct subnode for element and search there.

		return this->subnodes[this->FindSubnodeIndexForCoordinates(coordinates)]->FindExistingLeafNodeForCoordinates(coordinates);
	}
	else
	{
		// Node is not divided. Search is done.

		return this;
	}
}

template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
bool QuadTreeNode<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>::DoesElementAlreadyExistAtCoordinates(QuadTreeElement<DIM, objT, coordT>* element) const
{
	for (size_t i = 0; i < this->storedElements.size(); ++i)
	{
		if (element->midpoint == this->storedElements[i]->midpoint)
		{
			return true;
		}
	}

	return false;
}

template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
void QuadTreeNode<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>::SplitNode()
{
	// Two things have to be done during a node split.
	// Allocation of new subnodes and reinsertion of existing elements into them.

	// Because the dimension of the tree is variable, there has to be a special way to initialize each one of the 2^DIM subnodes generically.
	// We need to choose the correct boundaries for each one. This is going to be done using a isOnMaxSide array which tracks on which
	// side we currently are for each dimension.
	// Value 0 in the array means we are on the min side and value 1 means we are on the max side.
	// We are going to start from the state 0, 0, 0, ... (min, min, min, ...) and iterate through all the combinations
	// by simulating integer addition.
	// Example for 3D:
	// 0, 0, 0
	// 1, 0, 0
	// 0, 1, 0
	// 1, 1, 0
	// 0, 0, 1
	// 1, 0, 1
	// 0, 1, 1
	// 1, 1, 1

	// Array starts with all ones, so the first increment overflows the array to all zeroes.
	std::array<bool, DIM> isOnMaxSide = {};
	isOnMaxSide.fill(true);

	// Current boundaries.
	std::array<coordT, 2*DIM> localBoundaries = {};

	for (size_t i = 0; i < Pow<2, DIM>::value; ++i)
	{
		// Carry from previous iteration. Must be set to true, because we are incrementing the value inside the array by one.
		bool previousCarry = true;
		// New carry calculated in the current iteration. True is arbitrary.
		bool currentCarry = true;

		// Each dimension is going to be handled separately.
		for (size_t j = 0; j < DIM; ++j)
		{
			// Addition is performed on one (currently needed) digit in each iteration.
			currentCarry = isOnMaxSide[j] && previousCarry;
			isOnMaxSide[j] = (isOnMaxSide[j] != previousCarry);
			previousCarry = currentCarry;

			if (isOnMaxSide[j] == true)
			{
				// For max side, (midpoint, max) pair is chosen.
				localBoundaries[2*j] = this->midpoint[j];
				localBoundaries[2*j + 1] = this->boundaries[2*j + 1];
			}
			else
			{
				// For min side, (min, midpoint) pair is chosen.
				localBoundaries[2*j] = this->boundaries[2*j];
				localBoundaries[2*j + 1] = this->midpoint[j];
			}
		}

		// New subnode is allocated.
		this->subnodes[i] = this->nodePointerManager->Create(localBoundaries, this, this->nodePointerManager, this->elementPointerManager);
	}

	// Elements are redistributed into subnodes.
	for (size_t i = 0; i < this->storedElements.size(); ++i)
	{
		// Element counts do not change during a split.
		int elementCountChange = 0;
		int uniqueElementCountChange = 0;
		this->subnodes[this->FindSubnodeIndexForCoordinates(this->storedElements[i]->midpoint)]->InsertElementIntoThisLeafNode(this->storedElements[i], elementCountChange, uniqueElementCountChange);
	}

	// Lastly, the state of this node is set to divided and the vector containing stored elements is cleared.
	this->divided = true;
	this->storedElements.clear();
}

template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
void QuadTreeNode<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>::RecalculateExtendedBoundariesAfterElementInsertion(QuadTreeElement<DIM, objT, coordT>* element)
{
	// If inserted element is a point, then the extended boundaries won't change.
	// This is done to improve performance.
	if (element->isPoint == true)
	{
		return;
	}

	// The extended boundary of the leaf node is the AABB of all the inserted elements. So when the new element is inserted,
	// each side of the AABB must be tested separately to see if it should be moved or not.
	for (size_t i = 0; i < DIM; ++i)
	{
		// Min side of specific axis/dimension.
		this->extendedBoundaries[2*i] = element->boundaries[2*i] < this->extendedBoundaries[2*i] ? element->boundaries[2*i] : this->extendedBoundaries[2*i];
		// Max side of specific axis/dimension.
		this->extendedBoundaries[2*i + 1] = element->boundaries[2*i + 1] > this->extendedBoundaries[2*i + 1] ? element->boundaries[2*i + 1] : this->extendedBoundaries[2*i + 1];
	}
}

template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
void QuadTreeNode<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>::InsertElementIntoThisLeafNode(QuadTreeElement<DIM, objT, coordT>* element, int& elementCountChange, int& uniqueElementCountChange)
{
	// There are three possibilities when inserting an element into a leaf node.

	if (this->DoesElementAlreadyExistAtCoordinates(element) == true)
	{
		// Option one. In the leaf node there already is an element with the same coordinates.
		// In that case, the capacity is not taken into account and element is inserted.

		// If this is the first usage of storedElements vector, then MAX_CAP + 2(arbitrary overhang because of elements possibly occupying the same coordinates)
		// capacity is allocated at once. This is done to minimize reallocations. Testing showed that insertion performace improved by ~10% for 5-10 MAX_CAP.
		if (this->storedElements.capacity() == 0)
		{
			this->storedElements.reserve(MAX_CAP + 2);
		}

		this->storedElements.push_back(element);

		// Element with existing coordinates was added to this node. Count is incremented.
		elementCountChange = 1;
		uniqueElementCountChange = 0;
	}
	else
	{
		// this->depth cannot be less than 0, so the default case (MAX_DEPTH < 0 == no limit) always evaluates to false.
		// When MAX_DEPTH >= 0, then at max depth this condition will evaluate to true and override MAX_CAP.
		if (this->uniqueElementCount + 1 <= MAX_CAP || this->depth == MAX_DEPTH)
		{
			// Option two. No element with the same coordinates exists and node has free capacity. Element is inserted.

		    // If this is the first usage of storedElements vector, then MAX_CAP + 2(arbitrary overhang because of elements possibly occupying the same coordinates)
		    // capacity is allocated at once. This is done to minimize reallocations. Testing showed that insertion performace improved by ~10% for ~5-10 MAX_CAP.
			if (this->storedElements.capacity() == 0)
			{
				this->storedElements.reserve(MAX_CAP + 2);
			}

			this->storedElements.push_back(element);

			// Element with unique coordinates was added to this node. Both the count and unique count are incremented.
			elementCountChange = 1;
			uniqueElementCountChange = 1;
		}
		else
		{
			// Option three. No element with the same coordinates exists and node does not have free capacity.
			// This node will be split and the element will be inserted into one of the new subnodes.

			this->SplitNode();

			this->subnodes[this->FindSubnodeIndexForCoordinates(element->midpoint)]->InsertElementIntoThisLeafNode(element, elementCountChange, uniqueElementCountChange);

			// Counts are inherited from the subnode.
		}
	}

	// At this point, the new element is inserted into either the current node or one of the subnodes.

	// Extended boundaries of this node must be recalculated to take into the account the size of the new element.
	// It is not necessary however to take into account the AABBs of subnodes during recalculation in a case when the node is split.
	// Each node can be recalculated separately in relation to the new element and the whole extended boundary structure will still be valid.
	this->RecalculateExtendedBoundariesAfterElementInsertion(element);

	// Update class counts.
	this->elementCount = static_cast<size_t>(static_cast<int>(this->elementCount) + elementCountChange);
	this->uniqueElementCount = static_cast<size_t>(static_cast<int>(this->uniqueElementCount) + uniqueElementCountChange);
}

template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
void QuadTreeNode<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>::PropagateChangesUpAfterElementInsertion(QuadTreeNode<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>* stopNode, QuadTreeElement<DIM, objT, coordT>* element, const int& elementCountChange, const int& uniqueElementCountChange)
{
	// This node is going to update the one above it.
	if (this->parentNode != stopNode)
	{
		// Element was inserted, and thus the counts have to be incremented.
		this->parentNode->elementCount = static_cast<size_t>(static_cast<int>(this->parentNode->elementCount) + elementCountChange);
		this->parentNode->uniqueElementCount = static_cast<size_t>(static_cast<int>(this->parentNode->uniqueElementCount) + uniqueElementCountChange);

		// Extended boundaries have to be updated based on the boundaries of the inserted element.
		this->parentNode->RecalculateExtendedBoundariesAfterElementInsertion(element);

		// Propagate up by calling this function on the parent node.
		this->parentNode->PropagateChangesUpAfterElementInsertion(stopNode, element, elementCountChange, uniqueElementCountChange);
	}
}

template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
void QuadTreeNode<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>::RecalculateExtendedBoundariesAfterElementRemoval(bool isPoint)
{
	// If removed element is a point, then the extended boundaries won't change.
	// This is done to improve performance.
	if (isPoint == true)
	{
		return;
	}

	if (this->divided == true)
	{
		// The extended boundary of not a leaf node is the AABB of all the subnodes. When an element is removed,
		// every existing subnode has to be checked because we don't know which subnodes and elements are contributing
		// to the current values of extended boundaries. We iterate through each dimension and subnode.
		for (size_t i = 0; i < DIM; ++i)
		{
			// Values for specific dimension are reset.
			this->extendedBoundaries[2*i] = this->boundaries[2*i];
			this->extendedBoundaries[2*i + 1] = this->boundaries[2*i + 1];

			// Min and max values for extended boundaries are calculated based on subnodes' extended boundaries.
			for (size_t j = 0; j < Pow<2, DIM>::value; ++j)
			{
				// Min side of specific axis/dimension.
				this->extendedBoundaries[2*i] = this->subnodes[j]->extendedBoundaries[2*i] < this->extendedBoundaries[2*i] ? this->subnodes[j]->extendedBoundaries[2*i] : this->extendedBoundaries[2*i];
				// Max side of specific axis/dimension.
				this->extendedBoundaries[2*i + 1] = this->subnodes[j]->extendedBoundaries[2*i + 1] > this->extendedBoundaries[2*i + 1] ? this->subnodes[j]->extendedBoundaries[2*i + 1] : this->extendedBoundaries[2*i + 1];
			}
		}
	}
	else
	{
		// The extended boundary of a leaf node is the AABB of all the stored elements. When an element is removed,
		// every existing element has to be checked because we don't know which elements are contributing
		// to the current values of extended boundaries. We iterate through each dimension and element.
		for (size_t i = 0; i < DIM; ++i)
		{
			// Values for specific dimension are reset.
			this->extendedBoundaries[2*i] = this->boundaries[2*i];
			this->extendedBoundaries[2*i + 1] = this->boundaries[2*i + 1];

			// Min and max values for extended boundaries are calculated based on elements' boundaries.
			for (size_t j = 0; j < this->storedElements.size(); ++j)
			{
				// Min side of specific axis/dimension.
				this->extendedBoundaries[2*i] = this->storedElements[j]->boundaries[2*i] < this->extendedBoundaries[2*i] ? this->storedElements[j]->boundaries[2*i] : this->extendedBoundaries[2*i];
				// Max side of specific axis/dimension.
				this->extendedBoundaries[2*i + 1] = this->storedElements[j]->boundaries[2*i + 1] > this->extendedBoundaries[2*i + 1] ? this->storedElements[j]->boundaries[2*i + 1] : this->extendedBoundaries[2*i + 1];
			}
		}
	}
}

template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
bool QuadTreeNode<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>::RemoveElementFromThisLeafNode(std::vector<QuadTreeElement<DIM, objT, coordT>*>& elementCache, long long int id, const std::array<coordT, DIM>& paramMidpoint, const std::array<coordT, 2*DIM>& paramBoundaries, bool isPoint, int& elementCountChange, int& uniqueElementCountChange)
{
	// Result of the removal process.
	bool removalSuccess = false;

	// If element count is the same as unique element count, then there are no duplicates.
	// In that case, at most one element will be found.
	if (this->elementCount == this->uniqueElementCount)
	{
		// Iterator pointing to the found element.
		typename std::vector<QuadTreeElement<DIM, objT, coordT>*>::iterator elementIt = this->storedElements.begin();

		if (id == 0)
		{
			// Id is not given.

			if (isPoint == true)
			{
				// For point element, isPoint and midpoint have to match.
				elementIt = std::find_if(this->storedElements.begin(), this->storedElements.end(), [&paramMidpoint](QuadTreeElement<DIM, objT, coordT>* element){return element->isPoint == true && element->midpoint == paramMidpoint;});
			}
			else
			{
				// For ND element, isPoint, midpoint and boundaries have to match.
				// Midpoint equality is implied from boundaries, so it doesn't have to be tested.
				elementIt = std::find_if(this->storedElements.begin(), this->storedElements.end(), [&paramBoundaries](QuadTreeElement<DIM, objT, coordT>* element){return element->isPoint == false && element->boundaries == paramBoundaries;});
			}		
		}
		else
		{
			// Id is given.

			if (isPoint == true)
			{
				// For point element, id, isPoint and midpoint have to match.
				elementIt = std::find_if(this->storedElements.begin(), this->storedElements.end(), [&id, &paramMidpoint](QuadTreeElement<DIM, objT, coordT>* element){return element->id == id && element->isPoint == true && element->midpoint == paramMidpoint;});
			}
			else
			{
				// For ND element, id, isPoint, midpoint and boundaries have to match.
				// Midpoint equality is implied from boundaries, so it doesn't have to be tested.
				elementIt = std::find_if(this->storedElements.begin(), this->storedElements.end(), [&id, &paramBoundaries](QuadTreeElement<DIM, objT, coordT>* element){return element->id == id && element->isPoint == false && element->boundaries == paramBoundaries;});
			}
		}

		if (elementIt != this->storedElements.end())
		{
			// Element exists. It is removed from the array.

			std::iter_swap(elementIt, this->storedElements.end() - 1);
			elementCache.push_back(this->storedElements.back());
			this->storedElements.pop_back();

			elementCountChange = -1;
			uniqueElementCountChange = -1;

			removalSuccess = true;
		}
		else
		{
			// Element does not exist in this node. Failure is returned.

			elementCountChange = 0;
			uniqueElementCountChange = 0;

			removalSuccess = false;
		}
	}
	else
	{
		// Multiple matching elements could be at specified coordinates. All of them are going to be removed.

		// Number of matching elements at specified coordinates.
		size_t elementsOnCoordinatesCount = 0;
		// Iterator pointing to the beginning of the section with removed elements.
		typename std::vector<QuadTreeElement<DIM, objT, coordT>*>::iterator removeIt = this->storedElements.begin();

		if (id == 0)
		{
			// Id is not given.

			if (isPoint == true)
			{
				// Element is a point. All point elements with matching isPoint and MIDPOINT are removed.
				// Return values are negated because matching elements should be after the non matching ones.
				auto comparator = [&paramMidpoint, &elementsOnCoordinatesCount](QuadTreeElement<DIM, objT, coordT>* element)
				{
					if (element->midpoint == paramMidpoint)
					{
						// Number of elements at specified coordinates is counted.
						++elementsOnCoordinatesCount;

						if (element->isPoint == true)
						{
							return !true;
						}
					}

					return !false;
				};

				// Matching elements are shifted to the end of the vector.
				removeIt = std::partition(this->storedElements.begin(), this->storedElements.end(), comparator);
			}
			else
			{
				// Element is N-dimensional. All elements with matching isPoint and BOUNDARIES are removed.
				// Even though midpoint is implied from boundaries, it still has to be taken into account, because we need to count ALL elements at specified coordinates.
				// Return values are negated because matching elements should be after the non matching ones.
				auto comparator = [&paramMidpoint, &paramBoundaries, &elementsOnCoordinatesCount](QuadTreeElement<DIM, objT, coordT>* element)
				{
					if (element->midpoint == paramMidpoint)
					{
						// Number of elements at specified coordinates is counted.
						++elementsOnCoordinatesCount;

						if (element->isPoint == false && element->boundaries == paramBoundaries)
						{
							return !true;
						}
					}

					return !false;
				};

				// Matching elements are shifted to the end of the vector.
				removeIt = std::partition(this->storedElements.begin(), this->storedElements.end(), comparator);
			}
		}
		else
		{
			// Id is given.

			if (isPoint == true)
			{
				// Element is a point. All point elements with matching id, isPoint and MIDPOINT are removed.
				// Return values are negated because matching elements should be after the non matching ones.
				auto comparator = [&id, &paramMidpoint, &elementsOnCoordinatesCount](QuadTreeElement<DIM, objT, coordT>* element)
				{
					if (element->midpoint == paramMidpoint)
					{
						// Number of elements at specified coordinates is counted.
						++elementsOnCoordinatesCount;

						if (element->id == id && element->isPoint == true)
						{
							return !true;
						}
					}

					return !false;
				};

				// Matching elements are shifted to the end of the vector.
				removeIt = std::partition(this->storedElements.begin(), this->storedElements.end(), comparator);
			}
			else
			{
				// Element is N-dimensional. All elements with matching isPoint and BOUNDARIES are removed.
				// Even though midpoint is implied from boundaries, it still has to be taken into account, because we need to count ALL elements at specified coordinates.
				// Return values are negated because matching elements should be after the non matching ones.
				auto comparator = [&id, &paramMidpoint, &paramBoundaries, &elementsOnCoordinatesCount](QuadTreeElement<DIM, objT, coordT>* element)
				{
					if (element->midpoint == paramMidpoint)
					{
						// Number of elements at specified coordinates is counted.
						++elementsOnCoordinatesCount;

						if (element->id == id && element->isPoint == false && element->boundaries == paramBoundaries)
						{
							return !true;
						}
					}

					return !false;
				};

				// Matching elements are shifted to the end of the vector.
				removeIt = std::partition(this->storedElements.begin(), this->storedElements.end(), comparator);
			}
		}

		// Number of removed elements is calculated from the difference of iterators.
		elementCountChange = static_cast<int>(this->storedElements.end() - removeIt);

		if (elementCountChange > 0)
		{
			// At least one matching element was found.

			// Found elements are copied to the element cache, then removed from this node.
			std::copy(removeIt, this->storedElements.end(), std::back_inserter(elementCache));
			this->storedElements.erase(removeIt, this->storedElements.end());

			// If the number of found matching elements is the same as the number of all elements at specified coordinates,
			// then all of them were removed. That means that one unique element was removed. In the other case, some elements
			// still remain there, and thus the number of unique elements does not change.
			uniqueElementCountChange = static_cast<size_t>(elementCountChange) == elementsOnCoordinatesCount ? -1 : 0;
			// Elements were removed, so the change has to be negative.
			elementCountChange *= -1;

			removalSuccess = true;
		}
		else
		{
			// No matching element was found. elementCountChange is zero in this branch.

			uniqueElementCountChange = 0;

			removalSuccess = false;
		}
	}

	// Boundaries are recalculated only if removal was successful.
	if (removalSuccess == true)
	{
		this->RecalculateExtendedBoundariesAfterElementRemoval(isPoint);
	}

	// Class counts are updated.
	this->elementCount = static_cast<size_t>(static_cast<int>(this->elementCount) + elementCountChange);
	this->uniqueElementCount = static_cast<size_t>(static_cast<int>(this->uniqueElementCount) + uniqueElementCountChange);

	return removalSuccess;
}

template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
bool QuadTreeNode<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>::CanNodeBeMerged() const
{
	// Node can only be merged if child nodes are not divided and their total
	// unique element count is less then or equal to maximum capacity.

	size_t subnodesUniqueElementCount = 0;

	for (size_t i = 0; i < Pow<2, DIM>::value; ++i)
	{
		subnodesUniqueElementCount += this->subnodes[i]->uniqueElementCount;

		if (this->subnodes[i]->divided == true)
		{
			return false;
		}
	}

	if (subnodesUniqueElementCount > MAX_CAP)
	{
		return false;
	}

	return true;
}

template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
void QuadTreeNode<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>::RecalculateExtendedBoundariesAfterMerge()
{
	// There are two ways to get the new extended boundaries. Either by recalculating them by
	// using the elements or the subnodes. The test for a single element or a node is the same,
	// so we want to use the option with a lower number of operations.
	// Non unique element count is used here because we need to test every element because
	// we don't know which ones are on top of each other.
	if (this->elementCount < Pow<2, DIM>::value)
	{
		// IsPoint element property is not used here, because that would unnecessarily add if condition for every iteration.
		// We are not handling one changed element for which we know IsPoint, but all elements from the perspective of a node.

		for (size_t i = 0; i < DIM; ++i)
		{
			// Values for specific dimension are reset.
			this->extendedBoundaries[2*i] = this->boundaries[2*i];
			this->extendedBoundaries[2*i + 1] = this->boundaries[2*i + 1];

			// Min and max values for extended boundaries are calculated based on elements' boundaries.
			for (size_t j = 0; j < this->storedElements.size(); ++j)
			{
				// Min side of specific axis/dimension.
				this->extendedBoundaries[2*i] = this->storedElements[j]->boundaries[2*i] < this->extendedBoundaries[2*i] ? this->storedElements[j]->boundaries[2*i] : this->extendedBoundaries[2*i];
				// Max side of specific axis/dimension.
				this->extendedBoundaries[2*i + 1] = this->storedElements[j]->boundaries[2*i + 1] > this->extendedBoundaries[2*i + 1] ? this->storedElements[j]->boundaries[2*i + 1] : this->extendedBoundaries[2*i + 1];
			}
		}
	}
	else
	{
		for (size_t i = 0; i < DIM; ++i)
		{
			// Values for specific dimension are reset.
			this->extendedBoundaries[2*i] = this->boundaries[2*i];
			this->extendedBoundaries[2*i + 1] = this->boundaries[2*i + 1];

			// Min and max values for extended boundaries are calculated based on subnodes' extended boundaries.
			for (size_t j = 0; j < Pow<2, DIM>::value; ++j)
			{
				// Min side of specific axis/dimension.
				this->extendedBoundaries[2*i] = this->subnodes[j]->extendedBoundaries[2*i] < this->extendedBoundaries[2*i] ? this->subnodes[j]->extendedBoundaries[2*i] : this->extendedBoundaries[2*i];
				// Max side of specific axis/dimension.
				this->extendedBoundaries[2*i + 1] = this->subnodes[j]->extendedBoundaries[2*i + 1] > this->extendedBoundaries[2*i + 1] ? this->subnodes[j]->extendedBoundaries[2*i + 1] : this->extendedBoundaries[2*i + 1];
			}
		}
	}
}

template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
void QuadTreeNode<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>::MergeNode()
{
	// Current node is no longer a leaf node.
	this->divided = false;

	// Element counts are recalculated and elements from the leaf subnodes are copied into current node.
	this->elementCount = 0;
	this->uniqueElementCount = 0;

	for (size_t i = 0; i < Pow<2, DIM>::value; ++i)
	{
		this->elementCount += this->subnodes[i]->elementCount;
		this->uniqueElementCount += this->subnodes[i]->uniqueElementCount;

		// this->storedElements is already empty because this is not a leaf node yet.
		std::copy(this->subnodes[i]->storedElements.begin(), this->subnodes[i]->storedElements.end(), std::back_inserter(this->storedElements));
		// Subnode's vector has to be cleared, because otherwise the elements would be deallocated through a node's destructor.
		this->subnodes[i]->storedElements.clear();
	}

	// Extended boundaries are recalculated.
	this->RecalculateExtendedBoundariesAfterMerge();

	// Subnodes are deleted.
	for (size_t i = 0; i < Pow<2, DIM>::value; ++i)
	{
		this->nodePointerManager->Delete(this->subnodes[i]);
		this->subnodes[i] = nullptr;
	}
}

template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
void QuadTreeNode<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>::PropagateChangesUpAfterElementRemoval(QuadTreeNode<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>* stopNode, const int& elementCountChange, const int& uniqueElementCountChange, bool isPoint)
{
	// Element was removed, and thus the counts have to be adjusted.
	this->elementCount = static_cast<size_t>(static_cast<int>(this->elementCount) + elementCountChange);
	this->uniqueElementCount = static_cast<size_t>(static_cast<int>(this->uniqueElementCount) + uniqueElementCountChange);

	// Extended boundaries have to be updated based on the subnodes' extended boundaries.
	this->RecalculateExtendedBoundariesAfterElementRemoval(isPoint);

	// This node is going to update the one above it.
	if (this->parentNode != stopNode)
	{
		// Propagate up by calling this function on the parent node.
		this->parentNode->PropagateChangesUpAfterElementRemoval(stopNode, elementCountChange, uniqueElementCountChange, isPoint);
	}
}

template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
void QuadTreeNode<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>::FindExistingLeafNodeAndReinsertNodeForCoordinates(QuadTreeNode<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>*& existingLeafNode, QuadTreeNode<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>*& reinsertNode, const std::array<coordT, DIM>& currentCoordinates, const std::array<coordT, DIM>& newCoordinates)
{
	if (this->divided == true)
	{
		// Node is divided. Search continues in subnodes.

		// Correct subnode for both the current and new coordinates is found.
		QuadTreeNode<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>* existingLeafSubnode = this->subnodes[this->FindSubnodeIndexForCoordinates(currentCoordinates)];
		QuadTreeNode<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>* reinsertSubnode = this->subnodes[this->FindSubnodeIndexForCoordinates(newCoordinates)];
	
		if (existingLeafSubnode != reinsertSubnode)
		{
			// Subnodes don't match. That means that we are currently in the lowest node shared by the
			// current and new coordinates. Thus this node is what we are looking for as a reinsertNode.
			// For current existing coordinates we want to find their leaf node. The search continues
			// with the basic FindExistingLeafNodeForCoordinates function.

			reinsertNode = this;
			existingLeafNode = existingLeafSubnode->FindExistingLeafNodeForCoordinates(currentCoordinates);
		}
		else
		{
			// Subnodes match. That means that the lowest shared node is below this one.

			existingLeafSubnode->FindExistingLeafNodeAndReinsertNodeForCoordinates(existingLeafNode, reinsertNode, currentCoordinates, newCoordinates);
		}	
	}
	else
	{
		// Node is not divided. Both current and new coordinates belong to this node. Search is done.

		existingLeafNode = this;
		reinsertNode = this;
	}
}
