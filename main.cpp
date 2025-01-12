
#include <iostream>

#include "QuadTree.h"
#include "QuadTreeTester.h"


static void PrintStringVector(std::vector<std::string>& vector)
{
	std::cout << "Contained elements: " << "\n";

	for (std::string& element : vector)
	{
		std::cout << element << "\n";
	}

	std::cout << "\n";
}

template<typename coordType>
static void PrintPair(std::optional<std::pair<coordType, std::string>>& pair)
{
	std::cout << "Contained element: " << "\n";
	if (pair.has_value() == true)
	{
		std::cout << "Squared distance: " << pair.value().first << ", Value: " << pair.value().second << "\n";
	}
	std::cout << "\n";
}

template<typename coordType>
static void PrintPairVector(std::vector<std::pair<coordType, std::string>>& pairVector)
{
	std::cout << "Contained elements: " << "\n";
	for (std::pair<coordType, std::string>& element : pairVector)
	{
		std::cout << "Squared distance: " << element.first << ", Value: " << element.second << "\n";
	}
	std::cout << "\n";
}

int main()
{
	// Example usage of the QuadTree class in 2D. 


	// Tree construction.

	// Arguments for class template.
	using objectType = std::string;
	using coordType = double;

	constexpr size_t dimension = 2;
	constexpr size_t leafNodeCapacity = 5;
	constexpr size_t maxDepth = 5;

	// Tree boundaries.
	const coordType quadTreeMinX = -500.0;
	const coordType quadTreeMaxX = 500.0;
	const coordType quadTreeMinY = -500.0;
	const coordType quadTreeMaxY = 500.0;

	QuadTree<dimension, leafNodeCapacity, objectType, maxDepth, coordType> quadTree2D(quadTreeMinX, quadTreeMaxX, quadTreeMinY, quadTreeMaxY);


	// Tree operations.


	// Insertion.
	[[maybe_unused]] bool returnBool = true;
	coordType elementX = coordType(0.0);
	coordType elementY = coordType(0.0);
	coordType elementBoundaryMinX = coordType(0.0);
	coordType elementBoundaryMaxX = coordType(0.0);
	coordType elementBoundaryMinY = coordType(0.0);
	coordType elementBoundaryMaxY = coordType(0.0);

	// Insertion of point elements without id.
	elementX = coordType(0.0);
	elementY = coordType(0.0);
	returnBool = quadTree2D.InsertPointElement(0, std::string("[point element 1, no id]"), elementX, elementY);

	elementX = coordType(-100.0);
	elementY = coordType(0.0);
	returnBool = quadTree2D.InsertPointElement(0, std::string("[point element 2, no id]"), elementX, elementY);

	elementX = coordType(100.0);
	elementY = coordType(0.0);
	returnBool = quadTree2D.InsertPointElement(0, std::string("[point element 3, no id]"), elementX, elementY);

	elementX = coordType(0.0);
	elementY = coordType(-500.0);
	returnBool = quadTree2D.InsertPointElement(0, std::string("[point element 4, no id]"), elementX, elementY);

	// Insertion of point elements with id.
	elementX = coordType(0.0);
	elementY = coordType(100.0);
	returnBool = quadTree2D.InsertPointElement(5, std::string("[point element 5, id]"), elementX, elementY);

	elementX = coordType(0.0);
	elementY = coordType(-100.0);
	returnBool = quadTree2D.InsertPointElement(6, std::string("[point element 6, id]"), elementX, elementY);

	elementX = coordType(0.0);
	elementY = coordType(500.0);
	returnBool = quadTree2D.InsertPointElement(7, std::string("[point element 7, id]"), elementX, elementY);

	// Insertion of n-dimensional elements without id.
	elementBoundaryMinX = coordType(-125.0);
	elementBoundaryMaxX = coordType(-75.0);
	elementBoundaryMinY = coordType(75.0);
	elementBoundaryMaxY = coordType(125.0);
	returnBool = quadTree2D.InsertNDElement(0, std::string("[ND element 1, no id]"), elementBoundaryMinX, elementBoundaryMaxX, elementBoundaryMinY, elementBoundaryMaxY);

	elementBoundaryMinX = coordType(75.0);
	elementBoundaryMaxX = coordType(125.0);
	elementBoundaryMinY = coordType(75.0);
	elementBoundaryMaxY = coordType(125.0);
	returnBool = quadTree2D.InsertNDElement(0, std::string("[ND element 2, no id]"), elementBoundaryMinX, elementBoundaryMaxX, elementBoundaryMinY, elementBoundaryMaxY);

	elementBoundaryMinX = coordType(-525.0);
	elementBoundaryMaxX = coordType(-475.0);
	elementBoundaryMinY = coordType(-25.0);
	elementBoundaryMaxY = coordType(25.0);
	returnBool = quadTree2D.InsertNDElement(0, std::string("[ND element 3, no id]"), elementBoundaryMinX, elementBoundaryMaxX, elementBoundaryMinY, elementBoundaryMaxY);

	// Insertion of n-dimensional elements with id.
	elementBoundaryMinX = coordType(-125.0);
	elementBoundaryMaxX = coordType(-75.0);
	elementBoundaryMinY = coordType(-125.0);
	elementBoundaryMaxY = coordType(-75.0);
	returnBool = quadTree2D.InsertNDElement(4, std::string("[ND element 4, id]"), elementBoundaryMinX, elementBoundaryMaxX, elementBoundaryMinY, elementBoundaryMaxY);

	elementBoundaryMinX = coordType(75.0);
	elementBoundaryMaxX = coordType(125.0);
	elementBoundaryMinY = coordType(-125.0);
	elementBoundaryMaxY = coordType(-75.0);
	returnBool = quadTree2D.InsertNDElement(5, std::string("[ND element 5, id]"), elementBoundaryMinX, elementBoundaryMaxX, elementBoundaryMinY, elementBoundaryMaxY);

	elementBoundaryMinX = coordType(475.0);
	elementBoundaryMaxX = coordType(525.0);
	elementBoundaryMinY = coordType(-25.0);
	elementBoundaryMaxY = coordType(25.0);
	returnBool = quadTree2D.InsertNDElement(6, std::string("[ND element 6, id]"), elementBoundaryMinX, elementBoundaryMaxX, elementBoundaryMinY, elementBoundaryMaxY);


	// Relocation.
	coordType newElementX = coordType(0.0);
	coordType newElementY = coordType(0.0);
	coordType newElementBoundaryMinX = coordType(0.0);
	coordType newElementBoundaryMaxX = coordType(0.0);
	coordType newElementBoundaryMinY = coordType(0.0);
	coordType newElementBoundaryMaxY = coordType(0.0);

	// Relocation of point element without id.
	elementX = coordType(0.0);
	elementY = coordType(-500.0);
	newElementX = coordType(-500.0);
	newElementY = coordType(-500.0);
	returnBool = quadTree2D.RelocatePointElement(0, elementX, elementY, newElementX, newElementY);

	// Relocation of point element with id.
	elementX = coordType(0.0);
	elementY = coordType(500.0);
	newElementX = coordType(500.0);
	newElementY = coordType(500.0);
	returnBool = quadTree2D.RelocatePointElement(7, elementX, elementY, newElementX, newElementY);

	// Relocation of n-dimensional element without id.
	elementBoundaryMinX = coordType(-525.0);
	elementBoundaryMaxX = coordType(-475.0);
	elementBoundaryMinY = coordType(-25.0);
	elementBoundaryMaxY = coordType(25.0);
	newElementBoundaryMinX = coordType(-525.0);
	newElementBoundaryMaxX = coordType(-475.0);
	newElementBoundaryMinY = coordType(475.0);
	newElementBoundaryMaxY = coordType(525.0);
	returnBool = quadTree2D.RelocateNDElement(0, elementBoundaryMinX, elementBoundaryMaxX, elementBoundaryMinY, elementBoundaryMaxY, newElementBoundaryMinX, newElementBoundaryMaxX, newElementBoundaryMinY, newElementBoundaryMaxY);

	// Relocation of n-dimensional element with id.
	elementBoundaryMinX = coordType(475.0);
	elementBoundaryMaxX = coordType(525.0);
	elementBoundaryMinY = coordType(-25.0);
	elementBoundaryMaxY = coordType(25.0);
	newElementBoundaryMinX = coordType(475.0);
	newElementBoundaryMaxX = coordType(525.0);
	newElementBoundaryMinY = coordType(-525.0);
	newElementBoundaryMaxY = coordType(-475.0);
	returnBool = quadTree2D.RelocateNDElement(6, elementBoundaryMinX, elementBoundaryMaxX, elementBoundaryMinY, elementBoundaryMaxY, newElementBoundaryMinX, newElementBoundaryMaxX, newElementBoundaryMinY, newElementBoundaryMaxY);


	// Deletion.
	std::vector<std::string> returnStrings;

	// Deletion of point element without id.
	elementX = coordType(-500.0);
	elementY = coordType(-500.0);
	returnBool = quadTree2D.DeletePointElement(0, elementX, elementY);

	// Deletion (and return) of point element with id.
	elementX = coordType(500.0);
	elementY = coordType(500.0);
	returnStrings = quadTree2D.DeleteAndReturnPointElement(7, elementX, elementY);

	// Deletion (and return) of n-dimensional element without id.
	elementBoundaryMinX = coordType(-525.0);
	elementBoundaryMaxX = coordType(-475.0);
	elementBoundaryMinY = coordType(475.0);
	elementBoundaryMaxY = coordType(525.0);
	returnStrings = quadTree2D.DeleteAndReturnNDElement(0, elementBoundaryMinX, elementBoundaryMaxX, elementBoundaryMinY, elementBoundaryMaxY);

	// Deletion of n-dimensional element with id.
	elementBoundaryMinX = coordType(475.0);
	elementBoundaryMaxX = coordType(525.0);
	elementBoundaryMinY = coordType(-525.0);
	elementBoundaryMaxY = coordType(-475.0);
	returnBool = quadTree2D.DeleteNDElement(6, elementBoundaryMinX, elementBoundaryMaxX, elementBoundaryMinY, elementBoundaryMaxY);


	// Point query.
	coordType queryPointX = coordType(0.0);
	coordType queryPointY = coordType(0.0);
	returnStrings = quadTree2D.PointQuery(queryPointX, queryPointY);
	PrintStringVector(returnStrings);


	// N-dimensional query.
	coordType queryBoundaryMinX = coordType(-80.0);
	coordType queryBoundaryMaxX = coordType(80.0);
	coordType queryBoundaryMinY = coordType(-80.0);
	coordType queryBoundaryMaxY = coordType(80.0);
	returnStrings = quadTree2D.NDQuery(queryBoundaryMinX, queryBoundaryMaxX, queryBoundaryMinY, queryBoundaryMaxY);
	PrintStringVector(returnStrings);


	// Closest element query.
	coordType singleIterationStep = coordType(100.0);
	std::optional<std::pair<coordType, std::string>> returnPair;
	queryPointX = coordType(1000.0);
	queryPointY = coordType(1000.0);
	returnPair = quadTree2D.QueryClosestElement(singleIterationStep, queryPointX, queryPointY);
	PrintPair(returnPair);

	// N closest elements query.
	size_t numberOfClosestElements = 4;
	singleIterationStep = coordType(100.0);
	std::vector<std::pair<coordType, std::string>> returnPairVector;
	queryPointX = coordType(1000.0);
	queryPointY = coordType(1000.0);
	returnPairVector = quadTree2D.QueryNClosestElements(numberOfClosestElements, singleIterationStep, queryPointX, queryPointY);
	PrintPairVector(returnPairVector);


	// Radius query.
	coordType queryRadius = coordType(110.0);
	queryPointX = coordType(0.0);
	queryPointY = coordType(0.0);
	returnPairVector = quadTree2D.QueryElementsWithinRadius(queryRadius, queryPointX, queryPointY);
	PrintPairVector(returnPairVector);

	// Ray query.
	coordType rayFirstPointX = coordType(-1000.0);
	coordType rayFirstPointY = coordType(-1000.0);
	coordType raySecondPointX = coordType(1000.0);
	coordType raySecondPointY = coordType(1000.0);
	returnStrings = quadTree2D.RayQuery(rayFirstPointX, rayFirstPointY, raySecondPointX, raySecondPointY);
	PrintStringVector(returnStrings);

	QuadTreeTester().TestTree();








	// Point query.
	queryPointX = coordType(0.0);
	queryPointY = coordType(0.0);
	returnStrings = quadTree2D.PointQueryAndDelete(queryPointX, queryPointY);
	PrintStringVector(returnStrings);


	// N-dimensional query.
	queryBoundaryMinX = coordType(-80.0);
	queryBoundaryMaxX = coordType(80.0);
	queryBoundaryMinY = coordType(-80.0);
	queryBoundaryMaxY = coordType(80.0);
	returnStrings = quadTree2D.NDQueryAndDelete(queryBoundaryMinX, queryBoundaryMaxX, queryBoundaryMinY, queryBoundaryMaxY);
	PrintStringVector(returnStrings);


	// Closest element query.
	singleIterationStep = coordType(100.0);
	returnPair;
	queryPointX = coordType(1000.0);
	queryPointY = coordType(1000.0);
	returnPair = quadTree2D.QueryClosestElementAndDelete(singleIterationStep, queryPointX, queryPointY);
	PrintPair(returnPair);

	// N closest elements query.
	numberOfClosestElements = 4;
	singleIterationStep = coordType(100.0);
	returnPairVector;
	queryPointX = coordType(1000.0);
	queryPointY = coordType(1000.0);
	returnPairVector = quadTree2D.QueryNClosestElementsAndDelete(numberOfClosestElements, singleIterationStep, queryPointX, queryPointY);
	PrintPairVector(returnPairVector);


	// Radius query.
	queryRadius = coordType(110.0);
	queryPointX = coordType(0.0);
	queryPointY = coordType(0.0);
	returnPairVector = quadTree2D.QueryElementsWithinRadiusAndDelete(queryRadius, queryPointX, queryPointY);
	PrintPairVector(returnPairVector);

	// Ray query.
	rayFirstPointX = coordType(-1000.0);
	rayFirstPointY = coordType(-1000.0);
	raySecondPointX = coordType(1000.0);
	raySecondPointY = coordType(1000.0);
	returnStrings = quadTree2D.RayQueryAndDelete(rayFirstPointX, rayFirstPointY, raySecondPointX, raySecondPointY);
	PrintStringVector(returnStrings);

	size_t a = quadTree2D.Size();
	size_t b = quadTree2D.UniqueSize();
	quadTree2D.Clear();

	std::cout << a + b << std::endl;

	return 0;












	return 0;
}
