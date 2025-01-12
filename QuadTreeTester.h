
#pragma once

#include <cmath>
#include <random>

#include "QuadTree.h"


// Contains automatic tests for the quad tree class.
class QuadTreeTester
{
public:

	// Test element structure that is used to keep track of stored elements that are inside a quad tree externaly.
	template<size_t DIM, typename objT, typename coordT = double>
	struct TestElement
	{
		bool isPoint;
		bool hasId;
		long long int id;
		std::array<coordT, DIM> coordinates;
		std::array<coordT, 2*DIM> boundaries;
		objT object;
	};

	// Default constructor.
	QuadTreeTester();
	// Destructor.
	~QuadTreeTester();

	// Copy/move constructor/assignment.
	QuadTreeTester(const QuadTreeTester& edge) = delete;
	QuadTreeTester(QuadTreeTester&& edge) noexcept = delete;
	QuadTreeTester& operator=(const QuadTreeTester& edge) = delete;
	QuadTreeTester& operator=(QuadTreeTester&& edge) noexcept = delete;


	// Automatic insertion test.

	// Helper function that transforms array into a parameter pack for insertion test (caller part).
	template<size_t... indexes, size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT, size_t N>
	void InsertionWrapper_Caller([[maybe_unused]] std::index_sequence<indexes...> indexSequence, QuadTree<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>& quadTree, long long int& id, objT& object, std::array<coordT, N>& parameterArray) const;

	// Helper function that transforms array into a parameter pack for insertion test (sequence generator part).
	template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT, size_t N>
	void InsertionWrapper_SequenceGenerator(QuadTree<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>& quadTree, long long int& id, objT& object, std::array<coordT, N>& parameterArray) const;

	// Function expects empty SQUARE quad tree and empty vector of batches of test elements.
	// Elements are going to be inserted using several strategies.
	// 1) Insertion of floor(elementCount^(1/DIM))^DIM elements into an ordered axis aligned grid (from min to max boundary value).
	//    - 1 point element with id, 1 ND element with id, 2 point elements without id, 2 ND elements without id at each coordinates (stored as one batch in vector)
	// 2) Insertion of elementCount elements to random valid and invalid coordinates.
	//    - 1 point element with id, 1 ND element with id, 2 point elements without id, 2 ND elements without id at each coordinates (stored as one batch in vector)
	// 3) Insertion of elementCount/5 elements to random valid and invalid coordinates.
	//    - Always one element per coordinates (point/ND, id/no id) (one element per batch in vector).
	template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
	bool TestInsertion(QuadTree<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>& quadTree, std::vector<std::vector<TestElement<DIM, objT, coordT>>>& testElements, size_t elementCount) const;


	// Automatic deletion test.

	// Helper function that transforms array into a parameter pack for deletion test (caller part).
	template<size_t... indexes, size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT, size_t N>
	std::vector<objT> DeletionWrapper_Caller([[maybe_unused]] std::index_sequence<indexes...> indexSequence, QuadTree<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>& quadTree, long long int& id, std::array<coordT, N>& parameterArray) const;

	// Helper function that transforms array into a parameter pack for deletion test (sequence generator part).
	template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT, size_t N>
	std::vector<objT> DeletionWrapper_SequenceGenerator(QuadTree<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>& quadTree, long long int& id, std::array<coordT, N>& parameterArray) const;

	// Function expects SQUARE quad tree filled with elements and filled vector representing the same state.
	// Random element from random batch is chosen and deleted (according to its type) until no element remains.
	template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
	bool TestDeletion(QuadTree<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>& quadTree, std::vector<std::vector<TestElement<DIM, objT, coordT>>>& testElements) const;


	// Automatic relocation test.

	// Helper function that transforms arrays into a parameter pack for relocation test (caller part).
	template<size_t... indexes, size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT, size_t N>
	bool RelocationWrapper_Caller([[maybe_unused]] std::index_sequence<indexes...> indexSequence, QuadTree<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>& quadTree, long long int& id, std::array<coordT, N>& firstParameterArray, std::array<coordT, N>& secondParameterArray) const;

	// Helper function that transforms arrays into a parameter pack for relocation test (caller part).
	template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT, size_t N>
	bool RelocationWrapper_SequenceGenerator(QuadTree<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>& quadTree, long long int& id, std::array<coordT, N>& firstParameterArray, std::array<coordT, N>& secondParameterArray) const;

	// Function expects SQUARE quad tree filled with elements and filled vector representing the same state.
	// Elements are going to be relocated using several strategies.
	// 1) Relocation of fixed number of elements to invalid target coordinates. (Invalid source coordinates are present by default from the insertion test).
	// 2) Relocation of elementCount/4 elements into new coordinates.
	//    - Random element from random batch is chosen and relocated (according to its type).
	// 3) Relocation of elementCount/4 elements into existing coordinates.
	//    - Random element from random batch is chosen and relocated (according to its type).
	// It is possible for this function to return succes even though there might be errors because
	// the quad tree relocation function does not return sufficient info. So this call should always be
	// followed by the deletion test.
	template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
	bool TestRelocation(QuadTree<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>& quadTree, std::vector<std::vector<TestElement<DIM, objT, coordT>>>& testElements) const;


	// Automatic location query tests.

	// Helper function that transforms array into a parameter pack for location query test (caller part).
	template<size_t... indexes, size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT, size_t N>
	std::vector<objT> LocationQueryWrapper_Caller([[maybe_unused]] std::index_sequence<indexes...> indexSequence, QuadTree<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>& quadTree, std::array<coordT, N>& parameterArray) const;

	// Helper function that transforms array into a parameter pack for location query test (sequence generator part).
	template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT, size_t N>
	std::vector<objT> LocationQueryWrapper_SequenceGenerator(QuadTree<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>& quadTree, std::array<coordT, N>& parameterArray) const;

	// Function expects SQUARE quad tree filled with elements and filled vector representing the same state.
	// Elements are going to be queried using several strategies.
	// 1) Random point and ND queries.
	// 2) Random point and ND queries where coordinates are rounded to the nearest integer.
	template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
	bool TestLocationQueries(QuadTree<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>& quadTree, std::vector<std::vector<TestElement<DIM, objT, coordT>>>& testElements) const;

	// Perfoms tests of location queries' edge cases (vertex touch, edge touch, etc.).
	bool TestLocationQueriesEdgeCases() const;


	// Automatic distance query tests.

	// Helper function that transforms array into a parameter pack for closest element query test (caller part).
	template<size_t... indexes, size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT, size_t N>
	std::optional<std::pair<coordT, objT>> DistanceQueryClosestWrapper_Caller([[maybe_unused]] std::index_sequence<indexes...> indexSequence, QuadTree<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>& quadTree, coordT step, std::array<coordT, N>& parameterArray) const;

	// Helper function that transforms array into a parameter pack for closest element query test (sequence generator part).
	template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT, size_t N>
	std::optional<std::pair<coordT, objT>> DistanceQueryClosestWrapper_SequenceGenerator(QuadTree<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>& quadTree, coordT step, std::array<coordT, N>& parameterArray) const;

	// Helper function that transforms array into a parameter pack for n closest elements query test (caller part).
	template<size_t... indexes, size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT, size_t N>
	std::vector<std::pair<coordT, objT>> DistanceQueryNClosestWrapper_Caller([[maybe_unused]] std::index_sequence<indexes...> indexSequence, QuadTree<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>& quadTree, size_t count, coordT step, std::array<coordT, N>& parameterArray) const;

	// Helper function that transforms array into a parameter pack for n closest elements test (sequence generator part).
	template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT, size_t N>
	std::vector<std::pair<coordT, objT>> DistanceQueryNClosestWrapper_SequenceGenerator(QuadTree<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>& quadTree, size_t count, coordT step, std::array<coordT, N>& parameterArray) const;

	// Helper function that transforms array into a parameter pack for radius query test (caller part).
	template<size_t... indexes, size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT, size_t N>
	std::vector<std::pair<coordT, objT>> DistanceQueryRadiusWrapper_Caller([[maybe_unused]] std::index_sequence<indexes...> indexSequence, QuadTree<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>& quadTree, coordT radius, std::array<coordT, N>& parameterArray) const;

	// Helper function that transforms array into a parameter pack for radius query test (sequence generator part).
	template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT, size_t N>
	std::vector<std::pair<coordT, objT>> DistanceQueryRadiusWrapper_SequenceGenerator(QuadTree<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>& quadTree, coordT radius, std::array<coordT, N>& parameterArray) const;

	// Function expects SQUARE quad tree filled with elements and filled vector representing the same state.
	// Test chooses a random point and tests all distance queries with it as a starting point.
	template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
	bool TestDistanceQueries(QuadTree<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>& quadTree, std::vector<std::vector<TestElement<DIM, objT, coordT>>>& testElements) const;

	// Performs tests of radius query's edge cases (vertex touch, edge touch).
	bool TestRadiusQueryEdgeCases() const;


	// Automatic ray query tests.

	// Function expects SQUARE quad tree filled with elements and filled vector representing the same state.
	// Test generates random 2D rays and compares the quad tree result with brute force approach.
	template<size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
	bool TestRayQuery_2D(QuadTree<2, MAX_CAP, objT, MAX_DEPTH, coordT>& quadTree, std::vector<std::vector<TestElement<2, objT, coordT>>>& testElements) const;

	// Performs tests of 2D ray query's edge cases (vertex touch, edge touch).
	bool TestRayQueryEdgeCases_2D() const;

	// Function expects SQUARE quad tree filled with elements and filled vector representing the same state.
	// Test generates random 3D rays and compares the quad tree result with brute force approach.
	template<size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
	bool TestRayQuery_3D(QuadTree<3, MAX_CAP, objT, MAX_DEPTH, coordT>& quadTree, std::vector<std::vector<TestElement<3, objT, coordT>>>& testElements) const;

	// Performs tests of 3D ray query's edge cases (vertex touch, edge touch).
	bool TestRayQueryEdgeCases_3D() const;


	// General test functions.

	// General function performing all the quad tree operation tests.
	bool TestTreeOperations() const;

	// Performs tests of quad tree's copy/move constructors/assignments.
	bool TestTreeConstruction() const;

	// Fuction that runs all available tests.
	bool TestTree() const;
};

QuadTreeTester::QuadTreeTester()
{
}

QuadTreeTester::~QuadTreeTester()
{
}

template<size_t... indexes, size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT, size_t N>
void QuadTreeTester::InsertionWrapper_Caller([[maybe_unused]] std::index_sequence<indexes...> indexSequence, QuadTree<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>& quadTree, long long int& id, objT& object, std::array<coordT, N>& parameterArray) const
{
	if constexpr (N == DIM)
	{
		quadTree.InsertPointElement(id, object, parameterArray[indexes]...);
	}
	else
	{
		quadTree.InsertNDElement(id, object, parameterArray[indexes]...);
	}
}

template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT, size_t N>
void QuadTreeTester::InsertionWrapper_SequenceGenerator(QuadTree<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>& quadTree, long long int& id, objT& object, std::array<coordT, N>& parameterArray) const
{
	this->InsertionWrapper_Caller(std::make_index_sequence<N>(), quadTree, id, object, parameterArray);
}

template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
bool QuadTreeTester::TestInsertion(QuadTree<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>& quadTree, std::vector<std::vector<TestElement<DIM, objT, coordT>>>& testElements, size_t elementCount) const
{
	bool insertionSuccessful = true;

	// Element properties.
	bool currentIsPoint = false;
	bool currentHasId = false;
	long long int currentId = 0;
	std::array<coordT, DIM> currentCoordinates = {};
	std::array<coordT, 2*DIM> currentBoundaries = {};
	objT currentObject = 0;

	// Test properties.
	// Test assumes that quadTree is a square. Max of first dimension - min of first dimension.
	double quadTreeWidth = quadTree.GetBoundary(1, false) - quadTree.GetBoundary(1, true);
	// Calculated based on nth root of element count rounded down. Minimum number of elements per dimension is two.
	size_t elementsPerDimensionCount = static_cast<size_t>(std::floor(std::pow(elementCount, 1.0/DIM)) < 2.0 ? 2.0 : std::floor(std::pow(elementCount, 1.0/DIM)));
	double step = quadTreeWidth/(static_cast<double>(elementsPerDimensionCount) - 1.0);
	// Helper counter. (2*DIM so there is no overflow.)
	std::array<size_t, 2*DIM> counter = {};

	// Random generators.
	unsigned int seed = 1;
	std::mt19937_64 randomNumberGenerator(seed);
	// Boundary generator for range elements. If this value is too big, then the likelihood of numerical errors increases.
	// ND element doesn't coincide with point element in that case. (currentCoordinates[x] != (currentBoundaries[2*x] + currentBoundaries[2*x + 1])/2.0)
	// Even those cases should be valid and stable however.
	double boundaryMaxWidth = 100.0;
	std::uniform_real_distribution<double> uniformRealDistributionBoundary(0, std::nextafter(boundaryMaxWidth, std::numeric_limits<double>::max()));
	// Coordinate generator for random positions. Assumes that quad tree is a square. Overhang is arbitrary.
	double overhang = 20.0;
	std::uniform_real_distribution<double> uniformRealDistributionCoordinate(quadTree.GetBoundary(1, true) - overhang, std::nextafter(quadTree.GetBoundary(1, false) + overhang, std::numeric_limits<double>::max()));
	// Generic bool generator.
	std::uniform_int_distribution<size_t> uniformIntDistributionBool(0, 1);


	// Insertion of elements to fixed positions.

	// Helper counter reset.
	counter = {};

	// elementsPerDimensionCount^DIM represents to the total amount of elements to be added.
	for (size_t i = 1; i <= static_cast<size_t>(std::pow(elementsPerDimensionCount, DIM)); ++i)
	{
		// For fixed positions, elements are going to spaced in an even square grid.
		// Counter cycles through all possible combinations of 0 - elementsPerDimensionCount for each dimension.
		// (0, 0, ...) - (elementsPerDimensionCount, elementsPerDimensionCount, ...)
		// Coordinates are then calculated based on that.
		for (size_t j = 0; j < DIM; ++j)
		{
			currentCoordinates[j] = static_cast<double>(counter[j])*step - quadTreeWidth/2.0;
		}

		// Boundaries for elements with id.
		for (size_t j = 0; j < DIM; ++j)
		{
			double boundaryWidth = uniformRealDistributionBoundary(randomNumberGenerator);

			currentBoundaries[2*j] = currentCoordinates[j] - boundaryWidth/2.0;
			currentBoundaries[2*j + 1] = currentCoordinates[j] + boundaryWidth/2.0;
		}

		// Vector containing every element inserted to these specific coordinates.
		std::vector<TestElement<DIM, objT, coordT>> currentElementBatch;

		// One point element with id.

		currentIsPoint = true;
		// Id of point element is just i.
		currentId = static_cast<long long int>(i);
		currentHasId = true;
		currentObject = static_cast<int>(i);

		this->InsertionWrapper_SequenceGenerator(quadTree, currentId, currentObject, currentCoordinates);
		currentElementBatch.emplace_back(currentIsPoint, currentHasId, currentId, currentCoordinates, currentBoundaries, currentObject);
		insertionSuccessful &= quadTree.ValidateTree();

		// One ND element with id.
		currentIsPoint = false;
		// Id of ND element is elementCount + i, so that there is no overlap with point elements' ids.
		currentId = static_cast<long long int>(elementCount) + static_cast<long long int>(i);
		currentHasId = true;
		currentObject = static_cast<int>(elementCount + i);

		this->InsertionWrapper_SequenceGenerator(quadTree, currentId, currentObject, currentBoundaries);
		currentElementBatch.emplace_back(currentIsPoint, currentHasId, currentId, currentCoordinates, currentBoundaries, currentObject);
		insertionSuccessful &= quadTree.ValidateTree();

		// Boundaries for elements without id.
		for (size_t j = 0; j < DIM; ++j)
		{
			double boundaryWidth = uniformRealDistributionBoundary(randomNumberGenerator);

			currentBoundaries[2*j] = currentCoordinates[j] - boundaryWidth/2.0;
			currentBoundaries[2*j + 1] = currentCoordinates[j] + boundaryWidth/2.0;
		}

		// Two point elements without id.

		currentIsPoint = true;
		currentId = 0;
		currentHasId = false;

		currentObject = static_cast<int>(2*elementCount + i);

		this->InsertionWrapper_SequenceGenerator(quadTree, currentId, currentObject, currentCoordinates);
		currentElementBatch.emplace_back(currentIsPoint, currentHasId, currentId, currentCoordinates, currentBoundaries, currentObject);
		insertionSuccessful &= quadTree.ValidateTree();

		currentObject = static_cast<int>(3*elementCount + i);

		this->InsertionWrapper_SequenceGenerator(quadTree, currentId, currentObject, currentCoordinates);
		currentElementBatch.emplace_back(currentIsPoint, currentHasId, currentId, currentCoordinates, currentBoundaries, currentObject);
		insertionSuccessful &= quadTree.ValidateTree();

		// Two ND elements without id.

		currentIsPoint = false;
		currentId = 0;
		currentHasId = false;

		currentObject = static_cast<int>(4*elementCount + i);

		this->InsertionWrapper_SequenceGenerator(quadTree, currentId, currentObject, currentBoundaries);
		currentElementBatch.emplace_back(currentIsPoint, currentHasId, currentId, currentCoordinates, currentBoundaries, currentObject);
		insertionSuccessful &= quadTree.ValidateTree();

		currentObject = static_cast<int>(5*elementCount + i);

		this->InsertionWrapper_SequenceGenerator(quadTree, currentId, currentObject, currentBoundaries);
		currentElementBatch.emplace_back(currentIsPoint, currentHasId, currentId, currentCoordinates, currentBoundaries, currentObject);
		insertionSuccessful &= quadTree.ValidateTree();

		// Current batch of test elements is stored.
		testElements.push_back(std::move(currentElementBatch));

		// Counter is updated to the next combination value.
		for (size_t j = 0; j < 2*DIM; ++j)
		{
			if (counter[j] + 1 >= elementsPerDimensionCount)
			{
				counter[j] = 0;
			}
			else
			{
				++counter[j];
				break;
			}
		}
	}


	// Insertion of elements to random positions.

	// Given external element count is used.
	for (size_t i = 1; i <= elementCount; ++i)
	{
		// Same general idea as with the fixed position cases.

		// Random coordinates must be unique.
		auto coordinatesComparator = [&currentCoordinates](std::vector<TestElement<DIM, objT, coordT>>& elementBatch)
		{
			for (size_t j = 0; j < DIM; ++j)
			{
				if (elementBatch.front().coordinates[j] != currentCoordinates[j])
				{
					return false;
				}
			}

			return true;
		};

		do
		{
			for (size_t j = 0; j < DIM; ++j)
			{
				currentCoordinates[j] = uniformRealDistributionCoordinate(randomNumberGenerator);
			}
		}
		while (std::find_if(testElements.begin(), testElements.end(), coordinatesComparator) != testElements.end());
	
		// Boundaries for elements with id.
		for (size_t j = 0; j < DIM; ++j)
		{
			double boundaryWidth = uniformRealDistributionBoundary(randomNumberGenerator);

			currentBoundaries[2*j] = currentCoordinates[j] - boundaryWidth/2.0;
			currentBoundaries[2*j + 1] = currentCoordinates[j] + boundaryWidth/2.0;
		}

		std::vector<TestElement<DIM, objT, coordT>> currentElementBatch;

		// One point element with id.

		currentIsPoint = true;
		currentId = static_cast<long long int>(6*elementCount) + static_cast<long long int>(i);
		currentHasId = true;
		currentObject = static_cast<int>(6*elementCount + i);

		this->InsertionWrapper_SequenceGenerator(quadTree, currentId, currentObject, currentCoordinates);
		currentElementBatch.emplace_back(currentIsPoint, currentHasId, currentId, currentCoordinates, currentBoundaries, currentObject);
		insertionSuccessful &= quadTree.ValidateTree();

		// One ND element with id.
		currentIsPoint = false;
		currentId = static_cast<long long int>(7*elementCount) + static_cast<long long int>(i);
		currentHasId = true;
		currentObject = static_cast<int>(7*elementCount + i);

		this->InsertionWrapper_SequenceGenerator(quadTree, currentId, currentObject, currentBoundaries);
		currentElementBatch.emplace_back(currentIsPoint, currentHasId, currentId, currentCoordinates, currentBoundaries, currentObject);
		insertionSuccessful &= quadTree.ValidateTree();

		// Boundaries for elements without id.
		for (size_t j = 0; j < DIM; ++j)
		{
			double boundaryWidth = uniformRealDistributionBoundary(randomNumberGenerator);

			currentBoundaries[2*j] = currentCoordinates[j] - boundaryWidth/2.0;
			currentBoundaries[2*j + 1] = currentCoordinates[j] + boundaryWidth/2.0;
		}

		// Two point elements without id.

		currentIsPoint = true;
		currentId = 0;
		currentHasId = false;

		currentObject = static_cast<int>(8*elementCount + i);

		this->InsertionWrapper_SequenceGenerator(quadTree, currentId, currentObject, currentCoordinates);
		currentElementBatch.emplace_back(currentIsPoint, currentHasId, currentId, currentCoordinates, currentBoundaries, currentObject);
		insertionSuccessful &= quadTree.ValidateTree();

		currentObject = static_cast<int>(9*elementCount + i);

		this->InsertionWrapper_SequenceGenerator(quadTree, currentId, currentObject, currentCoordinates);
		currentElementBatch.emplace_back(currentIsPoint, currentHasId, currentId, currentCoordinates, currentBoundaries, currentObject);
		insertionSuccessful &= quadTree.ValidateTree();

		// Two ND elements without id.

		currentIsPoint = false;
		currentId = 0;
		currentHasId = false;

		currentObject = static_cast<int>(10*elementCount + i);

		this->InsertionWrapper_SequenceGenerator(quadTree, currentId, currentObject, currentBoundaries);
		currentElementBatch.emplace_back(currentIsPoint, currentHasId, currentId, currentCoordinates, currentBoundaries, currentObject);
		insertionSuccessful &= quadTree.ValidateTree();

		currentObject = static_cast<int>(11*elementCount + i);

		this->InsertionWrapper_SequenceGenerator(quadTree, currentId, currentObject, currentBoundaries);
		currentElementBatch.emplace_back(currentIsPoint, currentHasId, currentId, currentCoordinates, currentBoundaries, currentObject);
		insertionSuccessful &= quadTree.ValidateTree();

		// Current batch of test elements is stored.
		testElements.push_back(std::move(currentElementBatch));
	}


	// Insertion of elements to random positions, but no duplicates at coordinates.

	// Given external element count is used, but divided by 5.
	for (size_t i = 1; i <= elementCount/5; ++i)
	{
		// Same general idea as with the previous random position cases.

		auto coordinatesComparator = [&currentCoordinates](std::vector<TestElement<DIM, objT, coordT>>& elementBatch)
		{
			for (size_t j = 0; j < DIM; ++j)
			{
				if (elementBatch.front().coordinates[j] != currentCoordinates[j])
				{
					return false;
				}
			}

			return true;
		};

		do
		{
			for (size_t j = 0; j < DIM; ++j)
			{
				currentCoordinates[j] = uniformRealDistributionCoordinate(randomNumberGenerator);
			}
		}
		while (std::find_if(testElements.begin(), testElements.end(), coordinatesComparator) != testElements.end());
	
		for (size_t j = 0; j < DIM; ++j)
		{
			double boundaryWidth = uniformRealDistributionBoundary(randomNumberGenerator);

			currentBoundaries[2*j] = currentCoordinates[j] - boundaryWidth/2.0;
			currentBoundaries[2*j + 1] = currentCoordinates[j] + boundaryWidth/2.0;
		}

		std::vector<TestElement<DIM, objT, coordT>> currentElementBatch;

		currentObject = static_cast<int>(12*elementCount + i);

		if (uniformIntDistributionBool(randomNumberGenerator) == 0)
		{
			// With id.

			currentId = static_cast<long long int>(12*elementCount) + static_cast<long long int>(i);
			currentHasId = true;

		}
		else
		{
			// Without id.

			currentId = 0;
			currentHasId = false;
		}

		if (uniformIntDistributionBool(randomNumberGenerator) == 0)
		{
			// Point.

			currentIsPoint = true;

			this->InsertionWrapper_SequenceGenerator(quadTree, currentId, currentObject, currentCoordinates);
			currentElementBatch.emplace_back(currentIsPoint, currentHasId, currentId, currentCoordinates, currentBoundaries, currentObject);
			insertionSuccessful &= quadTree.ValidateTree();
		}
		else
		{
			// ND.

			currentIsPoint = false;

			this->InsertionWrapper_SequenceGenerator(quadTree, currentId, currentObject, currentBoundaries);
			currentElementBatch.emplace_back(currentIsPoint, currentHasId, currentId, currentCoordinates, currentBoundaries, currentObject);
			insertionSuccessful &= quadTree.ValidateTree();
		}

		testElements.push_back(std::move(currentElementBatch));
	}


	if (insertionSuccessful == true)
	{
		std::cout << "Insertion test successful!" << std::endl;

		return true;
	}
	else
	{
		std::cout << "Insertion test not successful!" << std::endl;

		return false;
	}
}

template<size_t... indexes, size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT, size_t N>
std::vector<objT> QuadTreeTester::DeletionWrapper_Caller([[maybe_unused]] std::index_sequence<indexes...> indexSequence, QuadTree<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>& quadTree, long long int& id, std::array<coordT, N>& parameterArray) const
{
	if constexpr (N == DIM)
	{
		return quadTree.DeleteAndReturnPointElement(id, parameterArray[indexes]...);
	}
	else
	{
		return quadTree.DeleteAndReturnNDElement(id, parameterArray[indexes]...);
	}
}

template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT, size_t N>
std::vector<objT> QuadTreeTester::DeletionWrapper_SequenceGenerator(QuadTree<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>& quadTree, long long int& id, std::array<coordT, N>& parameterArray) const
{
	return this->DeletionWrapper_Caller(std::make_index_sequence<N>(), quadTree, id, parameterArray);
}

template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
bool QuadTreeTester::TestDeletion(QuadTree<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>& quadTree, std::vector<std::vector<TestElement<DIM, objT, coordT>>>& testElements) const
{
	bool deletionSuccessful = true;

	// Random generators.
	unsigned int seed = 1;
	std::mt19937_64 randomNumberGenerator(seed);
	// Random size_t.
	std::uniform_int_distribution<size_t> uniformIntDistributionRandomSizet(0, std::numeric_limits<size_t>::max());
	// Coordinate generator for random positions. Assumes that quad tree is a square.
	std::uniform_real_distribution<double> uniformRealDistributionCoordinate(quadTree.GetBoundary(1, true), std::nextafter(quadTree.GetBoundary(1, false), std::numeric_limits<double>::max()));


	// First, a deletion of nonexistent elements will be tested.
	for (size_t i = 1; i <= 100; ++i)
	{
		// Coordinate generation is taken from the insertion test.
		std::array<coordT, DIM> currentCoordinates = {};
		std::array<coordT, 2*DIM> currentBoundaries = {};

		auto coordinatesComparator = [&currentCoordinates](std::vector<TestElement<DIM, objT, coordT>>& elementBatch)
		{
			for (size_t j = 0; j < DIM; ++j)
			{
				if (elementBatch.front().coordinates[j] != currentCoordinates[j])
				{
					return false;
				}
			}

			return true;
		};

		do
		{
			for (size_t j = 0; j < DIM; ++j)
			{
				currentCoordinates[j] = uniformRealDistributionCoordinate(randomNumberGenerator);

				currentBoundaries[2*j] = currentCoordinates[j];
				currentBoundaries[2*j + 1] = currentCoordinates[j];
			}
		}
		while (std::find_if(testElements.begin(), testElements.end(), coordinatesComparator) != testElements.end());

		long long int id = 0;

		std::vector<objT> returnVector;

		// Point, no id.
		id = 0;
		returnVector = this->DeletionWrapper_SequenceGenerator(quadTree, id, currentCoordinates);
		deletionSuccessful &= returnVector.empty() == true;
		deletionSuccessful &= quadTree.ValidateTree();

		// Point, id.
		id = 1;
		returnVector = this->DeletionWrapper_SequenceGenerator(quadTree, id, currentCoordinates);
		deletionSuccessful &= returnVector.empty() == true;
		deletionSuccessful &= quadTree.ValidateTree();

		// ND, no id.
		id = 0;
		returnVector = this->DeletionWrapper_SequenceGenerator(quadTree, id, currentBoundaries);
		deletionSuccessful &= returnVector.empty() == true;
		deletionSuccessful &= quadTree.ValidateTree();

		// ND, id.
		id = 1;
		returnVector = this->DeletionWrapper_SequenceGenerator(quadTree, id, currentBoundaries);
		deletionSuccessful &= returnVector.empty() == true;
		deletionSuccessful &= quadTree.ValidateTree();

		if (deletionSuccessful == false)
		{
			break;
		}
	}


	// Stored elements are going to be deleted randomly until they are all removed.
	while (testElements.empty() == false && deletionSuccessful == true)
	{
		// Random element batch is chosen.
		size_t randomBatchIndex = uniformIntDistributionRandomSizet(randomNumberGenerator) % testElements.size();
		std::vector<TestElement<DIM, objT, coordT>>& currentBatch = testElements[randomBatchIndex];
		using diffTypeBatch = typename std::vector<std::vector<TestElement<DIM, objT, coordT>>>::difference_type;

		// Random element from the batch is chosen.
		size_t randomElementIndex = uniformIntDistributionRandomSizet(randomNumberGenerator) % currentBatch.size();
		// Copy because of std::partition later.
		TestElement<DIM, objT, coordT> currentElement = currentBatch[randomElementIndex];
		using diffTypeElement = typename std::vector<TestElement<DIM, objT, coordT>>::difference_type;

		// Check for element outside bounds.
		bool elementOutsideBounds = false;

		for (size_t i = 0; i < DIM; ++i)
		{
			if (currentElement.coordinates[i] < quadTree.GetBoundary(i + 1, true) || currentElement.coordinates[i] > quadTree.GetBoundary(i + 1, false))
			{
				elementOutsideBounds = true;

				break;
			}
		}

		if (currentElement.isPoint == true)
		{
			// Point element deletion.

			std::vector<objT> returnVector = this->DeletionWrapper_SequenceGenerator(quadTree, currentElement.id, currentElement.coordinates);
			deletionSuccessful &= quadTree.ValidateTree();

			if (elementOutsideBounds == true)
			{
				if (returnVector.empty() == true)
				{
					testElements.erase(testElements.begin() + static_cast<diffTypeBatch>(randomBatchIndex));

					continue;
				}
				else
				{
					deletionSuccessful = false;
				}
			}

			if (deletionSuccessful == false)
			{
				break;
			}

			if (currentElement.hasId == true)
			{
				// Element has id. The returned vector has to contain 1 element and the object has to match.

				if (returnVector.size() == 1 && returnVector.front() == currentElement.object)
				{
					// Element was found. It is also erased from the current test batch.

					currentBatch.erase(currentBatch.begin() + static_cast<diffTypeElement>(randomElementIndex));
				}
				else
				{
					deletionSuccessful = false;

					break;
				}
			}
			else
			{
				// Element doesn't have an id. All point elements from this batch should be removed.

				// Element to be removed are going to be moved to the end of the batch.
				// (Coordinates must be checked in a case there is a numerical error in this batch and the elements do not lie exactly at the same coordinates.)
				auto partitionIt = std::partition(currentBatch.begin(), currentBatch.end(), [&currentElement](TestElement<DIM, objT, coordT>& element){return element.isPoint == currentElement.isPoint && element.coordinates == currentElement.coordinates ? !true : !false;});

				// Number of removed elements from the batch must be the same as number of elements removed from the quad tree.
				if (static_cast<size_t>(currentBatch.end() - partitionIt) != returnVector.size())
				{
					deletionSuccessful = false;

					break;
				}

				// Object values must match as well. (Object values should be unique from the insertion test.)
				for (auto it = partitionIt; it != currentBatch.end(); ++it)
				{
					deletionSuccessful &= std::find_if(returnVector.begin(), returnVector.end(), [&it](objT object){return object == it->object;}) != returnVector.end();
				}

				if (deletionSuccessful == false)
				{
					break;
				}

				// All found elements will be removed from this batch.
				currentBatch.erase(partitionIt, currentBatch.end());
			}
		}
		else
		{
			// Equivalent for ND element.

			std::vector<objT> returnVector = this->DeletionWrapper_SequenceGenerator(quadTree, currentElement.id, currentElement.boundaries);
			deletionSuccessful &= quadTree.ValidateTree();

			if (elementOutsideBounds == true)
			{
				if (returnVector.empty() == true)
				{
					testElements.erase(testElements.begin() + static_cast<diffTypeBatch>(randomBatchIndex));

					continue;
				}
				else
				{
					deletionSuccessful = false;
				}
			}

			if (deletionSuccessful == false)
			{
				break;
			}

			if (currentElement.hasId == true)
			{
				if (returnVector.size() == 1 && returnVector.front() == currentElement.object)
				{
					currentBatch.erase(currentBatch.begin() + static_cast<diffTypeElement>(randomElementIndex));
				}
				else
				{
					deletionSuccessful = false;

					break;
				}
			}
			else
			{
				// Return values are negated because matching elements should be after the non matching ones.
				auto comparator = [&currentElement](TestElement<DIM, objT, coordT>& element)
				{
					if (currentElement.isPoint == true)
					{
						if (element.isPoint == true && element.coordinates == currentElement.coordinates)
						{
							return !true;
						}
						else
						{
							return !false;
						}
					}
					else
					{
						if (element.isPoint == false && element.boundaries == currentElement.boundaries)
						{
							return !true;
						}
						else
						{
							return !false;
						}
					}
				};

				auto partitionIt = std::partition(currentBatch.begin(), currentBatch.end(), comparator);

				if (static_cast<size_t>(currentBatch.end() - partitionIt) != returnVector.size())
				{
					deletionSuccessful = false;

					break;
				}

				for (auto it = partitionIt; it != currentBatch.end(); ++it)
				{
					deletionSuccessful &= std::find_if(returnVector.begin(), returnVector.end(), [&it](objT object){return object == it->object;}) != returnVector.end();
				}

				if (deletionSuccessful == false)
				{
					break;
				}

				currentBatch.erase(partitionIt, currentBatch.end());
			}
		}

		if (currentBatch.empty() == true)
		{
			testElements.erase(testElements.begin() + static_cast<diffTypeBatch>(randomBatchIndex));
		}
	}


	// At the end of the deletion, both the testElements vector and the quad tree have to be empty.
	deletionSuccessful &= testElements.empty() == true;
	deletionSuccessful &= quadTree.Size() == 0;


	if (deletionSuccessful == true)
	{
		std::cout << "Deletion test successful!" << std::endl;

		return true;
	}
	else
	{
		std::cout << "Deletion test not successful!" << std::endl;

		return false;
	}
}

template<size_t... indexes, size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT, size_t N>
bool QuadTreeTester::RelocationWrapper_Caller([[maybe_unused]] std::index_sequence<indexes...> indexSequence, QuadTree<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>& quadTree, long long int& id, std::array<coordT, N>& firstParameterArray, std::array<coordT, N>& secondParameterArray) const
{
	if constexpr (N == DIM)
	{
		return quadTree.RelocatePointElement(id, firstParameterArray[indexes]..., secondParameterArray[indexes]...);
	}
	else
	{
		return quadTree.RelocateNDElement(id, firstParameterArray[indexes]..., secondParameterArray[indexes]...);
	}
}

template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT, size_t N>
bool QuadTreeTester::RelocationWrapper_SequenceGenerator(QuadTree<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>& quadTree, long long int& id, std::array<coordT, N>& firstParameterArray, std::array<coordT, N>& secondParameterArray) const
{
	return this->RelocationWrapper_Caller(std::make_index_sequence<N>(), quadTree, id, firstParameterArray, secondParameterArray);
}

template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
bool QuadTreeTester::TestRelocation(QuadTree<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>& quadTree, std::vector<std::vector<TestElement<DIM, objT, coordT>>>& testElements) const
{
	bool relocationSuccessful = true;

	// Random generators.
	unsigned int seed = 1;
	std::mt19937_64 randomNumberGenerator(seed);
	// Boundary generator for range elements.
	double boundaryMaxWidth = 100.0;
	std::uniform_real_distribution<double> uniformRealDistributionBoundary(0, std::nextafter(boundaryMaxWidth, std::numeric_limits<double>::max()));
	// Random size_t.
	std::uniform_int_distribution<size_t> uniformIntDistributionRandomSizet(0, std::numeric_limits<size_t>::max());
	// Coordinate generator for random positions. Assumes that quad tree is a square.
	std::uniform_real_distribution<double> uniformRealDistributionCoordinate(quadTree.GetBoundary(1, true), std::nextafter(quadTree.GetBoundary(1, false), std::numeric_limits<double>::max()));


	// Quarter of elements will be relocated to existing coordinates and another quarter of elements
	// will be relocated to new coordinates.

	size_t testElementsSizeQuarter = testElements.size()/4;

	// Elements used for relocation tests will be moved to separate vectors.
	std::vector<std::vector<TestElement<DIM, objT, coordT>>> testElementsExistingCoordinatesRelocation;
	std::vector<std::vector<TestElement<DIM, objT, coordT>>> testElementsNewCoordinatesRelocation;

	// Vectors are filled with random elements from test vector.
	using diffTypeBatch = typename std::vector<std::vector<TestElement<DIM, objT, coordT>>>::difference_type;

	while (testElementsNewCoordinatesRelocation.size() < testElementsSizeQuarter)
	{
		size_t randomBatchIndex = uniformIntDistributionRandomSizet(randomNumberGenerator) % testElements.size();

		testElementsNewCoordinatesRelocation.push_back(testElements[randomBatchIndex]);
		testElements.erase(testElements.begin() + static_cast<diffTypeBatch>(randomBatchIndex));
	}

	while (testElementsExistingCoordinatesRelocation.size() < testElementsSizeQuarter)
	{
		size_t randomBatchIndex = uniformIntDistributionRandomSizet(randomNumberGenerator) % testElements.size();

		testElementsExistingCoordinatesRelocation.push_back(testElements[randomBatchIndex]);
		testElements.erase(testElements.begin() + static_cast<diffTypeBatch>(randomBatchIndex));
	}


	// First, a relocation to invalid coordinates will be tested.
	for (size_t i = 1; i <= 100; ++i)
	{
		// Random element batch is chosen.
		size_t randomBatchIndex = uniformIntDistributionRandomSizet(randomNumberGenerator) % testElements.size();
		std::vector<TestElement<DIM, objT, coordT>>& currentBatch = testElements[randomBatchIndex];

		// Random element from the batch is chosen.
		size_t randomElementIndex = uniformIntDistributionRandomSizet(randomNumberGenerator) % currentBatch.size();
		TestElement<DIM, objT, coordT>& currentElement = currentBatch[randomElementIndex];

		// Invalid target coordinates are generated.
		std::array<coordT, DIM> newCoordinates = {};
		std::array<coordT, 2*DIM> newBoundaries = {};

		for (size_t j = 0; j < DIM; ++j)
		{
			newCoordinates[j] = quadTree.GetBoundary(j + 1, false) + 100.0;

			newBoundaries[2*j] = newCoordinates[j] - 50.0;
			newBoundaries[2*j + 1] = newCoordinates[j] + 50.0;
		}

		// False is always expected as return value.

		long long int id = 0;

		if (currentElement.isPoint == true)
		{
			if (currentElement.hasId == true)
			{
				id = currentElement.id;
				relocationSuccessful &= !(this->RelocationWrapper_SequenceGenerator(quadTree, id, currentElement.coordinates, newCoordinates));
				relocationSuccessful &= quadTree.ValidateTree();
			}
			else
			{
				relocationSuccessful &= !(this->RelocationWrapper_SequenceGenerator(quadTree, id, currentElement.boundaries, newBoundaries));
				relocationSuccessful &= quadTree.ValidateTree();
			}
		}
		else
		{
			if (currentElement.hasId == true)
			{
				id = currentElement.id;
				relocationSuccessful &= !(this->RelocationWrapper_SequenceGenerator(quadTree, id, currentElement.coordinates, newCoordinates));
				relocationSuccessful &= quadTree.ValidateTree();
			}
			else
			{
				relocationSuccessful &= !(this->RelocationWrapper_SequenceGenerator(quadTree, id, currentElement.boundaries, newBoundaries));
				relocationSuccessful &= quadTree.ValidateTree();
			}
		}

		if (relocationSuccessful == false)
		{
			break;
		}
	}


	// Chosen elements will be moved to new coordinates one by one.
	while (testElementsNewCoordinatesRelocation.empty() == false && relocationSuccessful == true)
	{
		// Random element batch is chosen.
		size_t randomBatchIndex = uniformIntDistributionRandomSizet(randomNumberGenerator) % testElementsNewCoordinatesRelocation.size();
		std::vector<TestElement<DIM, objT, coordT>>& currentBatch = testElementsNewCoordinatesRelocation[randomBatchIndex];

		// Random element from the batch is chosen.
		size_t randomElementIndex = uniformIntDistributionRandomSizet(randomNumberGenerator) % currentBatch.size();
		// Copy because of std::partition later.
		TestElement<DIM, objT, coordT> currentElement = currentBatch[randomElementIndex];
		using diffTypeElement = typename std::vector<TestElement<DIM, objT, coordT>>::difference_type;

		// Not yet existing random target coordinates are generated.
		std::array<coordT, DIM> newCoordinates = {};
		std::array<coordT, 2*DIM> newBoundaries = {};

		auto coordinatesComparator = [&newCoordinates](std::vector<TestElement<DIM, objT, coordT>>& elementBatch)
		{
			for (size_t i = 0; i < DIM; ++i)
			{
				if (elementBatch.front().coordinates[i] != newCoordinates[i])
				{
					return false;
				}
			}

			return true;
		};

		do
		{
			bool standardGeneration = true;

			// 25% of the time, the random target coordinates are artifically generated
			// close to the source coordinates to test relocation within a single node.
			if (randomBatchIndex % 4 == 0)
			{
				standardGeneration = false;

				for (size_t j = 0; j < DIM; ++j)
				{
					coordT randomCoordinate = uniformRealDistributionCoordinate(randomNumberGenerator);
					coordT randomSmallOffset = randomCoordinate - std::round(randomCoordinate);

					newCoordinates[j] = currentElement.coordinates[j] + randomSmallOffset;

					double boundaryWidth = uniformRealDistributionBoundary(randomNumberGenerator);

					newBoundaries[2*j] = newCoordinates[j] - boundaryWidth/2.0;
					newBoundaries[2*j + 1] = newCoordinates[j] + boundaryWidth/2.0;

					// If any coordinate is outside bounds, then the current progress is scraped
					// and the coordinates are generated again the normal way.
					if (newCoordinates[j] < quadTree.GetBoundary(j + 1, true) || newCoordinates[j] > quadTree.GetBoundary(j + 1, false))
					{
						standardGeneration = true;
						break;
					}
				}
			}

			if (standardGeneration == true)
			{
				for (size_t j = 0; j < DIM; ++j)
				{
					newCoordinates[j] = uniformRealDistributionCoordinate(randomNumberGenerator);

					double boundaryWidth = uniformRealDistributionBoundary(randomNumberGenerator);

					newBoundaries[2*j] = newCoordinates[j] - boundaryWidth/2.0;
					newBoundaries[2*j + 1] = newCoordinates[j] + boundaryWidth/2.0;
				}
			}
		}
		while (std::find_if(testElements.begin(), testElements.end(), coordinatesComparator) != testElements.end());

		// Check if the current element lies outside bounds.
		bool elementOutsideBounds = false;

		for (size_t i = 0; i < DIM; ++i)
		{
			if (currentElement.coordinates[i] < quadTree.GetBoundary(i + 1, true) || currentElement.coordinates[i] > quadTree.GetBoundary(i + 1, false))
			{
				elementOutsideBounds = true;

				break;
			}
		}

		if (currentElement.isPoint == true)
		{
			// Point element relocation.

			bool returnValue = this->RelocationWrapper_SequenceGenerator(quadTree, currentElement.id, currentElement.coordinates, newCoordinates);
			relocationSuccessful &= quadTree.ValidateTree();

			if (elementOutsideBounds == true)
			{
				// If element is outside bounds, then false should be returned.

				if (returnValue == false)
				{
					// Whole batch is moved back to the original vector as is.
					testElements.push_back(currentBatch);
					testElementsNewCoordinatesRelocation.erase(testElementsNewCoordinatesRelocation.begin() + static_cast<diffTypeBatch>(randomBatchIndex));

					continue;
				}
				else
				{
					relocationSuccessful = false;
				}
			}

			if (relocationSuccessful == false)
			{
				break;
			}

			if (currentElement.hasId == true)
			{
				// Element has id. It is updated and moved back to the original vector.

				if (returnValue == true)
				{
					currentElement.coordinates = newCoordinates;

					// Coordinates do not exist, so a new batch has to be created.
					testElements.emplace_back();
					testElements.back().push_back(currentElement);

					currentBatch.erase(currentBatch.begin() + static_cast<diffTypeElement>(randomElementIndex));
				}
				else
				{
					relocationSuccessful = false;

					break;
				}
			}
			else
			{
				// Element doesn't have an id. All point elements from this batch should be relocated.

				// Elements to be relocated are going to be moved to the end of the batch.
				// (Coordinates must be checked in a case there is a numerical error in this batch and the elements do not lie exactly at the same coordinates.)
				auto partitionIt = std::partition(currentBatch.begin(), currentBatch.end(), [&currentElement](TestElement<DIM, objT, coordT>& element){return element.isPoint == currentElement.isPoint && element.coordinates == currentElement.coordinates ? !true : !false;});

				// Relocation should be successful in this case.
				if (returnValue == false)
				{
					relocationSuccessful = false;

					break;
				}
	
				// Coordinates do not exist, so a new batch has to be created.
				testElements.emplace_back();

				// Elements are copied into it.
				for (auto it = partitionIt; it != currentBatch.end(); ++it)
				{
					it->coordinates = newCoordinates;

					testElements.back().push_back(*it);
				}

				// Elements are removed from the current batch.
				currentBatch.erase(partitionIt, currentBatch.end());
			}
		}
		else
		{
			// Equivalent for ND element.

			bool returnValue = this->RelocationWrapper_SequenceGenerator(quadTree, currentElement.id, currentElement.boundaries, newBoundaries);
			relocationSuccessful &= quadTree.ValidateTree();

			if (elementOutsideBounds == true)
			{
				if (returnValue == false)
				{
					testElements.push_back(currentBatch);
					testElementsNewCoordinatesRelocation.erase(testElementsNewCoordinatesRelocation.begin() + static_cast<diffTypeBatch>(randomBatchIndex));

					continue;
				}
				else
				{
					relocationSuccessful = false;
				}
			}

			if (relocationSuccessful == false)
			{
				break;
			}

			if (currentElement.hasId == true)
			{
				if (returnValue == true)
				{
					currentElement.boundaries = newBoundaries;

					testElements.emplace_back();
					testElements.back().push_back(currentElement);

					currentBatch.erase(currentBatch.begin() + static_cast<diffTypeElement>(randomElementIndex));
				}
				else
				{
					relocationSuccessful = false;

					break;
				}
			}
			else
			{
				// Return values are negated because matching elements should be after the non matching ones.
				auto comparator = [&currentElement](TestElement<DIM, objT, coordT>& element)
				{
					if (currentElement.isPoint == true)
					{
						if (element.isPoint == true && element.coordinates == currentElement.coordinates)
						{
							return !true;
						}
						else
						{
							return !false;
						}
					}
					else
					{
						if (element.isPoint == false && element.boundaries == currentElement.boundaries)
						{
							return !true;
						}
						else
						{
							return !false;
						}
					}
				};

				auto partitionIt = std::partition(currentBatch.begin(), currentBatch.end(), comparator);

				if (returnValue == false)
				{
					relocationSuccessful = false;

					break;
				}

				testElements.emplace_back();

				for (auto it = partitionIt; it != currentBatch.end(); ++it)
				{
					it->boundaries = newBoundaries;

					testElements.back().push_back(*it);
				}

				currentBatch.erase(partitionIt, currentBatch.end());
			}
		}

		if (currentBatch.empty() == true)
		{
			testElementsNewCoordinatesRelocation.erase(testElementsNewCoordinatesRelocation.begin() + static_cast<diffTypeBatch>(randomBatchIndex));
		}
	}


	// Chosen elements will be moved to existing coordinates one by one.
	while (testElementsExistingCoordinatesRelocation.empty() == false && relocationSuccessful == true)
	{
		// Same handling as new coordinates case, except for coordinates generation and test element vector handling.

		size_t randomBatchIndex = uniformIntDistributionRandomSizet(randomNumberGenerator) % testElementsExistingCoordinatesRelocation.size();
		std::vector<TestElement<DIM, objT, coordT>>& currentBatch = testElementsExistingCoordinatesRelocation[randomBatchIndex];

		size_t randomElementIndex = uniformIntDistributionRandomSizet(randomNumberGenerator) % currentBatch.size();
		TestElement<DIM, objT, coordT> currentElement = currentBatch[randomElementIndex];
		using diffTypeElement = typename std::vector<TestElement<DIM, objT, coordT>>::difference_type;

		// Already existing target coordinates are retrieved.
		std::array<coordT, DIM> newCoordinates = {};
		std::array<coordT, 2*DIM> newBoundaries = {};

		// Existing ND element will be found from which both the coordinates and boundaries will be extracted.
		bool existingNDElementFound = false;
		size_t targetBatchIndex = 0;
		
		for (size_t i = 0; i < testElements.size(); ++i)
		{
			for (size_t j = 0; j < testElements[i].size(); ++j)
			{
				if (testElements[i][j].isPoint == false)
				{
					newCoordinates = testElements[i][j].coordinates;
					newBoundaries = testElements[i][j].boundaries;

					existingNDElementFound = true;
					break;
				}
			}

			if (existingNDElementFound == true)
			{
				targetBatchIndex = i;
				break;
			}
		}

		bool elementOutsideBounds = false;

		for (size_t i = 0; i < DIM; ++i)
		{
			if (currentElement.coordinates[i] < quadTree.GetBoundary(i + 1, true) || currentElement.coordinates[i] > quadTree.GetBoundary(i + 1, false))
			{
				elementOutsideBounds = true;

				break;
			}
		}

		if (currentElement.isPoint == true)
		{
			bool returnValue = this->RelocationWrapper_SequenceGenerator(quadTree, currentElement.id, currentElement.coordinates, newCoordinates);
			relocationSuccessful &= quadTree.ValidateTree();

			if (elementOutsideBounds == true)
			{
				if (returnValue == false)
				{
					testElements.push_back(currentBatch);
					testElementsExistingCoordinatesRelocation.erase(testElementsExistingCoordinatesRelocation.begin() + static_cast<diffTypeBatch>(randomBatchIndex));

					continue;
				}
				else
				{
					relocationSuccessful = false;
				}
			}

			if (relocationSuccessful == false)
			{
				break;
			}

			if (currentElement.hasId == true)
			{
				if (returnValue == true)
				{
					currentElement.coordinates = newCoordinates;

					testElements[targetBatchIndex].push_back(currentElement);

					currentBatch.erase(currentBatch.begin() + static_cast<diffTypeElement>(randomElementIndex));
				}
				else
				{
					relocationSuccessful = false;

					break;
				}
			}
			else
			{
				auto partitionIt = std::partition(currentBatch.begin(), currentBatch.end(), [&currentElement](TestElement<DIM, objT, coordT>& element){return element.isPoint == currentElement.isPoint && element.coordinates == currentElement.coordinates ? !true : !false;});

				if (returnValue == false)
				{
					relocationSuccessful = false;

					break;
				}
	
				for (auto it = partitionIt; it != currentBatch.end(); ++it)
				{
					it->coordinates = newCoordinates;

					testElements[targetBatchIndex].push_back(*it);
				}

				currentBatch.erase(partitionIt, currentBatch.end());
			}
		}
		else
		{
			bool returnValue = this->RelocationWrapper_SequenceGenerator(quadTree, currentElement.id, currentElement.boundaries, newBoundaries);
			relocationSuccessful &= quadTree.ValidateTree();

			if (elementOutsideBounds == true)
			{
				if (returnValue == false)
				{
					testElements.push_back(currentBatch);
					testElementsExistingCoordinatesRelocation.erase(testElementsExistingCoordinatesRelocation.begin() + static_cast<diffTypeBatch>(randomBatchIndex));

					continue;
				}
				else
				{
					relocationSuccessful = false;
				}
			}

			if (relocationSuccessful == false)
			{
				break;
			}

			if (currentElement.hasId == true)
			{
				if (returnValue == true)
				{
					currentElement.boundaries = newBoundaries;

					testElements[targetBatchIndex].push_back(currentElement);

					currentBatch.erase(currentBatch.begin() + static_cast<diffTypeElement>(randomElementIndex));
				}
				else
				{
					relocationSuccessful = false;

					break;
				}
			}
			else
			{
				auto comparator = [&currentElement](TestElement<DIM, objT, coordT>& element)
				{
					if (currentElement.isPoint == true)
					{
						if (element.isPoint == true && element.coordinates == currentElement.coordinates)
						{
							return !true;
						}
						else
						{
							return !false;
						}
					}
					else
					{
						if (element.isPoint == false && element.boundaries == currentElement.boundaries)
						{
							return !true;
						}
						else
						{
							return !false;
						}
					}
				};

				auto partitionIt = std::partition(currentBatch.begin(), currentBatch.end(), comparator);

				if (returnValue == false)
				{
					relocationSuccessful = false;

					break;
				}

				for (auto it = partitionIt; it != currentBatch.end(); ++it)
				{
					it->boundaries = newBoundaries;

					testElements[targetBatchIndex].push_back(*it);
				}

				currentBatch.erase(partitionIt, currentBatch.end());
			}
		}

		if (currentBatch.empty() == true)
		{
			testElementsExistingCoordinatesRelocation.erase(testElementsExistingCoordinatesRelocation.begin() + static_cast<diffTypeBatch>(randomBatchIndex));
		}
	}


	if (relocationSuccessful == true)
	{
		std::cout << "Relocation test successful!" << std::endl;

		return true;
	}
	else
	{
		std::cout << "Relocation test not successful!" << std::endl;

		return false;
	}
}

template<size_t... indexes, size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT, size_t N>
std::vector<objT> QuadTreeTester::LocationQueryWrapper_Caller([[maybe_unused]] std::index_sequence<indexes...> indexSequence, QuadTree<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>& quadTree, std::array<coordT, N>& parameterArray) const
{
	if constexpr (N == DIM)
	{
		return quadTree.PointQuery(parameterArray[indexes]...);
	}
	else
	{
		return quadTree.NDQuery(parameterArray[indexes]...);
	}
}

template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT, size_t N>
std::vector<objT> QuadTreeTester::LocationQueryWrapper_SequenceGenerator(QuadTree<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>& quadTree, std::array<coordT, N>& parameterArray) const
{
	return this->LocationQueryWrapper_Caller(std::make_index_sequence<N>(), quadTree, parameterArray);
}

template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
bool QuadTreeTester::TestLocationQueries(QuadTree<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>& quadTree, std::vector<std::vector<TestElement<DIM, objT, coordT>>>& testElements) const
{
	bool testSuccessful = true;

	// Quad tree is assumed to be a square.

	// Random generators.
	unsigned int seed = 1;
	std::mt19937_64 randomNumberGenerator(seed);
	// Query midpoint generator.
	std::uniform_real_distribution<double> uniformRealDistributionQueryMidpoint(quadTree.GetBoundary(1, true) - 25.0, std::nextafter(quadTree.GetBoundary(1, false) + 25.0, std::numeric_limits<double>::max()));
	// Query width generator.
	double queryMaxWidth = 100.0;
	std::uniform_real_distribution<double> uniformRealDistributionQueryWidth(0, std::nextafter(queryMaxWidth, std::numeric_limits<double>::max()));


	size_t queryTestsCount = 1000;


	// Random ND and point query.

	// Query window parameters.
	std::array<double, DIM> queryMidpoint = {};
	std::array<double, 2*DIM> queryBoundaries = {};

	for (size_t i = 0; i < queryTestsCount; ++i)
	{
		// Random query window is generated.
		for (size_t j = 0; j < DIM; ++j)
		{
			double randomMidpoint = uniformRealDistributionQueryMidpoint(randomNumberGenerator);
			double randomWidth = uniformRealDistributionQueryWidth(randomNumberGenerator);

			queryMidpoint[j] = randomMidpoint;

			queryBoundaries[2*j] = randomMidpoint - randomWidth/2.0;
			queryBoundaries[2*j + 1] = randomMidpoint + randomWidth/2.0;
		}


		// ND query.
		std::vector<objT> returnVector_NDQuery = this->LocationQueryWrapper_SequenceGenerator(quadTree, queryBoundaries);
		testSuccessful &= quadTree.ValidateTree();

		// Equivalent query performed on testElements vector.
		std::vector<objT> returnVector_NDElements;

		// Boundaries used for both ND and point elements.
		std::array<double, 2*DIM> elementBoundaries;

		// Each element in each batch is tested.
		for (auto& batch : testElements)
		{
			for (auto& element : batch)
			{
				if (element.isPoint == true)
				{
					// For point element, the min and max boundary will be the same.

					for (size_t j = 0; j < DIM; ++j)
					{
						elementBoundaries[2*j] = element.coordinates[j];
						elementBoundaries[2*j + 1] = element.coordinates[j];
					}
				}
				else
				{
					elementBoundaries = element.boundaries;
				}

				// Test if element is inside ND volume.
				if (DoAxisAlignedNDVolumesIntersect<DIM>(queryBoundaries, elementBoundaries) == true)
				{
					// Element is only taken into consideration if it is inside bounds.
					bool elementOutsideBounds = false;

					for (size_t j = 0; j < DIM; ++j)
					{
						if (element.coordinates[j] < quadTree.GetBoundary(j + 1, true) || element.coordinates[j] > quadTree.GetBoundary(j + 1, false))
						{
							elementOutsideBounds = true;

							break;
						}
					}

					if (elementOutsideBounds == false)
					{
						returnVector_NDElements.push_back(element.object);
					}
				}
			}
		}

		// Test that return vectors are equivalent.
		// Objects are unique, so each element is found in both vectors. Then they are both removed.
		if (returnVector_NDQuery.size() == returnVector_NDElements.size())
		{
			while (returnVector_NDQuery.empty() == false)
			{
				auto matchingElementIt = std::find(returnVector_NDElements.begin(), returnVector_NDElements.end(), returnVector_NDQuery.back());

				if (matchingElementIt != returnVector_NDElements.end())
				{
					returnVector_NDQuery.pop_back();
					returnVector_NDElements.erase(matchingElementIt);
				}
				else
				{
					testSuccessful = false;
					break;
				}
			}
		}
		else
		{
			testSuccessful = false;
		}


		// For point query, min and max of the query window are the asme as midpoint.
		for (size_t j = 0; j < DIM; ++j)
		{
			queryBoundaries[2*j] = queryMidpoint[j];
			queryBoundaries[2*j + 1] = queryMidpoint[j];
		}

		// Point query. Same process as ND query.
		std::vector<objT> returnVector_PointQuery = this->LocationQueryWrapper_SequenceGenerator(quadTree, queryMidpoint);
		testSuccessful &= quadTree.ValidateTree();

		std::vector<objT> returnVector_PointElements;

		for (auto& batch : testElements)
		{
			for (auto& element : batch)
			{
				if (element.isPoint == true)
				{
					for (size_t j = 0; j < DIM; ++j)
					{
						elementBoundaries[2*j] = element.coordinates[j];
						elementBoundaries[2*j + 1] = element.coordinates[j];
					}
				}
				else
				{
					elementBoundaries = element.boundaries;
				}

				if (DoAxisAlignedNDVolumesIntersect<DIM>(queryBoundaries, elementBoundaries) == true)
				{
					bool elementOutsideBounds = false;

					for (size_t j = 0; j < DIM; ++j)
					{
						if (element.coordinates[j] < quadTree.GetBoundary(j + 1, true) || element.coordinates[j] > quadTree.GetBoundary(j + 1, false))
						{
							elementOutsideBounds = true;

							break;
						}
					}

					if (elementOutsideBounds == false)
					{
						returnVector_PointElements.push_back(element.object);
					}
				}
			}
		}

		if (returnVector_PointQuery.size() == returnVector_PointElements.size())
		{
			while (returnVector_PointQuery.empty() == false)
			{
				auto matchingElementIt = std::find(returnVector_PointElements.begin(), returnVector_PointElements.end(), returnVector_PointQuery.back());

				if (matchingElementIt != returnVector_PointElements.end())
				{
					returnVector_PointQuery.pop_back();
					returnVector_PointElements.erase(matchingElementIt);
				}
				else
				{
					testSuccessful = false;
					break;
				}
			}
		}
		else
		{
			testSuccessful = false;
		}
	}


	// Random ND and point query with rounded midpoint/boundaries.
	// Same principle as the previous case without rounding.

	for (size_t i = 0; i < queryTestsCount; ++i)
	{
		for (size_t j = 0; j < DIM; ++j)
		{
			// Generated numbers are rounded to the nearest integer.
			double randomMidpoint = std::round(uniformRealDistributionQueryMidpoint(randomNumberGenerator));
			double randomWidth = std::round(uniformRealDistributionQueryWidth(randomNumberGenerator));

			// If generated width is odd, then we make it even, so that
			// when it is divided by 2, it's still an integer.
			if (static_cast<int>(randomWidth) % 2 == 1)
			{
				randomWidth += 1.0;
			}

			queryMidpoint[j] = randomMidpoint;

			queryBoundaries[2*j] = randomMidpoint - randomWidth/2.0;
			queryBoundaries[2*j + 1] = randomMidpoint + randomWidth/2.0;
		}


		// ND query.
		std::vector<objT> returnVector_NDQuery = this->LocationQueryWrapper_SequenceGenerator(quadTree, queryBoundaries);
		testSuccessful &= quadTree.ValidateTree();

		std::vector<objT> returnVector_NDElements;

		std::array<double, 2*DIM> elementBoundaries;

		for (auto& batch : testElements)
		{
			for (auto& element : batch)
			{
				if (element.isPoint == true)
				{

					for (size_t j = 0; j < DIM; ++j)
					{
						elementBoundaries[2*j] = element.coordinates[j];
						elementBoundaries[2*j + 1] = element.coordinates[j];
					}
				}
				else
				{
					elementBoundaries = element.boundaries;
				}

				if (DoAxisAlignedNDVolumesIntersect<DIM>(queryBoundaries, elementBoundaries) == true)
				{
					bool elementOutsideBounds = false;

					for (size_t j = 0; j < DIM; ++j)
					{
						if (element.coordinates[j] < quadTree.GetBoundary(j + 1, true) || element.coordinates[j] > quadTree.GetBoundary(j + 1, false))
						{
							elementOutsideBounds = true;

							break;
						}
					}

					if (elementOutsideBounds == false)
					{
						returnVector_NDElements.push_back(element.object);
					}
				}
			}
		}

		if (returnVector_NDQuery.size() == returnVector_NDElements.size())
		{
			while (returnVector_NDQuery.empty() == false)
			{
				auto matchingElementIt = std::find(returnVector_NDElements.begin(), returnVector_NDElements.end(), returnVector_NDQuery.back());

				if (matchingElementIt != returnVector_NDElements.end())
				{
					returnVector_NDQuery.pop_back();
					returnVector_NDElements.erase(matchingElementIt);
				}
				else
				{
					testSuccessful = false;
					break;
				}
			}
		}
		else
		{
			testSuccessful = false;
		}


		for (size_t j = 0; j < DIM; ++j)
		{
			queryBoundaries[2*j] = queryMidpoint[j];
			queryBoundaries[2*j + 1] = queryMidpoint[j];
		}

		// Point query. Same process as ND query.
		std::vector<objT> returnVector_PointQuery = this->LocationQueryWrapper_SequenceGenerator(quadTree, queryMidpoint);
		testSuccessful &= quadTree.ValidateTree();

		std::vector<objT> returnVector_PointElements;

		for (auto& batch : testElements)
		{
			for (auto& element : batch)
			{
				if (element.isPoint == true)
				{
					for (size_t j = 0; j < DIM; ++j)
					{
						elementBoundaries[2*j] = element.coordinates[j];
						elementBoundaries[2*j + 1] = element.coordinates[j];
					}
				}
				else
				{
					elementBoundaries = element.boundaries;
				}

				if (DoAxisAlignedNDVolumesIntersect<DIM>(queryBoundaries, elementBoundaries) == true)
				{
					bool elementOutsideBounds = false;

					for (size_t j = 0; j < DIM; ++j)
					{
						if (element.coordinates[j] < quadTree.GetBoundary(j + 1, true) || element.coordinates[j] > quadTree.GetBoundary(j + 1, false))
						{
							elementOutsideBounds = true;

							break;
						}
					}

					if (elementOutsideBounds == false)
					{
						returnVector_PointElements.push_back(element.object);
					}
				}
			}
		}

		if (returnVector_PointQuery.size() == returnVector_PointElements.size())
		{
			while (returnVector_PointQuery.empty() == false)
			{
				auto matchingElementIt = std::find(returnVector_PointElements.begin(), returnVector_PointElements.end(), returnVector_PointQuery.back());

				if (matchingElementIt != returnVector_PointElements.end())
				{
					returnVector_PointQuery.pop_back();
					returnVector_PointElements.erase(matchingElementIt);
				}
				else
				{
					testSuccessful = false;
					break;
				}
			}
		}
		else
		{
			testSuccessful = false;
		}
	}

	if (testSuccessful == true)
	{
		std::cout << "Location queries test successful!" << std::endl;

		return true;
	}
	else
	{
		std::cout << "Location queries test not successful!" << std::endl;

		return false;
	}
}

bool QuadTreeTester::TestLocationQueriesEdgeCases() const
{
	bool testSuccessful = true;

	// 2^4 + 1, so that the inserted elements lie exatly on the boundaries of nodes.
	size_t elementsPerDimension = 17;

	// Distance between elements.
	double step = 0;

	// Id used when inserting elements.
	long long int nextId = 1;


	// Quad trees have to be square.
	// All queries are geometricaly constructed in such a way, that there always
	// will be exactly one intersecting element.


	// 2D, 1 capacity, only points inside.
	QuadTree<2, 1, int> quadTree2DCap1_Point(-400.0, 400.0, -400.0, 400.0);

	step = (quadTree2DCap1_Point.GetBoundary(1, false) - quadTree2DCap1_Point.GetBoundary(1, true))/static_cast<double>((elementsPerDimension - 1));

	// Insertion of point elements.
	for (size_t i = 0; i < elementsPerDimension; ++i)
	{
		for (size_t j = 0; j < elementsPerDimension; ++j)
		{
			double pointX = quadTree2DCap1_Point.GetBoundary(1, true) + static_cast<double>(i)*step;
			double pointY = quadTree2DCap1_Point.GetBoundary(2, true) + static_cast<double>(j)*step;

			testSuccessful &= quadTree2DCap1_Point.InsertPointElement(nextId, static_cast<int>(nextId), pointX, pointY);
			++nextId;

			testSuccessful &= quadTree2DCap1_Point.ValidateTree();
		}
	}

	// Point querying.
	for (size_t i = 0; i < elementsPerDimension; ++i)
	{
		for (size_t j = 0; j < elementsPerDimension; ++j)
		{
			double pointX = -400.0 + static_cast<double>(i)*step;
			double pointY = -400.0 + static_cast<double>(j)*step;

			// Point on point.
			testSuccessful &= quadTree2DCap1_Point.PointQuery(pointX, pointY).size() == 1;

			testSuccessful &= quadTree2DCap1_Point.ValidateTree();
		}
	}

	// ND querying.
	for (size_t i = 0; i < elementsPerDimension; ++i)
	{
		for (size_t j = 0; j < elementsPerDimension; ++j)
		{
			double pointX = -400.0 + static_cast<double>(i)*step;
			double pointY = -400.0 + static_cast<double>(j)*step;

			// Query window vertex touches point.
			testSuccessful &= quadTree2DCap1_Point.NDQuery(pointX - 1.0, pointX, pointY, pointY + 1.0).size() == 1;
			testSuccessful &= quadTree2DCap1_Point.NDQuery(pointX, pointX + 1.0, pointY, pointY + 1.0).size() == 1;
			testSuccessful &= quadTree2DCap1_Point.NDQuery(pointX - 1.0, pointX, pointY - 1.0, pointY).size() == 1;
			testSuccessful &= quadTree2DCap1_Point.NDQuery(pointX, pointX + 1.0, pointY - 1.0, pointY).size() == 1;

			// Query window edge touches point.
			testSuccessful &= quadTree2DCap1_Point.NDQuery(pointX - 1.0, pointX + 1.0, pointY, pointY + 1.0).size() == 1;
			testSuccessful &= quadTree2DCap1_Point.NDQuery(pointX - 1.0, pointX + 1.0, pointY - 1.0, pointY).size() == 1;
			testSuccessful &= quadTree2DCap1_Point.NDQuery(pointX - 1.0, pointX, pointY - 1.0, pointY + 1.0).size() == 1;
			testSuccessful &= quadTree2DCap1_Point.NDQuery(pointX, pointX + 1.0, pointY - 1.0, pointY + 1.0).size() == 1;

			// Query window surrounds point.
			testSuccessful &= quadTree2DCap1_Point.NDQuery(pointX - 1.0, pointX + 1.0, pointY - 1.0, pointY + 1.0).size() == 1;

			testSuccessful &= quadTree2DCap1_Point.ValidateTree();
		}
	}


	// 2D, 1 capacity, only ND elements inside.
	QuadTree<2, 1, int> quadTree2DCap1_ND(-400.0, 400.0, -400.0, 400.0);

	step = (quadTree2DCap1_ND.GetBoundary(1, false) - quadTree2DCap1_ND.GetBoundary(1, true))/static_cast<double>((elementsPerDimension - 1));

	// Insertion of ND elements.
	for (size_t i = 0; i < elementsPerDimension; ++i)
	{
		for (size_t j = 0; j < elementsPerDimension; ++j)
		{
			double pointX = quadTree2DCap1_ND.GetBoundary(1, true) + static_cast<double>(i)*step;
			double pointY = quadTree2DCap1_ND.GetBoundary(2, true) + static_cast<double>(j)*step;

			double boundaryMinX = pointX - 10.0;
			double boundaryMaxX = pointX + 10.0;
			double boundaryMinY = pointY - 10.0;
			double boundaryMaxY = pointY + 10.0;

			testSuccessful &= quadTree2DCap1_ND.InsertNDElement(nextId, static_cast<int>(nextId), boundaryMinX, boundaryMaxX, boundaryMinY, boundaryMaxY);
			++nextId;

			testSuccessful &= quadTree2DCap1_ND.ValidateTree();
		}
	}

	// Point querying.
	for (size_t i = 0; i < elementsPerDimension; ++i)
	{
		for (size_t j = 0; j < elementsPerDimension; ++j)
		{
			double pointX = -400.0 + static_cast<double>(i)*step;
			double pointY = -400.0 + static_cast<double>(j)*step;

			double boundaryMinX = pointX - 10.0;
			double boundaryMaxX = pointX + 10.0;
			double boundaryMinY = pointY - 10.0;
			double boundaryMaxY = pointY + 10.0;

			// Point query on ND element's corner.
			testSuccessful &= quadTree2DCap1_ND.PointQuery(boundaryMinX, boundaryMinY).size() == 1;
			testSuccessful &= quadTree2DCap1_ND.PointQuery(boundaryMinX, boundaryMaxY).size() == 1;
			testSuccessful &= quadTree2DCap1_ND.PointQuery(boundaryMaxX, boundaryMinY).size() == 1;
			testSuccessful &= quadTree2DCap1_ND.PointQuery(boundaryMaxX, boundaryMaxY).size() == 1;

			// Point query on ND element's edge
			testSuccessful &= quadTree2DCap1_ND.PointQuery(boundaryMinX, pointY).size() == 1;
			testSuccessful &= quadTree2DCap1_ND.PointQuery(pointX, boundaryMinY).size() == 1;
			testSuccessful &= quadTree2DCap1_ND.PointQuery(boundaryMaxX, pointY).size() == 1;
			testSuccessful &= quadTree2DCap1_ND.PointQuery(pointX, boundaryMaxY).size() == 1;

			// Point query on ND element's center.
			testSuccessful &= quadTree2DCap1_ND.PointQuery(pointX, pointY).size() == 1;

			testSuccessful &= quadTree2DCap1_ND.ValidateTree();
		}
	}

	// ND querying.
	for (size_t i = 0; i < elementsPerDimension; ++i)
	{
		for (size_t j = 0; j < elementsPerDimension; ++j)
		{
			double pointX = -400.0 + static_cast<double>(i)*step;
			double pointY = -400.0 + static_cast<double>(j)*step;

			double boundaryMinX = pointX - 10.0;
			double boundaryMaxX = pointX + 10.0;
			double boundaryMinY = pointX - 10.0;
			double boundaryMaxY = pointX + 10.0;

			// Query window vertex touches ND element's corner.
			testSuccessful &= quadTree2DCap1_ND.NDQuery(boundaryMinX - 1.0, boundaryMinX, boundaryMaxY, boundaryMaxY + 1.0).size() == 1;
			testSuccessful &= quadTree2DCap1_ND.NDQuery(boundaryMinX, boundaryMinX + 1.0, boundaryMaxY, boundaryMaxY + 1.0).size() == 1;
			testSuccessful &= quadTree2DCap1_ND.NDQuery(boundaryMinX - 1.0, boundaryMinX, boundaryMaxY - 1.0, boundaryMaxY).size() == 1;
			testSuccessful &= quadTree2DCap1_ND.NDQuery(boundaryMinX, boundaryMinX + 1.0, boundaryMaxY - 1.0, boundaryMaxY).size() == 1;

			// Query window edge touches ND element's edge.
			testSuccessful &= quadTree2DCap1_ND.NDQuery(pointX - 1.0, pointX + 1.0, boundaryMaxY, boundaryMaxY + 1.0).size() == 1;
			testSuccessful &= quadTree2DCap1_ND.NDQuery(pointX - 1.0, pointX + 1.0, boundaryMinY - 1.0, boundaryMinY).size() == 1;
			testSuccessful &= quadTree2DCap1_ND.NDQuery(boundaryMinX - 1.0, boundaryMinX, pointY - 1.0, pointY + 1.0).size() == 1;
			testSuccessful &= quadTree2DCap1_ND.NDQuery(boundaryMaxY, boundaryMaxY + 1.0, pointY - 1.0, pointY + 1.0).size() == 1;

			// Query window is inside ND element.
			testSuccessful &= quadTree2DCap1_ND.NDQuery(pointX - 1.0, pointX + 1.0, pointY - 1.0, pointY + 1.0).size() == 1;

			// Query window surrounds ND element.
			testSuccessful &= quadTree2DCap1_ND.NDQuery(boundaryMinX - 1.0, boundaryMaxX + 1.0, boundaryMinY - 1.0, boundaryMaxY + 1.0).size() == 1;

			testSuccessful &= quadTree2DCap1_ND.ValidateTree();
		}
	}

	if (testSuccessful == true)
	{
		std::cout << "Test of location query's edge cases successful." << std::endl;

		return true;
	}
	else
	{
		std::cout << "Test of location query's edge cases not successful." << std::endl;

		return false;
	}
}

template<size_t... indexes, size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT, size_t N>
std::optional<std::pair<coordT, objT>> QuadTreeTester::DistanceQueryClosestWrapper_Caller([[maybe_unused]] std::index_sequence<indexes...> indexSequence, QuadTree<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>& quadTree, coordT step, std::array<coordT, N>& parameterArray) const
{
	return quadTree.QueryClosestElement(step, parameterArray[indexes]...);
}

template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT, size_t N>
std::optional<std::pair<coordT, objT>> QuadTreeTester::DistanceQueryClosestWrapper_SequenceGenerator(QuadTree<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>& quadTree, coordT step, std::array<coordT, N>& parameterArray) const
{
	return this->DistanceQueryClosestWrapper_Caller(std::make_index_sequence<N>(), quadTree, step, parameterArray);
}

template<size_t... indexes, size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT, size_t N>
std::vector<std::pair<coordT, objT>> QuadTreeTester::DistanceQueryNClosestWrapper_Caller([[maybe_unused]] std::index_sequence<indexes...> indexSequence, QuadTree<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>& quadTree, size_t count, coordT step, std::array<coordT, N>& parameterArray) const
{
	return quadTree.QueryNClosestElements(count, step, parameterArray[indexes]...);
}

template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT, size_t N>
std::vector<std::pair<coordT, objT>> QuadTreeTester::DistanceQueryNClosestWrapper_SequenceGenerator(QuadTree<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>& quadTree, size_t count, coordT step, std::array<coordT, N>& parameterArray) const
{
	return this->DistanceQueryNClosestWrapper_Caller(std::make_index_sequence<N>(), quadTree, count, step, parameterArray);
}

template<size_t... indexes, size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT, size_t N>
std::vector<std::pair<coordT, objT>> QuadTreeTester::DistanceQueryRadiusWrapper_Caller([[maybe_unused]] std::index_sequence<indexes...> indexSequence, QuadTree<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>& quadTree, coordT radius, std::array<coordT, N>& parameterArray) const
{
	return quadTree.QueryElementsWithinRadius(radius, parameterArray[indexes]...);
}

template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT, size_t N>
std::vector<std::pair<coordT, objT>> QuadTreeTester::DistanceQueryRadiusWrapper_SequenceGenerator(QuadTree<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>& quadTree, coordT radius, std::array<coordT, N>& parameterArray) const
{
	return this->DistanceQueryRadiusWrapper_Caller(std::make_index_sequence<N>(), quadTree, radius, parameterArray);
}

template<size_t DIM, size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
bool QuadTreeTester::TestDistanceQueries(QuadTree<DIM, MAX_CAP, objT, MAX_DEPTH, coordT>& quadTree, std::vector<std::vector<TestElement<DIM, objT, coordT>>>& testElements) const
{
	// Square quad tree is expected.

	bool testSuccessful = true;

	// Returns sorted testElements in a form of a vector of pairs of squared distance and object.
	auto findClosestElements = [&testElements, &quadTree](const std::array<coordT, DIM>& queryPoint) -> std::vector<std::pair<coordT, objT>>
	{
		std::vector<std::pair<coordT, objT>> returnVector;

		for (auto& batch : testElements)
		{
			for (auto& element : batch)
			{
				bool elementOutsideBounds = false;

				for (size_t i = 0; i < DIM; ++i)
				{
					if (element.coordinates[i] < quadTree.GetBoundary(i + 1, true) || element.coordinates[i] > quadTree.GetBoundary(i + 1, false))
					{
						elementOutsideBounds = true;

						break;
					}
				}

				// Only elements inside bounds are considered.
				if (elementOutsideBounds == false)
				{
					returnVector.emplace_back(CalculateSquaredEuclidianDistanceBetweenPoints(queryPoint, element.coordinates), element.object);
				}
			}
		}

		// Elements are sorted by their squared distance.
		std::sort(returnVector.begin(), returnVector.end(), [](auto& firstPair, auto& secondPair){return firstPair.first < secondPair.first;});

		return returnVector;
	};

	// Veryfies that the the result from quad tree and test elements is the same.
	auto verifyResult = [](const std::vector<std::pair<coordT, objT>>& quadTreeVector, coordT radius, const std::vector<std::pair<coordT, objT>>& testElementsVector)
	{
		coordT epsilon = 0.00001;

		bool vectorMatches = true;

		// Problem is that when there are multiple elements at the same coordinates, the quadtree returns them at random.
		// So instead of comparing elements one by one, a local search will be performed around the current index in the test elements vector.
		for (size_t i = 0; i < quadTreeVector.size(); ++i)
		{
			bool elementMatches = false;

			size_t j = i;

			// Current index and forward direction.
			while (j < testElementsVector.size())
			{
				if (quadTreeVector[i].first <= testElementsVector[j].first + epsilon && quadTreeVector[i].first >= testElementsVector[j].first - epsilon)
				{
					if (quadTreeVector[i].second == testElementsVector[j].second)
					{
						elementMatches = true;
						break;
					}
				}
				else
				{
					break;
				}

				++j;
			}

			int k = static_cast<int>(i) - 1;

			// Backward direction.
			while (elementMatches == false && k >= 0)
			{
				if (quadTreeVector[i].first <= testElementsVector[static_cast<size_t>(k)].first + epsilon && quadTreeVector[i].first >= testElementsVector[static_cast<size_t>(k)].first - epsilon)
				{
					if (quadTreeVector[i].second == testElementsVector[static_cast<size_t>(k)].second)
					{
						elementMatches = true;
						break;
					}
				}
				else
				{
					break;
				}

				--k;
			}

			if (elementMatches == false)
			{
				vectorMatches = false;
			}
		}

		// There are always elements in the tree, so the only possiblity how can the quadTreeVector be empty
		// is when the radius query contained no elements. In that chase the distance to the closest element
		// has to be bigger than the given radius - error otherwise.
		if (vectorMatches == true && quadTreeVector.empty() == true && !((radius - epsilon)*(radius - epsilon) < testElementsVector.front().first))
		{
			vectorMatches = false;
		}

		// Separate verification of elements being in the descending order.
		for (size_t i = 0; i < quadTreeVector.size(); ++i)
		{
			if	(i != quadTreeVector.size() - 1)
			{
				if (quadTreeVector[i].first > quadTreeVector[i + 1].first)
				{
					vectorMatches = false;
					break;
				}
			}
		}

		return vectorMatches;
	};

	// Random generators.
	unsigned int seed = 1;
	std::mt19937_64 randomNumberGenerator(seed);
	// Query midpoint generator.
	std::uniform_real_distribution<double> uniformRealDistributionQueryMidpoint(quadTree.GetBoundary(1, true) - 50.0, std::nextafter(quadTree.GetBoundary(1, false) + 50.0, std::numeric_limits<double>::max()));
	// Count generator.
	std::uniform_int_distribution<size_t> uniformIntDistributionCount(10, 100);
	// Radius generator.
	std::uniform_real_distribution<double> uniformRealDistributionRadius(30.0, 100.0);


	// Distance queries tests.

	size_t queryTestsCount = 1000;

	// Query parameters.
	std::array<double, DIM> queryMidpoint = {};
	size_t count = 0;
	double radius = 0.0;

	std::vector<std::pair<coordT, objT>> quadTreeVector;
	std::vector<std::pair<coordT, objT>> testElementsVector;

	for (size_t i = 0; i < queryTestsCount; ++i)
	{
		// Random query parameters are generated.

		for (size_t j = 0; j < DIM; ++j)
		{
			queryMidpoint[j] = uniformRealDistributionQueryMidpoint(randomNumberGenerator);
		}

		count = uniformIntDistributionCount(randomNumberGenerator);
		radius = uniformRealDistributionRadius(randomNumberGenerator);

		// Control result based on testElements is calculated.
		testElementsVector = findClosestElements(queryMidpoint);


		// Closest element test.

		std::optional<std::pair<coordT, objT>> returnPair = DistanceQueryClosestWrapper_SequenceGenerator(quadTree, 100.0, queryMidpoint);
		quadTreeVector.clear();
		if (returnPair.has_value() == true)
		{
			quadTreeVector.push_back(returnPair.value());
		}
		testSuccessful &= quadTree.ValidateTree();
		testSuccessful &= verifyResult(quadTreeVector, radius, testElementsVector);

		if (testSuccessful == false)
		{
			break;
		}


		// N closest elements test.

		quadTreeVector = DistanceQueryNClosestWrapper_SequenceGenerator(quadTree, count, 100.0, queryMidpoint);
		testSuccessful &= quadTree.ValidateTree();
		testSuccessful &= verifyResult(quadTreeVector, radius, testElementsVector);

		if (testSuccessful == false)
		{
			break;
		}


		// Elements in radius test.

		quadTreeVector = DistanceQueryRadiusWrapper_SequenceGenerator(quadTree, radius, queryMidpoint);
		testSuccessful &= quadTree.ValidateTree();
		testSuccessful &= verifyResult(quadTreeVector, radius, testElementsVector);

		if (testSuccessful == false)
		{
			break;
		}
	}

	if (testSuccessful == true)
	{
		std::cout << "Distance queries test successful!" << std::endl;

		return true;
	}
	else
	{
		std::cout << "Distance queries test not successful!" << std::endl;

		return false;
	}
}

bool QuadTreeTester::TestRadiusQueryEdgeCases() const
{
	bool testSuccessful = true;

	// All queries are geometricaly constructed in such a way, that there always
	// will be exactly one element returned.

	// 2D, 1 capacity, only points inside.
	QuadTree<2, 1, int> quadTree2DCap1_Point(-400.0, 400.0, -400.0, 400.0);

	quadTree2DCap1_Point.InsertPointElement(1, 1, -400.0, -400.0);
	quadTree2DCap1_Point.InsertPointElement(2, 2, -400.0, 400.0);
	quadTree2DCap1_Point.InsertPointElement(3, 3, 400.0, -400.0);
	quadTree2DCap1_Point.InsertPointElement(4, 4, 400.0, 400.0);

	quadTree2DCap1_Point.InsertPointElement(5, 5, -400.0, 0.0);
	quadTree2DCap1_Point.InsertPointElement(6, 6, 400.0, 0.0);
	quadTree2DCap1_Point.InsertPointElement(7, 7, 0.0, -400.0);
	quadTree2DCap1_Point.InsertPointElement(8, 8, 0.0, 400.0);


	// Edge touches.
	testSuccessful &= quadTree2DCap1_Point.QueryElementsWithinRadius(100.0, -500.0, 0.0).size() == 1;
	testSuccessful &= quadTree2DCap1_Point.ValidateTree();

	testSuccessful &= quadTree2DCap1_Point.QueryElementsWithinRadius(100.0, 500.0, 0.0).size() == 1;
	testSuccessful &= quadTree2DCap1_Point.ValidateTree();

	testSuccessful &= quadTree2DCap1_Point.QueryElementsWithinRadius(100.0, 0.0, 500.0).size() == 1;
	testSuccessful &= quadTree2DCap1_Point.ValidateTree();

	testSuccessful &= quadTree2DCap1_Point.QueryElementsWithinRadius(100.0, 0.0, -500.0).size() == 1;
	testSuccessful &= quadTree2DCap1_Point.ValidateTree();


	// Top left, 45 degree touch.
	testSuccessful &= quadTree2DCap1_Point.QueryElementsWithinRadius(100.0, -400.0 - 100*(1.0/std::sqrt(2.0)), 400.0 + 100*(1.0/std::sqrt(2.0))).size() == 1;
	testSuccessful &= quadTree2DCap1_Point.ValidateTree();

	// Top right, 45 degree touch.
	testSuccessful &= quadTree2DCap1_Point.QueryElementsWithinRadius(100.0, 400.0 + 100*(1.0/std::sqrt(2.0)), 400.0 + 100*(1.0/std::sqrt(2.0))).size() == 1;
	testSuccessful &= quadTree2DCap1_Point.ValidateTree();


	// Bottom left, 5/5, 4/5, 3/5 triangle touch.
	testSuccessful &= quadTree2DCap1_Point.QueryElementsWithinRadius(100.0, -400.0 - 100*(4.0/5.0), -400.0 - 100*(3.0/5.0)).size() == 1;
	testSuccessful &= quadTree2DCap1_Point.ValidateTree();

	// Top right, 5/5, 4/5, 3/5 triangle touch.
	testSuccessful &= quadTree2DCap1_Point.QueryElementsWithinRadius(100.0, 400.0 + 100*(4.0/5.0), -400.0 - 100*(3.0/5.0)).size() == 1;
	testSuccessful &= quadTree2DCap1_Point.ValidateTree();


	if (testSuccessful == true)
	{
		std::cout << "Test of radius query's edge cases successful." << std::endl;

		return true;
	}
	else
	{
		std::cout << "Test of radius query's edge cases not successful." << std::endl;

		return false;
	}
}

template<size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
bool QuadTreeTester::TestRayQuery_2D(QuadTree<2, MAX_CAP, objT, MAX_DEPTH, coordT>& quadTree, std::vector<std::vector<TestElement<2, objT, coordT>>>& testElements) const
{
	// Square quad tree is expected.

	bool testSuccessful = true;

	// Finds elements from testElements vector that are intersecting given ray.
	auto findElementsIntersectingRay = [&testElements, &quadTree](const std::array<coordT, 2>& rayStartPoint, const std::array<coordT, 2>& rayEndPoint) -> std::vector<objT>
	{
		double epsilon = 0.00000001;

		std::vector<objT> returnVector;

		for (auto& batch : testElements)
		{
			for (auto& element : batch)
			{
				bool elementOutsideBounds = false;

				for (size_t i = 0; i < 2; ++i)
				{
					if (element.coordinates[i] < quadTree.GetBoundary(i + 1, true) || element.coordinates[i] > quadTree.GetBoundary(i + 1, false))
					{
						elementOutsideBounds = true;

						break;
					}
				}

				// Only elements inside bounds are considered.
				if (elementOutsideBounds == false)
				{
					if (element.isPoint == true)
					{
						if (DoesLineIntersectPoint_2D(rayStartPoint[0], rayStartPoint[1], rayEndPoint[0], rayEndPoint[1], element.coordinates[0], element.coordinates[1], epsilon) == true)
						{
							returnVector.push_back(element.object);
						}
					}
					else
					{
						if (DoesLineIntersectRectangle_2D(rayStartPoint[0], rayStartPoint[1], rayEndPoint[0], rayEndPoint[1], element.boundaries[0], element.boundaries[1], element.boundaries[2], element.boundaries[3], epsilon) == true)
						{
							returnVector.push_back(element.object);
						}
					}
				}
			}
		}

		return returnVector;
	};

	// Random generators.
	unsigned int seed = 1;
	std::mt19937_64 randomNumberGenerator(seed);
	// Ray start point generator.
	std::uniform_real_distribution<double> uniformRealDistributionRayStartPoint(quadTree.GetBoundary(1, true) - 50.0, std::nextafter(quadTree.GetBoundary(1, false) + 50.0, std::numeric_limits<double>::max()));
	// Ray direction generator.
	std::uniform_real_distribution<double> uniformRealDistributionRayDirection(-1.0, 1.0);


	size_t queryTestsCount = 10000;

	// Query parameters.
	std::array<double, 2> rayStartPoint = {};
	std::array<double, 2> rayEndPoint = {};

	std::vector<objT> quadTreeVector;
	std::vector<objT> testElementsVector;

	for (size_t i = 0; i < queryTestsCount; ++i)
	{
		// Random ray start point and end point are generated.
		rayStartPoint[0] = uniformRealDistributionRayStartPoint(randomNumberGenerator);
		rayStartPoint[1] = uniformRealDistributionRayStartPoint(randomNumberGenerator);
		rayEndPoint[0] = rayStartPoint[0] + uniformRealDistributionRayDirection(randomNumberGenerator);
		rayEndPoint[1] = rayStartPoint[1] + uniformRealDistributionRayDirection(randomNumberGenerator);

		// Control result based on testElements is calculated.
		testElementsVector = findElementsIntersectingRay(rayStartPoint, rayEndPoint);

		// Quad tree ray test.
		quadTreeVector = quadTree.RayQuery(rayStartPoint[0], rayStartPoint[1], rayEndPoint[0], rayEndPoint[1]);

		testSuccessful &= quadTree.ValidateTree();
		testSuccessful &= testElementsVector.size() == quadTreeVector.size();

		if (testSuccessful == false)
		{
			break;
		}
	}

	if (testSuccessful == true)
	{
		std::cout << "2D ray query test successful!" << std::endl;

		return true;
	}
	else
	{
		std::cout << "2D ray query test not successful!" << std::endl;

		return false;
	}
}

template<size_t MAX_CAP, typename objT, int MAX_DEPTH, typename coordT>
bool QuadTreeTester::TestRayQuery_3D(QuadTree<3, MAX_CAP, objT, MAX_DEPTH, coordT>& quadTree, std::vector<std::vector<TestElement<3, objT, coordT>>>& testElements) const
{
	// Square quad tree is expected.

	bool testSuccessful = true;

	// Finds elements from testElements vector that are intersecting given ray.
	auto findElementsIntersectingRay = [&testElements, &quadTree](const std::array<coordT, 3>& rayStartPoint, const std::array<coordT, 3>& rayEndPoint) -> std::vector<objT>
	{
		double epsilon = 0.00000001;

		std::vector<objT> returnVector;

		for (auto& batch : testElements)
		{
			for (auto& element : batch)
			{
				bool elementOutsideBounds = false;

				for (size_t i = 0; i < 3; ++i)
				{
					if (element.coordinates[i] < quadTree.GetBoundary(i + 1, true) || element.coordinates[i] > quadTree.GetBoundary(i + 1, false))
					{
						elementOutsideBounds = true;

						break;
					}
				}

				// Only elements inside bounds are considered.
				if (elementOutsideBounds == false)
				{
					if (element.isPoint == true)
					{
						if (DoesLineIntersectPoint_3D(rayStartPoint[0], rayStartPoint[1], rayStartPoint[2], rayEndPoint[0], rayEndPoint[1], rayEndPoint[2], element.coordinates[0], element.coordinates[1], element.coordinates[2], epsilon) == true)
						{
							returnVector.push_back(element.object);
						}
					}
					else
					{
						if (DoesLineIntersectRectangularCuboid_3D(rayStartPoint[0], rayStartPoint[1], rayStartPoint[2], rayEndPoint[0], rayEndPoint[1], rayEndPoint[2], element.boundaries[0], element.boundaries[1], element.boundaries[2], element.boundaries[3], element.boundaries[4], element.boundaries[5], epsilon) == true)
						{
							returnVector.push_back(element.object);
						}
					}
				}
			}
		}

		return returnVector;
	};

	// Random generators.
	unsigned int seed = 1;
	std::mt19937_64 randomNumberGenerator(seed);
	// Ray start point generator.
	std::uniform_real_distribution<double> uniformRealDistributionRayStartPoint(quadTree.GetBoundary(1, true) - 50.0, std::nextafter(quadTree.GetBoundary(1, false) + 50.0, std::numeric_limits<double>::max()));
	// Ray direction generator.
	std::uniform_real_distribution<double> uniformRealDistributionRayDirection(-1.0, 1.0);


	size_t queryTestsCount = 10000;

	// Query parameters.
	std::array<double, 3> rayStartPoint = {};
	std::array<double, 3> rayEndPoint = {};

	std::vector<objT> quadTreeVector;
	std::vector<objT> testElementsVector;

	for (size_t i = 0; i < queryTestsCount; ++i)
	{
		// Random ray start point and end point are generated.
		rayStartPoint[0] = uniformRealDistributionRayStartPoint(randomNumberGenerator);
		rayStartPoint[1] = uniformRealDistributionRayStartPoint(randomNumberGenerator);
		rayStartPoint[2] = uniformRealDistributionRayStartPoint(randomNumberGenerator);
		rayEndPoint[0] = rayStartPoint[0] + uniformRealDistributionRayDirection(randomNumberGenerator);
		rayEndPoint[1] = rayStartPoint[1] + uniformRealDistributionRayDirection(randomNumberGenerator);
		rayEndPoint[2] = rayStartPoint[2] + uniformRealDistributionRayDirection(randomNumberGenerator);

		// Control result based on testElements is calculated.
		testElementsVector = findElementsIntersectingRay(rayStartPoint, rayEndPoint);

		// Quad tree ray test.
		quadTreeVector = quadTree.RayQuery(rayStartPoint[0], rayStartPoint[1], rayStartPoint[2], rayEndPoint[0], rayEndPoint[1], rayEndPoint[2]);

		testSuccessful &= quadTree.ValidateTree();
		testSuccessful &= testElementsVector.size() == quadTreeVector.size();

		if (testSuccessful == false)
		{
			break;
		}
	}

	if (testSuccessful == true)
	{
		std::cout << "3D ray query test successful!" << std::endl;

		return true;
	}
	else
	{
		std::cout << "3D ray query test not successful!" << std::endl;

		return false;
	}
}

bool QuadTreeTester::TestRayQueryEdgeCases_2D() const
{
	bool testSuccessful = true;

	// All queries are geometricaly constructed in such a way, that there always
	// will be exactly one element returned.

	// 2D, 1 capacity
	QuadTree<2, 1, int> quadTree2DCap1(-400.0, 400.0, -400.0, 400.0);


	// Point element tests.

	quadTree2DCap1.InsertPointElement(1, 1, 0.0, 0.0);

	testSuccessful &= quadTree2DCap1.RayQuery(-100.0, 0.0, 100.0, 0.0).size() == 1;
	testSuccessful &= quadTree2DCap1.ValidateTree();
	testSuccessful &= quadTree2DCap1.RayQuery(100.0, 0.0, -100.0, 0.0).size() == 1;
	testSuccessful &= quadTree2DCap1.ValidateTree();

	testSuccessful &= quadTree2DCap1.RayQuery(0.0, 100.0, 0.0, -100.0).size() == 1;
	testSuccessful &= quadTree2DCap1.ValidateTree();
	testSuccessful &= quadTree2DCap1.RayQuery(0.0, -100.0, 0.0, 100.0).size() == 1;
	testSuccessful &= quadTree2DCap1.ValidateTree();

	testSuccessful &= quadTree2DCap1.RayQuery(-100.0, 100.0, 100.0, -100.0).size() == 1;
	testSuccessful &= quadTree2DCap1.ValidateTree();
	testSuccessful &= quadTree2DCap1.RayQuery(100.0, -100.0, -100.0, 100.0).size() == 1;
	testSuccessful &= quadTree2DCap1.ValidateTree();

	testSuccessful &= quadTree2DCap1.RayQuery(100.0, 100.0, -100.0, -100.0).size() == 1;
	testSuccessful &= quadTree2DCap1.ValidateTree();
	testSuccessful &= quadTree2DCap1.RayQuery(-100.0, -100.0, 100.0, 100.0).size() == 1;
	testSuccessful &= quadTree2DCap1.ValidateTree();

	quadTree2DCap1.DeletePointElement(1, 0.0, 0.0);


	// ND element tests.

	quadTree2DCap1.InsertNDElement(2, 2, -100.0, 100.0, -100.0, 100.0);

	// Line touch.

	testSuccessful &= quadTree2DCap1.RayQuery(-100.0, -200.0, -100.0, 200.0).size() == 1;
	testSuccessful &= quadTree2DCap1.ValidateTree();
	testSuccessful &= quadTree2DCap1.RayQuery(-100.0, 200.0, -100.0, -200.0).size() == 1;
	testSuccessful &= quadTree2DCap1.ValidateTree();

	testSuccessful &= quadTree2DCap1.RayQuery(100.0, -200.0, 100.0, 200.0).size() == 1;
	testSuccessful &= quadTree2DCap1.ValidateTree();
	testSuccessful &= quadTree2DCap1.RayQuery(100.0, 200.0, 100.0, -200.0).size() == 1;
	testSuccessful &= quadTree2DCap1.ValidateTree();

	testSuccessful &= quadTree2DCap1.RayQuery(-200.0, 100.0, 200.0, 100.0).size() == 1;
	testSuccessful &= quadTree2DCap1.ValidateTree();
	testSuccessful &= quadTree2DCap1.RayQuery(200.0, 100.0, -200.0, 100.0).size() == 1;
	testSuccessful &= quadTree2DCap1.ValidateTree();

	testSuccessful &= quadTree2DCap1.RayQuery(-200.0, -100.0, 200.0, -100.0).size() == 1;
	testSuccessful &= quadTree2DCap1.ValidateTree();
	testSuccessful &= quadTree2DCap1.RayQuery(200.0, -100.0, -200.0, -100.0).size() == 1;
	testSuccessful &= quadTree2DCap1.ValidateTree();

	// Vertex touch.

	testSuccessful &= quadTree2DCap1.RayQuery(-200.0, 0.0, 0.0, 200.0).size() == 1;
	testSuccessful &= quadTree2DCap1.ValidateTree();
	testSuccessful &= quadTree2DCap1.RayQuery(0.0, 200.0, 200.0, 0.0).size() == 1;
	testSuccessful &= quadTree2DCap1.ValidateTree();

	testSuccessful &= quadTree2DCap1.RayQuery(0.0, 200.0, 200.0, 0.0).size() == 1;
	testSuccessful &= quadTree2DCap1.ValidateTree();
	testSuccessful &= quadTree2DCap1.RayQuery(200.0, 0.0, 0.0, 200.0).size() == 1;
	testSuccessful &= quadTree2DCap1.ValidateTree();

	testSuccessful &= quadTree2DCap1.RayQuery(200.0, 0.0, 0.0, -200.0).size() == 1;
	testSuccessful &= quadTree2DCap1.ValidateTree();
	testSuccessful &= quadTree2DCap1.RayQuery(0.0, -200.0, 200.0, 0.0).size() == 1;
	testSuccessful &= quadTree2DCap1.ValidateTree();

	testSuccessful &= quadTree2DCap1.RayQuery(0.0, -200.0, -200.0, 0.0).size() == 1;
	testSuccessful &= quadTree2DCap1.ValidateTree();
	testSuccessful &= quadTree2DCap1.RayQuery(-200.0, 0.0, 0.0, -200.0).size() == 1;
	testSuccessful &= quadTree2DCap1.ValidateTree();

	if (testSuccessful == true)
	{
		std::cout << "Test of 2D ray query's edge cases successful." << std::endl;

		return true;
	}
	else
	{
		std::cout << "Test of 2D ray query's edge cases not successful." << std::endl;

		return false;
	}
}

bool QuadTreeTester::TestRayQueryEdgeCases_3D() const
{
	bool testSuccessful = true;

	// All queries are geometricaly constructed in such a way, that there always
	// will be exactly one element returned.

	// 3D, 1 capacity
	QuadTree<3, 1, int> quadTree3DCap1(-400.0, 400.0, -400.0, 400.0, -400.0, 400.0);


	// Point element tests.

	quadTree3DCap1.InsertPointElement(1, 1, 0.0, 0.0, 0.0);

	// 2D tests still have to work (Z coordinate is 0.0)

	testSuccessful &= quadTree3DCap1.RayQuery(-100.0, 0.0, 0.0, 100.0, 0.0, 0.0).size() == 1;
	testSuccessful &= quadTree3DCap1.ValidateTree();
	testSuccessful &= quadTree3DCap1.RayQuery(100.0, 0.0, 0.0, -100.0, 0.0, 0.0).size() == 1;
	testSuccessful &= quadTree3DCap1.ValidateTree();

	testSuccessful &= quadTree3DCap1.RayQuery(0.0, 100.0, 0.0, 0.0, -100.0, 0.0).size() == 1;
	testSuccessful &= quadTree3DCap1.ValidateTree();
	testSuccessful &= quadTree3DCap1.RayQuery(0.0, -100.0, 0.0, 0.0, 100.0, 0.0).size() == 1;
	testSuccessful &= quadTree3DCap1.ValidateTree();

	testSuccessful &= quadTree3DCap1.RayQuery(-100.0, 100.0, 0.0, 100.0, -100.0, 0.0).size() == 1;
	testSuccessful &= quadTree3DCap1.ValidateTree();
	testSuccessful &= quadTree3DCap1.RayQuery(100.0, -100.0, 0.0, -100.0, 100.0, 0.0).size() == 1;
	testSuccessful &= quadTree3DCap1.ValidateTree();

	testSuccessful &= quadTree3DCap1.RayQuery(100.0, 100.0, 0.0, -100.0, -100.0, 0.0).size() == 1;
	testSuccessful &= quadTree3DCap1.ValidateTree();
	testSuccessful &= quadTree3DCap1.RayQuery(-100.0, -100.0, 0.0, 100.0, 100.0, 0.0).size() == 1;
	testSuccessful &= quadTree3DCap1.ValidateTree();

	// 3D tests (selected combinations).

	testSuccessful &= quadTree3DCap1.RayQuery(0.0, 0.0, -100.0, 0.0, 0.0, 100.0).size() == 1;
	testSuccessful &= quadTree3DCap1.ValidateTree();
	testSuccessful &= quadTree3DCap1.RayQuery(0.0, 0.0, 100.0, 0.0, 0.0, -100.0).size() == 1;
	testSuccessful &= quadTree3DCap1.ValidateTree();

	testSuccessful &= quadTree3DCap1.RayQuery(0.0, 100.0, 100.0, 0.0, -100.0, -100.0).size() == 1;
	testSuccessful &= quadTree3DCap1.ValidateTree();
	testSuccessful &= quadTree3DCap1.RayQuery(0.0, -100.0, -100.0, 0.0, 100.0, 100.0).size() == 1;
	testSuccessful &= quadTree3DCap1.ValidateTree();

	testSuccessful &= quadTree3DCap1.RayQuery(100.0, 100.0, 100.0, -100.0, -100.0, -100.0).size() == 1;
	testSuccessful &= quadTree3DCap1.ValidateTree();
	testSuccessful &= quadTree3DCap1.RayQuery(-100.0, -100.0, -100.0, 100.0, 100.0, 100.0).size() == 1;
	testSuccessful &= quadTree3DCap1.ValidateTree();

	quadTree3DCap1.DeletePointElement(1, 0.0, 0.0, 0.0);


	// ND element tests.

	quadTree3DCap1.InsertNDElement(2, 2, -100.0, 100.0, -100.0, 100.0, -100.0, 100.0);

	// 2D tests still have to work (Z coordinate is 0.0)

	// Line touch.

	testSuccessful &= quadTree3DCap1.RayQuery(-100.0, -200.0, 0.0, -100.0, 200.0, 0.0).size() == 1;
	testSuccessful &= quadTree3DCap1.ValidateTree();
	testSuccessful &= quadTree3DCap1.RayQuery(-100.0, 200.0, 0.0, -100.0, -200.0, 0.0).size() == 1;
	testSuccessful &= quadTree3DCap1.ValidateTree();

	testSuccessful &= quadTree3DCap1.RayQuery(100.0, -200.0, 0.0, 100.0, 200.0, 0.0).size() == 1;
	testSuccessful &= quadTree3DCap1.ValidateTree();
	testSuccessful &= quadTree3DCap1.RayQuery(100.0, 200.0, 0.0, 100.0, -200.0, 0.0).size() == 1;
	testSuccessful &= quadTree3DCap1.ValidateTree();

	// Vertex touch.

	testSuccessful &= quadTree3DCap1.RayQuery(-200.0, 0.0, 0.0, 0.0, 200.0, 0.0).size() == 1;
	testSuccessful &= quadTree3DCap1.ValidateTree();
	testSuccessful &= quadTree3DCap1.RayQuery(0.0, 200.0, 0.0, 200.0, 0.0, 0.0).size() == 1;
	testSuccessful &= quadTree3DCap1.ValidateTree();

	// 3D tests (selected combinations).

	// Line touch.

	testSuccessful &= quadTree3DCap1.RayQuery(-100.0, 100.0, 200.0, -100.0, 100.0, -200.0).size() == 1;
	testSuccessful &= quadTree3DCap1.ValidateTree();
	testSuccessful &= quadTree3DCap1.RayQuery(-100.0, 100.0, -200.0, -100.0, 100.0, 200.0).size() == 1;
	testSuccessful &= quadTree3DCap1.ValidateTree();

	testSuccessful &= quadTree3DCap1.RayQuery(100.0, -100.0, 200.0, 100.0, -100.0, -200.0).size() == 1;
	testSuccessful &= quadTree3DCap1.ValidateTree();
	testSuccessful &= quadTree3DCap1.RayQuery(100.0, -100.0, -200.0, 100.0, -100.0, 200.0).size() == 1;
	testSuccessful &= quadTree3DCap1.ValidateTree();

	testSuccessful &= quadTree3DCap1.RayQuery(0.0, 0.0, -200.0, -200.0, 0.0, 0.0).size() == 1;
	testSuccessful &= quadTree3DCap1.ValidateTree();
	testSuccessful &= quadTree3DCap1.RayQuery(-200.0, 0.0, 0.0, 0.0, 0.0, -200.0).size() == 1;
	testSuccessful &= quadTree3DCap1.ValidateTree();

	testSuccessful &= quadTree3DCap1.RayQuery(0.0, 0.0, -200.0, 200.0, 100.0, 0.0).size() == 1;
	testSuccessful &= quadTree3DCap1.ValidateTree();
	testSuccessful &= quadTree3DCap1.RayQuery(200.0, 100.0, 0.0, 0.0, 0.0, -200.0).size() == 1;
	testSuccessful &= quadTree3DCap1.ValidateTree();

	// Vertex touch.

	testSuccessful &= quadTree3DCap1.RayQuery(0.0, 100.0, -200.0, -200.0, 100.0, 0.0).size() == 1;
	testSuccessful &= quadTree3DCap1.ValidateTree();
	testSuccessful &= quadTree3DCap1.RayQuery(-200.0, 100.0, 0.0, 0.0, 100.0, -200.0).size() == 1;
	testSuccessful &= quadTree3DCap1.ValidateTree();

	testSuccessful &= quadTree3DCap1.RayQuery(0.0, 0.0, -200.0, 200.0, 200.0, 0.0).size() == 1;
	testSuccessful &= quadTree3DCap1.ValidateTree();
	testSuccessful &= quadTree3DCap1.RayQuery(200.0, 200.0, 0.0, 0.0, 0.0, -200.0).size() == 1;
	testSuccessful &= quadTree3DCap1.ValidateTree();


	if (testSuccessful == true)
	{
		std::cout << "Test of 3D ray query's edge cases successful." << std::endl;

		return true;
	}
	else
	{
		std::cout << "Test of 3D ray query's edge cases not successful." << std::endl;

		return false;
	}
}

bool QuadTreeTester::TestTreeOperations() const
{
	bool treeOperationsSuccessful = true;


	// 1D, 1 capacity
	QuadTree<1, 1, int> quadTree1DCap1(-400.0, 400.0);
	std::vector<std::vector<QuadTreeTester::TestElement<1, int>>> testElements1DCap1;

	treeOperationsSuccessful &= this->TestInsertion(quadTree1DCap1, testElements1DCap1, 1025); // (2^10+1)
	treeOperationsSuccessful &= this->TestLocationQueries(quadTree1DCap1, testElements1DCap1);
	treeOperationsSuccessful &= this->TestDistanceQueries(quadTree1DCap1, testElements1DCap1);
	treeOperationsSuccessful &= this->TestRelocation(quadTree1DCap1, testElements1DCap1);
	treeOperationsSuccessful &= this->TestDeletion(quadTree1DCap1, testElements1DCap1);

	if (treeOperationsSuccessful == false)
	{
		std::cout << "1D, 1 capacity test was not successful." << std::endl << std::endl;
		return false;
	}
	else
	{
		std::cout << "1D, 1 capacity test was successful." << std::endl << std::endl;
	}


	// 1D, 3 capacity
	QuadTree<1, 3, int> quadTree1DCap3(-400.0, 400.0);
	std::vector<std::vector<QuadTreeTester::TestElement<1, int>>> testElements1DCap3;

	treeOperationsSuccessful &= this->TestInsertion(quadTree1DCap3, testElements1DCap3, 1025); // (2^10+1)
	treeOperationsSuccessful &= this->TestLocationQueries(quadTree1DCap3, testElements1DCap3);
	treeOperationsSuccessful &= this->TestDistanceQueries(quadTree1DCap3, testElements1DCap3);
	treeOperationsSuccessful &= this->TestRelocation(quadTree1DCap3, testElements1DCap3);
	treeOperationsSuccessful &= this->TestDeletion(quadTree1DCap3, testElements1DCap3);

	if (treeOperationsSuccessful == false)
	{
		std::cout << "1D, 3 capacity test was not successful." << std::endl << std::endl;
		return false;
	}
	else
	{
		std::cout << "1D, 3 capacity test was successful." << std::endl << std::endl;
	}


	// 2D, 1 capacity
	QuadTree<2, 1, int> quadTree2DCap1(-400.0, 400.0, -400.0, 400.0);
	std::vector<std::vector<QuadTreeTester::TestElement<2, int>>> testElements2DCap1;

	treeOperationsSuccessful &= this->TestInsertion(quadTree2DCap1, testElements2DCap1, 1089); // (2^5+1)*(2^5+1)
	treeOperationsSuccessful &= this->TestLocationQueries(quadTree2DCap1, testElements2DCap1);
	treeOperationsSuccessful &= this->TestDistanceQueries(quadTree2DCap1, testElements2DCap1);
	treeOperationsSuccessful &= this->TestRayQuery_2D(quadTree2DCap1, testElements2DCap1);
	treeOperationsSuccessful &= this->TestRelocation(quadTree2DCap1, testElements2DCap1);
	treeOperationsSuccessful &= this->TestDeletion(quadTree2DCap1, testElements2DCap1);

	if (treeOperationsSuccessful == false)
	{
		std::cout << "2D, 1 capacity test was not successful." << std::endl << std::endl;
		return false;
	}
	else
	{
		std::cout << "2D, 1 capacity test was successful." << std::endl << std::endl;
	}
	
	// 2D, 3 capacity
	QuadTree<2, 3, int> quadTree2DCap3(-400.0, 400.0, -400.0, 400.0);
	std::vector<std::vector<QuadTreeTester::TestElement<2, int>>> testElements2DCap3;

	treeOperationsSuccessful &= this->TestInsertion(quadTree2DCap3, testElements2DCap3, 1089); // (2^5+1)*(2^5+1)
	treeOperationsSuccessful &= this->TestLocationQueries(quadTree2DCap3, testElements2DCap3);
	treeOperationsSuccessful &= this->TestDistanceQueries(quadTree2DCap3, testElements2DCap3);
	treeOperationsSuccessful &= this->TestRayQuery_2D(quadTree2DCap3, testElements2DCap3);
	treeOperationsSuccessful &= this->TestRelocation(quadTree2DCap3, testElements2DCap3);
	treeOperationsSuccessful &= this->TestDeletion(quadTree2DCap3, testElements2DCap3);

	if (treeOperationsSuccessful == false)
	{
		std::cout << "2D, 3 capacity test was not successful." << std::endl << std::endl;
		return false;
	}
	else
	{
		std::cout << "2D, 3 capacity test was successful." << std::endl << std::endl;
	}


	// 3D, 1 capacity
	QuadTree<3, 1, int> quadTree3DCap1(-400.0, 400.0, -400.0, 400.0, -400.0, 400.0);
	std::vector<std::vector<QuadTreeTester::TestElement<3, int>>> testElements3DCap1;

	treeOperationsSuccessful &= this->TestInsertion(quadTree3DCap1, testElements3DCap1, 729); // (2^3+1)*(2^3+1)*(2^3+1)
	treeOperationsSuccessful &= this->TestLocationQueries(quadTree3DCap1, testElements3DCap1);
	treeOperationsSuccessful &= this->TestDistanceQueries(quadTree3DCap1, testElements3DCap1);
	treeOperationsSuccessful &= this->TestRayQuery_3D(quadTree3DCap1, testElements3DCap1);
	treeOperationsSuccessful &= this->TestRelocation(quadTree3DCap1, testElements3DCap1);
	treeOperationsSuccessful &= this->TestDeletion(quadTree3DCap1, testElements3DCap1);

	if (treeOperationsSuccessful == false)
	{
		std::cout << "3D, 1 capacity test was not successful." << std::endl << std::endl;
		return false;
	}
	else
	{
		std::cout << "3D, 1 capacity test was successful." << std::endl << std::endl;
	}


	// 3D, 3 capacity
	QuadTree<3, 3, int> quadTree3DCap3(-400.0, 400.0, -400.0, 400.0, -400.0, 400.0);
	std::vector<std::vector<QuadTreeTester::TestElement<3, int>>> testElements3DCap3;

	treeOperationsSuccessful &= this->TestInsertion(quadTree3DCap3, testElements3DCap3, 729); // (2^3+1)*(2^3+1)*(2^3+1)
	treeOperationsSuccessful &= this->TestLocationQueries(quadTree3DCap3, testElements3DCap3);
	treeOperationsSuccessful &= this->TestDistanceQueries(quadTree3DCap3, testElements3DCap3);
	treeOperationsSuccessful &= this->TestRayQuery_3D(quadTree3DCap3, testElements3DCap3);
	treeOperationsSuccessful &= this->TestRelocation(quadTree3DCap3, testElements3DCap3);
	treeOperationsSuccessful &= this->TestDeletion(quadTree3DCap3, testElements3DCap3);

	if (treeOperationsSuccessful == false)
	{
		std::cout << "3D, 3 capacity test was not successful." << std::endl << std::endl;
		return false;
	}
	else
	{
		std::cout << "3D, 3 capacity test was successful." << std::endl << std::endl;
	}


	// 4D, 1 capacity
	QuadTree<4, 1, int> quadTree4DCap1(-400.0, 400.0, -400.0, 400.0, -400.0, 400.0, -400.0, 400.0);
	std::vector<std::vector<QuadTreeTester::TestElement<4, int>>> testElements4DCap1;

	treeOperationsSuccessful &= this->TestInsertion(quadTree4DCap1, testElements4DCap1, 625); // (2^2+1)*(2^2+1)*(2^2+1)*(2^2+1)
	treeOperationsSuccessful &= this->TestLocationQueries(quadTree4DCap1, testElements4DCap1);
	treeOperationsSuccessful &= this->TestDistanceQueries(quadTree4DCap1, testElements4DCap1);
	treeOperationsSuccessful &= this->TestRelocation(quadTree4DCap1, testElements4DCap1);
	treeOperationsSuccessful &= this->TestDeletion(quadTree4DCap1, testElements4DCap1);

	if (treeOperationsSuccessful == false)
	{
		std::cout << "4D, 1 capacity test was not successful." << std::endl << std::endl;
		return false;
	}
	else
	{
		std::cout << "4D, 1 capacity test was successful." << std::endl << std::endl;
	}


	// 4D, 3 capacity
	QuadTree<4, 3, int> quadTree4DCap3(-400.0, 400.0, -400.0, 400.0, -400.0, 400.0, -400.0, 400.0);
	std::vector<std::vector<QuadTreeTester::TestElement<4, int>>> testElements4DCap3;

	treeOperationsSuccessful &= this->TestInsertion(quadTree4DCap3, testElements4DCap3, 625); // (2^2+1)*(2^2+1)*(2^2+1)*(2^2+1)
	treeOperationsSuccessful &= this->TestLocationQueries(quadTree4DCap3, testElements4DCap3);
	treeOperationsSuccessful &= this->TestDistanceQueries(quadTree4DCap3, testElements4DCap3);
	treeOperationsSuccessful &= this->TestRelocation(quadTree4DCap3, testElements4DCap3);
	treeOperationsSuccessful &= this->TestDeletion(quadTree4DCap3, testElements4DCap3);

	if (treeOperationsSuccessful == false)
	{
		std::cout << "4D, 3 capacity test was not successful." << std::endl << std::endl;
		return false;
	}
	else
	{
		std::cout << "4D, 3 capacity test was successful." << std::endl << std::endl;
	}


	// 2D, 1 capacity, depth 1
	QuadTree<2, 1, int, 1> quadTree2DCap1_Depth1(-400.0, 400.0, -400.0, 400.0);
	std::vector<std::vector<QuadTreeTester::TestElement<2, int>>> testElements2DCap1_Depth1;

	treeOperationsSuccessful &= this->TestInsertion(quadTree2DCap1_Depth1, testElements2DCap1_Depth1, 1089); // (2^5+1)*(2^5+1)
	treeOperationsSuccessful &= this->TestLocationQueries(quadTree2DCap1_Depth1, testElements2DCap1_Depth1);
	treeOperationsSuccessful &= this->TestDistanceQueries(quadTree2DCap1_Depth1, testElements2DCap1_Depth1);
	treeOperationsSuccessful &= this->TestRayQuery_2D(quadTree2DCap1_Depth1, testElements2DCap1_Depth1);
	treeOperationsSuccessful &= this->TestRelocation(quadTree2DCap1_Depth1, testElements2DCap1_Depth1);
	treeOperationsSuccessful &= this->TestDeletion(quadTree2DCap1_Depth1, testElements2DCap1_Depth1);

	if (treeOperationsSuccessful == false)
	{
		std::cout << "2D, 1 capacity, depth 1 test was not successful." << std::endl << std::endl;
		return false;
	}
	else
	{
		std::cout << "2D, 1 capacity, depth 1 test was successful." << std::endl << std::endl;
	}


	// 2D, 1 capacity, depth 3
	QuadTree<2, 1, int, 3> quadTree2DCap1_Depth3(-400.0, 400.0, -400.0, 400.0);
	std::vector<std::vector<QuadTreeTester::TestElement<2, int>>> testElements2DCap1_Depth3;

	treeOperationsSuccessful &= this->TestInsertion(quadTree2DCap1_Depth3, testElements2DCap1_Depth3, 1089); // (2^5+1)*(2^5+1)
	treeOperationsSuccessful &= this->TestLocationQueries(quadTree2DCap1_Depth3, testElements2DCap1_Depth3);
	treeOperationsSuccessful &= this->TestDistanceQueries(quadTree2DCap1_Depth3, testElements2DCap1_Depth3);
	treeOperationsSuccessful &= this->TestRayQuery_2D(quadTree2DCap1_Depth3, testElements2DCap1_Depth3);
	treeOperationsSuccessful &= this->TestRelocation(quadTree2DCap1_Depth3, testElements2DCap1_Depth3);
	treeOperationsSuccessful &= this->TestDeletion(quadTree2DCap1_Depth3, testElements2DCap1_Depth3);

	if (treeOperationsSuccessful == false)
	{
		std::cout << "2D, 1 capacity, depth 3 test was not successful." << std::endl << std::endl;
		return false;
	}
	else
	{
		std::cout << "2D, 1 capacity, depth 3 test was successful." << std::endl << std::endl;
	}


	// Independent test for location query's edge cases in 2D.
	treeOperationsSuccessful &= this->TestLocationQueriesEdgeCases();

	if (treeOperationsSuccessful == false)
	{
		std::cout << "Independent test for location query's edge cases was not successful." << std::endl << std::endl;
		return false;
	}
	else
	{
		std::cout << "Independent test for location query's edge cases was successful." << std::endl << std::endl;
	}
	
	// Independent test for distance query's edge cases in 2D.
	treeOperationsSuccessful &= this->TestRadiusQueryEdgeCases();
	
	if (treeOperationsSuccessful == false)
	{
		std::cout << "Independent test for distance query's edge cases was not successful." << std::endl << std::endl;
		return false;
	}
	else
	{
		std::cout << "Independent test for radius query's edge cases was successful." << std::endl << std::endl;
	}

	// Independent test for ray query's edge cases in 2D.
	treeOperationsSuccessful &= this->TestRayQueryEdgeCases_2D();

	if (treeOperationsSuccessful == false)
	{
		std::cout << "Independent test for ray query's edge cases in 2D was not successful." << std::endl << std::endl;
		return false;
	}
	else
	{
		std::cout << "Independent test for ray query's edge cases in 2D was successful." << std::endl << std::endl;
	}

	// Independent test for ray query's edge cases in 3D.
	treeOperationsSuccessful &= this->TestRayQueryEdgeCases_3D();

	if (treeOperationsSuccessful == false)
	{
		std::cout << "Independent test for ray query's edge cases in 3D was not successful." << std::endl << std::endl;
		return false;
	}
	else
	{
		std::cout << "Independent test for ray query's edge cases in 3D was successful." << std::endl << std::endl;
	}

	return true;
}

bool QuadTreeTester::TestTreeConstruction() const
{
	bool treeOperationsSuccessful = true;
	bool treeConstructionSuccessful = true;


	// Copy construction.
	QuadTree<2, 3, int> quadTree2DCap3_CopyConstructionBaseline(-400.0, 400.0, -400.0, 400.0);
	std::vector<std::vector<QuadTreeTester::TestElement<2, int>>> testElements2DCap3_CopyConstructionBaseline;

	treeOperationsSuccessful &= this->TestInsertion(quadTree2DCap3_CopyConstructionBaseline, testElements2DCap3_CopyConstructionBaseline, 50); // (2^4+1)*(2^4+1)

	QuadTree<2, 3, int> quadTree2DCap3_CopyConstructed(quadTree2DCap3_CopyConstructionBaseline);

	treeConstructionSuccessful &= quadTree2DCap3_CopyConstructionBaseline.ValidateTree();
	treeConstructionSuccessful &= quadTree2DCap3_CopyConstructed.ValidateTree();

	treeOperationsSuccessful &= this->TestDeletion(quadTree2DCap3_CopyConstructed, testElements2DCap3_CopyConstructionBaseline);


	// Copy assignment.
	QuadTree<2, 3, int> quadTree2DCap3_CopyAssignmentBaseline(-400.0, 400.0, -400.0, 400.0);
	std::vector<std::vector<QuadTreeTester::TestElement<2, int>>> testElements2DCap3_CopyAssignmentBaseline;

	treeOperationsSuccessful &= this->TestInsertion(quadTree2DCap3_CopyAssignmentBaseline, testElements2DCap3_CopyAssignmentBaseline, 50); // (2^4+1)*(2^4+1)

	QuadTree<2, 3, int> quadTree2DCap3_CopyAssigned_CopyConstructedTarget(quadTree2DCap3_CopyAssignmentBaseline);
	quadTree2DCap3_CopyAssigned_CopyConstructedTarget = quadTree2DCap3_CopyAssignmentBaseline;

	treeConstructionSuccessful &= quadTree2DCap3_CopyAssignmentBaseline.ValidateTree();
	treeConstructionSuccessful &= quadTree2DCap3_CopyAssigned_CopyConstructedTarget.ValidateTree();

	treeOperationsSuccessful &= this->TestDeletion(quadTree2DCap3_CopyAssigned_CopyConstructedTarget, testElements2DCap3_CopyAssignmentBaseline);


	// Move constructor.
	QuadTree<2, 3, int> quadTree2DCap3_MoveConstructionBaseline(-400.0, 400.0, -400.0, 400.0);
	std::vector<std::vector<QuadTreeTester::TestElement<2, int>>> testElements2DCap3_MoveConstructionBaseline;

	treeOperationsSuccessful &= this->TestInsertion(quadTree2DCap3_MoveConstructionBaseline, testElements2DCap3_MoveConstructionBaseline, 50); // (2^4+1)*(2^4+1)

	QuadTree<2, 3, int> quadTree2DCap3_MoveConstructed(std::move(quadTree2DCap3_MoveConstructionBaseline));

	// treeConstructionSuccessful &= quadTree2DCap3_MoveConstructionBaseline.ValidateTree(); // Undefined.
	treeConstructionSuccessful &= quadTree2DCap3_MoveConstructed.ValidateTree();

	treeOperationsSuccessful &= this->TestDeletion(quadTree2DCap3_MoveConstructed, testElements2DCap3_MoveConstructionBaseline);


	// Move assignment.
	QuadTree<2, 3, int> quadTree2DCap3_MoveAssignmentBaseline(-400.0, 400.0, -400.0, 400.0);
	std::vector<std::vector<QuadTreeTester::TestElement<2, int>>> testElements2DCap3_MoveAssignmentBaseline;

	treeOperationsSuccessful &= this->TestInsertion(quadTree2DCap3_MoveAssignmentBaseline, testElements2DCap3_MoveAssignmentBaseline, 50); // (2^4+1)*(2^4+1)

	QuadTree<2, 3, int> quadTree2DCap3_MoveAssigned_CopyConstructedTarget(quadTree2DCap3_MoveAssignmentBaseline);
	quadTree2DCap3_MoveAssigned_CopyConstructedTarget = std::move(quadTree2DCap3_MoveAssignmentBaseline);

	// treeConstructionSuccessful &= quadTree2DCap3_MoveAssignmentBaseline.ValidateTree(); // Undefined.
	treeConstructionSuccessful &= quadTree2DCap3_MoveAssigned_CopyConstructedTarget.ValidateTree();

	treeOperationsSuccessful &= this->TestDeletion(quadTree2DCap3_MoveAssigned_CopyConstructedTarget, testElements2DCap3_MoveAssignmentBaseline);


	if (treeConstructionSuccessful == false || treeOperationsSuccessful == false)
	{
		std::cout << "Tree construction test was not successful." << std::endl << std::endl;
		return false;
	}
	else
	{
		std::cout << "Tree construction test was successful." << std::endl << std::endl;
	}

	return true;
}

bool QuadTreeTester::TestTree() const
{
	bool testSuccessful = true;

	testSuccessful &= this->TestTreeOperations();
	testSuccessful &= this->TestTreeConstruction();

	if (testSuccessful == false)
	{
		std::cout << "Full tree test was not successful." << std::endl << std::endl;
		return false;
	}
	else
	{
		std::cout << "Full tree test was successful." << std::endl << std::endl;
		return true;
	}
}
