
#pragma once

#include <optional>
#include <tuple>
#include <utility>


// Helper metaprogramming construct that computes a power for unsigned integers.

// General case. Power is calculated using recursive repeated multiplication.
template<size_t BASE, size_t EXP>
struct Pow
{
	constexpr static size_t value = BASE*Pow<BASE, EXP - 1>::value;
};

// Base case. BASE^1 = BASE.
template<size_t BASE>
struct Pow<BASE, 1>
{
	constexpr static size_t value = BASE;
};


// Helper function that splits parameter pack and stores the first half into the first array and second half into the second array.

template<size_t... indexesHalfSize, typename T, size_t N, typename... Args>
void SplitAndSaveParameterPackIntoArrays_Splitter(std::index_sequence<indexesHalfSize...>, std::array<T, N>& firstHalfArray, std::array<T, N>& secondHalfArray, Args... parameterPack)
{
	// Parameter pack is transformed into a tuple and indexes from the index sequence are deduced into indexesHalfSize.
	// indexesHalfSize are then put into std::get and unfolded. This results into choosing only certain values from the tuple (parameter pack).
	// std::get<0>, std::get<1>, ... std::get<N - 1> -> first half of the tuple (parameter pack).
	firstHalfArray = {std::get<indexesHalfSize>(std::make_tuple(parameterPack...))...};
	// std::get<N>, std::get<N + 1>, ... std::get<2*N - 1> -> second half of the tuple (parameter pack).
	secondHalfArray = {std::get<N + indexesHalfSize>(std::make_tuple(parameterPack...))...};
}

template<typename T, size_t N, typename... Args>
void SplitAndSaveParameterPackIntoArrays_SequenceGenerator(std::array<T, N>& firstHalfArray, std::array<T, N>& secondHalfArray, Args... parameterPack)
{
	static_assert(2*N == sizeof...(Args), "Parameter pack must be twice the size of the target arrays.");
	static_assert((... && std::is_same_v<Args, T>) == true, "All members of a parameter pack must be of type T.");

	// Index sequence is generated containing indexes from 0 to N - 1 (same as 0 to sizeof(Args)/2 - 1).
	SplitAndSaveParameterPackIntoArrays_Splitter(std::make_index_sequence<N>(), firstHalfArray, secondHalfArray, parameterPack...);
}


// Helper geometric functions.

// Calculates squared euclidian distance from one point to another point in any dimension.
template<size_t DIM, typename coordT>
coordT CalculateSquaredEuclidianDistanceBetweenPoints(const std::array<coordT, DIM>& firstPoint, const std::array<coordT, DIM>& secondPoint)
{
	coordT squaredDistance = static_cast<coordT>(0.0);

	for (size_t i = 0; i < DIM; ++i)
	{
		squaredDistance += (firstPoint[i] - secondPoint[i])*(firstPoint[i] - secondPoint[i]);
	}

	return squaredDistance;
}

// Returns true if two axis aligned N-dimensional volumes intersect. Touch is also considered as an intersection.
// 1D) Line segment intersection (both on single line).
// 2D) Rectange intersection.
// 3D) Rectangular cuboid intersection.
// ....
template<size_t DIM, typename coordT>
bool DoAxisAlignedNDVolumesIntersect(const std::array<coordT, 2*DIM>& firstVolumeBoundaries, const std::array<coordT, 2*DIM>& secondVolumeBoundaries)
{
	// Every dimension can be tested separately.
	// We start assuming that volumes are intersecting.
	bool volumesIntersect = true;
	size_t index = 0;

	// If intersection fails in specific dimension, then the test stops early.
	while (volumesIntersect == true && index < DIM)
	{
		// For each dimension, we are basically testing the intersection of two line segments lying on a single line.
		// ---|-1-|---|--2--|---|-1-|---
		// If the right boundary of the first one is on the left of the left boundary of the second one OR
		// if the left boundary of the first one is on the right of the right boundary of the second one THEN
		// there is no intersection. Because the default value is true, the condition is negated.
		volumesIntersect &= !(firstVolumeBoundaries[2*index] > secondVolumeBoundaries[2*index + 1] || firstVolumeBoundaries[2*index + 1] < secondVolumeBoundaries[2*index]);

		++index;
	}

	return volumesIntersect;
}

// Returns true if 2D line intersects 2D point.
template<typename coordT>
bool DoesLineIntersectPoint_2D(coordT lineFirstPointX, coordT lineFirstPointY, coordT lineSecondPointX, coordT lineSecondPointY, coordT PointX, coordT PointY, coordT epsilon)
{
	// https://stackoverflow.com/questions/11907947/how-to-check-if-a-point-lies-on-a-line-between-2-other-points
	// 2D "cross product" based test. Point lies on a line if cross product of the three points is equal to zero.

	coordT crossProduct = (PointX - lineFirstPointX)*(lineSecondPointY - lineFirstPointY) - (PointY - lineFirstPointY)*(lineSecondPointX - lineFirstPointX);

	if (crossProduct >= -epsilon && crossProduct <= epsilon)
	{
		return true;
	}
	else
	{
		return false;
	}
}

// Returns X coordinate of an intersection point of any 2D line and 2D horizontal line (if exists).
// fLFPX means fistLineFirstPointX, sLSPY means secondLineSecondPointY, etc.
template<typename coordT>
std::optional<coordT> CalculateLineHorizontalLineIntersectionPoint_2D_X(coordT fLFPX, coordT fLFPY, coordT fLSPX, coordT fLSPY, coordT sLFPX, coordT sLFPY, coordT sLSPX, coordT sLSPY, coordT epsilon)
{
	// https://en.wikipedia.org/wiki/Line–line_intersection
	// Simplifed version of a generic line-line intersection point calculation.
	// Second line is horizontal, so sLFPY == sLSPY. This makes some of the terms zero.
	// Also, this only returns the value of intersection point for X, because Y == sLFPY == sLSPY.

	std::optional<coordT> returnPoint;

	coordT denominator = -(fLFPY - fLSPY)*(sLFPX - sLSPX);

	// If lines are parallel or identical (denominator == 0), no intersection point is returned.
	if (denominator >= -epsilon && denominator <= epsilon)
	{
		return returnPoint;
	}

	returnPoint.emplace(((fLFPX*fLSPY - fLFPY*fLSPX)*(sLFPX - sLSPX) - (fLFPX - fLSPX)*(sLFPX*sLSPY - sLFPY*sLSPX))/denominator);

	return returnPoint;
}

// Returns Y coordinate of an intersection point of any 2D line and 2D vertical line (if exists).
// fLFPX means fistLineFirstPointX, sLSPY means secondLineSecondPointY, etc.
template<typename coordT>
std::optional<coordT> CalculateLineVerticalLineIntersectionPoint_2D_Y(coordT fLFPX, coordT fLFPY, coordT fLSPX, coordT fLSPY, coordT sLFPX, coordT sLFPY, coordT sLSPX, coordT sLSPY, coordT epsilon)
{
	// https://en.wikipedia.org/wiki/Line–line_intersection
	// Simplifed version of a generic line-line intersection point calculation.
	// Second line is vertical, so sLFPX == sLSPX. This makes some of the terms zero.
	// Also, this only returns the value of intersection point for Y, because X == sLFPX == sLSPX.

	std::optional<coordT> returnPoint;

	coordT denominator = (fLFPX - fLSPX)*(sLFPY - sLSPY);

	// If lines are parallel or identical (denominator == 0), no intersection point is returned.
	if (denominator >= -epsilon && denominator <= epsilon)
	{
		return returnPoint;
	}

	returnPoint.emplace(((fLFPX*fLSPY - fLFPY*fLSPX)*(sLFPY - sLSPY) - (fLFPY - fLSPY)*(sLFPX*sLSPY - sLFPY*sLSPX))/denominator);

	return returnPoint;
}

// Returns true is 2D line intersects rectange.
// IMPORTANT - Works only for non-degenrate rectangles.
template<typename coordT>
bool DoesLineIntersectRectangle_2D(coordT lineFirstPointX, coordT lineFirstPointY, coordT lineSecondPointX, coordT lineSecondPointY, coordT rectangleBoundaryMinX, coordT rectangleBoundaryMaxX, coordT rectangleBoundaryMinY, coordT rectangleBoundaryMaxY, coordT epsilon)
{
	// If any side of the rectangle intersects with the line, then the line intersects the rectangle.
	// If the line is colinear or identical with the rectangle's side, then no value is returned for the intersection point.
	// But importantly, if there is an intersection, then at least one side intersects in a point.
	// In other words, the line cannot be colinear in both the X and Y direction, so if there is no value
	// returned, then the result can just be ignored. Other condition will catch the intersection.

	std::optional<coordT> minXSideIntersection = CalculateLineVerticalLineIntersectionPoint_2D_Y(lineFirstPointX, lineFirstPointY, lineSecondPointX, lineSecondPointY, rectangleBoundaryMinX, rectangleBoundaryMinY, rectangleBoundaryMinX, rectangleBoundaryMaxY, epsilon);

	if (minXSideIntersection.has_value() == true && minXSideIntersection.value() >= rectangleBoundaryMinY - epsilon && minXSideIntersection.value() <= rectangleBoundaryMaxY + epsilon)
	{
		return true;
	}

	std::optional<coordT> maxXSideIntersection = CalculateLineVerticalLineIntersectionPoint_2D_Y(lineFirstPointX, lineFirstPointY, lineSecondPointX, lineSecondPointY, rectangleBoundaryMaxX, rectangleBoundaryMinY, rectangleBoundaryMaxX, rectangleBoundaryMaxY, epsilon);

	if (maxXSideIntersection.has_value() == true && maxXSideIntersection.value() >= rectangleBoundaryMinY - epsilon && maxXSideIntersection.value() <= rectangleBoundaryMaxY + epsilon)
	{
		return true;
	}

	std::optional<coordT> minYSideIntersection = CalculateLineHorizontalLineIntersectionPoint_2D_X(lineFirstPointX, lineFirstPointY, lineSecondPointX, lineSecondPointY, rectangleBoundaryMinX, rectangleBoundaryMinY, rectangleBoundaryMaxX, rectangleBoundaryMinY, epsilon);

	if (minYSideIntersection.has_value() == true && minYSideIntersection.value() >= rectangleBoundaryMinX - epsilon && minYSideIntersection.value() <= rectangleBoundaryMaxX + epsilon)
	{
		return true;
	}

	std::optional<coordT> maxYSideIntersection = CalculateLineHorizontalLineIntersectionPoint_2D_X(lineFirstPointX, lineFirstPointY, lineSecondPointX, lineSecondPointY, rectangleBoundaryMinX, rectangleBoundaryMaxY, rectangleBoundaryMaxX, rectangleBoundaryMaxY, epsilon);

	if (maxYSideIntersection.has_value() == true && maxYSideIntersection.value() >= rectangleBoundaryMinX - epsilon && maxYSideIntersection.value() <= rectangleBoundaryMaxX + epsilon)
	{
		return true;
	}

	return false;
}

// Returns true if 3D line intersects 3D point.
template<typename coordT>
bool DoesLineIntersectPoint_3D(coordT lineFirstPointX, coordT lineFirstPointY, coordT lineFirstPointZ, coordT lineSecondPointX, coordT lineSecondPointY, coordT lineSecondPointZ, coordT pointX, coordT pointY, coordT pointZ, coordT epsilon)
{
	// 3D cross product based test. Point lies on a line if the length of the normal vector
	// produced by a cross product is zero.

	// First vector: point -> lineFirstPoint.
	coordT firstVectorX = lineFirstPointX - pointX;
	coordT firstVectorY = lineFirstPointY - pointY;
	coordT firstVectorZ = lineFirstPointZ - pointZ;

	// Second vector point -> lineSecondPoint.
	coordT secondVectorX = lineSecondPointX - pointX;
	coordT secondVectorY = lineSecondPointY - pointY;
	coordT secondVectorZ = lineSecondPointZ - pointZ;

	// Cross product calculation:
	// u = (u1, u2, u3), v = (v1, v2, v3)
	// w = u × v = (u2*v3 - u3*v2; u3*v1 - u1*v3; u1*v2 - u2*v1)

	// Instead of calculating the length of the normal vector, we test whether the
	// endpoint of the normal vector is the same as point. The test will be performed
	// dimension by dimension.

	coordT normalX = firstVectorY*secondVectorZ - firstVectorZ*secondVectorY;

	if (normalX < pointX - epsilon || pointX + epsilon < normalX)
	{
		return false;
	}

	coordT normalY = firstVectorZ*secondVectorX - firstVectorX*secondVectorZ;

	if (normalY < pointY - epsilon || pointY + epsilon < normalY)
	{
		return false;
	}

	coordT normalZ = firstVectorX*secondVectorY - firstVectorY*secondVectorX;

	if (normalZ < pointZ - epsilon || pointZ + epsilon < normalZ)
	{
		return false;
	}

	return true;
}

// Returns X and Y coordinate of an intersection point of any 3D line and XY plane (if exists).
// lFPX means lineFirstPointX, pFPX means planeFirstPointX, etc.
template<typename coordT>
std::optional<std::pair<coordT, coordT>> CalculateLineXYPlaneIntersectionPoint_3D_XY(coordT lFPX, coordT lFPY, coordT lFPZ, coordT lSPX, coordT lSPY, coordT lSPZ, coordT pFPX, coordT pFPY, coordT pFPZ, coordT pSPX, coordT pSPY,/* coordT pSPZ,*/ coordT pTPX, coordT pTPY,/* coordT pTPZ,*/ coordT epsilon)
{
	// Formula taken from https://en.wikipedia.org/wiki/Line–plane_intersection.

	// This version deals with XY plane, which means that pFPZ == pSPZ == pTPZ.
	// Every part that becomes zero because of this fact is commented out from the general formula to increase performance.

	std::optional<std::pair<coordT, coordT>> returnPair;

	// Flipped (vector (line first point -> line second point)).
	coordT flippedLineVectorX = -(lSPX - lFPX);
	coordT flippedLineVectorY = -(lSPY - lFPY);
	coordT flippedLineVectorZ = -(lSPZ - lFPZ);

	// Vector (plane first point -> plane second point).
	coordT firstPlaneVectorX = pSPX - pFPX;
	coordT firstPlaneVectorY = pSPY - pFPY;
	// coordT firstPlaneVectorZ = pSPZ - pFPZ;

	// Vector (plane first point -> plane third point).
	coordT secondPlaneVectorX = pTPX - pFPX;
	coordT secondPlaneVectorY = pTPY - pFPY;
	// coordT secondPlaneVectorZ = pTPZ - pFPZ;

	// Normal vector of the two plane vectors (calculated using cross product).
	// coordT normalPlaneVectorX = firstPlaneVectorY*secondPlaneVectorZ - firstPlaneVectorZ*secondPlaneVectorY;
	// coordT normalPlaneVectorY = firstPlaneVectorZ*secondPlaneVectorX - firstPlaneVectorX*secondPlaneVectorZ;
	coordT normalPlaneVectorZ = firstPlaneVectorX*secondPlaneVectorY - firstPlaneVectorY*secondPlaneVectorX;

	// To test if the line lies inside the plane or is parallel to it, the dot product of the plane normal vector
	// and the line vector will be calculated. If the result is zero, then their angle is 90 degrees,
	// which means that the line is inside the plane.
	coordT linePlaneNormalDotProduct = /*flippedLineVectorX*normalPlaneVectorX + *//*flippedLineVectorY*normalPlaneVectorY + */flippedLineVectorZ*normalPlaneVectorZ;

	if (linePlaneNormalDotProduct >= -epsilon && linePlaneNormalDotProduct <= epsilon)
	{
		return returnPair;
	}

	// Parameter t is calculated (see wiki page).
	coordT t = (/*normalPlaneVectorX*(lFPX - pFPX) + *//*normalPlaneVectorY*(lFPY - pFPY) + */normalPlaneVectorZ*(lFPZ - pFPZ))/linePlaneNormalDotProduct;

	// Final intersection point is calculated.
	returnPair.emplace(lFPX + (-flippedLineVectorX)*t, lFPY + (-flippedLineVectorY)*t);

	return returnPair;
}

// Returns X and Z coordinate of an intersection point of any 3D line and XZ plane (if exists).
// lFPX means lineFirstPointX, pFPX means planeFirstPointX, etc.
template<typename coordT>
std::optional<std::pair<coordT, coordT>> CalculateLineXZPlaneIntersectionPoint_3D_XZ(coordT lFPX, coordT lFPY, coordT lFPZ, coordT lSPX, coordT lSPY, coordT lSPZ, coordT pFPX, coordT pFPY, coordT pFPZ, coordT pSPX,/* coordT pSPY,*/ coordT pSPZ, coordT pTPX,/* coordT pTPY,*/ coordT pTPZ, coordT epsilon)
{
	// Formula taken from https://en.wikipedia.org/wiki/Line–plane_intersection.

	// This version deals with XZ plane, which means that pFPY == pSPY == pTPY.
	// Every part that becomes zero because of this fact is commented out from the general formula to increase performance.

	std::optional<std::pair<coordT, coordT>> returnPair;

	// Flipped (vector (line first point -> line second point)).
	coordT flippedLineVectorX = -(lSPX - lFPX);
	coordT flippedLineVectorY = -(lSPY - lFPY);
	coordT flippedLineVectorZ = -(lSPZ - lFPZ);

	// Vector (plane first point -> plane second point).
	coordT firstPlaneVectorX = pSPX - pFPX;
	// coordT firstPlaneVectorY = pSPY - pFPY;
	coordT firstPlaneVectorZ = pSPZ - pFPZ;

	// Vector (plane first point -> plane third point).
	coordT secondPlaneVectorX = pTPX - pFPX;
	// coordT secondPlaneVectorY = pTPY - pFPY;
	coordT secondPlaneVectorZ = pTPZ - pFPZ;

	// Normal vector of the two plane vectors (calculated using cross product).
	// coordT normalPlaneVectorX = firstPlaneVectorY*secondPlaneVectorZ - firstPlaneVectorZ*secondPlaneVectorY;
	coordT normalPlaneVectorY = firstPlaneVectorZ*secondPlaneVectorX - firstPlaneVectorX*secondPlaneVectorZ;
	// coordT normalPlaneVectorZ = firstPlaneVectorX*secondPlaneVectorY - firstPlaneVectorY*secondPlaneVectorX;

	// To test if the line lies inside the plane or is parallel to it, the dot product of the plane normal vector
	// and the line vector will be calculated. If the result is zero, then their angle is 90 degrees,
	// which means that the line is inside the plane.
	coordT linePlaneNormalDotProduct = /*flippedLineVectorX*normalPlaneVectorX + */flippedLineVectorY*normalPlaneVectorY/* + flippedLineVectorZ*normalPlaneVectorZ*/;

	if (linePlaneNormalDotProduct >= -epsilon && linePlaneNormalDotProduct <= epsilon)
	{
		return returnPair;
	}

	// Parameter t is calculated (see wiki page).
	coordT t = (/*normalPlaneVectorX*(lFPX - pFPX) + */normalPlaneVectorY*(lFPY - pFPY)/* + normalPlaneVectorZ*(lFPZ - pFPZ)*/)/linePlaneNormalDotProduct;

	// Final intersection point is calculated.
	returnPair.emplace(lFPX + (-flippedLineVectorX)*t, lFPZ + (-flippedLineVectorZ)*t);

	return returnPair;
}

// Returns Y and Z coordinate of an intersection point of any 3D line and YZ plane (if exists).
// lFPX means lineFirstPointX, pFPX means planeFirstPointX, etc.
template<typename coordT>
std::optional<std::pair<coordT, coordT>> CalculateLineYZPlaneIntersectionPoint_3D_YZ(coordT lFPX, coordT lFPY, coordT lFPZ, coordT lSPX, coordT lSPY, coordT lSPZ, coordT pFPX, coordT pFPY, coordT pFPZ,/* coordT pSPX,*/ coordT pSPY, coordT pSPZ,/* coordT pTPX,*/ coordT pTPY, coordT pTPZ, coordT epsilon)
{
	// Formula taken from https://en.wikipedia.org/wiki/Line–plane_intersection.

	// This version deals with YZ plane, which means that pFPX == pSPX == pTPX.
	// Every part that becomes zero because of this fact is commented out from the general formula to increase performance.

	std::optional<std::pair<coordT, coordT>> returnPair;

	// Flipped (vector (line first point -> line second point)).
	coordT flippedLineVectorX = -(lSPX - lFPX);
	coordT flippedLineVectorY = -(lSPY - lFPY);
	coordT flippedLineVectorZ = -(lSPZ - lFPZ);

	// Vector (plane first point -> plane second point).
	// coordT firstPlaneVectorX = pSPX - pFPX;
	coordT firstPlaneVectorY = pSPY - pFPY;
	coordT firstPlaneVectorZ = pSPZ - pFPZ;

	// Vector (plane first point -> plane third point).
	// coordT secondPlaneVectorX = pTPX - pFPX;
	coordT secondPlaneVectorY = pTPY - pFPY;
	coordT secondPlaneVectorZ = pTPZ - pFPZ;

	// Normal vector of the two plane vectors (calculated using cross product).
	coordT normalPlaneVectorX = firstPlaneVectorY*secondPlaneVectorZ - firstPlaneVectorZ*secondPlaneVectorY;
	// coordT normalPlaneVectorY = firstPlaneVectorZ*secondPlaneVectorX - firstPlaneVectorX*secondPlaneVectorZ;
	// coordT normalPlaneVectorZ = firstPlaneVectorX*secondPlaneVectorY - firstPlaneVectorY*secondPlaneVectorX;

	// To test if the line lies inside the plane or is parallel to it, the dot product of the plane normal vector
	// and the line vector will be calculated. If the result is zero, then their angle is 90 degrees,
	// which means that the line is inside the plane.
	coordT linePlaneNormalDotProduct = flippedLineVectorX*normalPlaneVectorX/* + flippedLineVectorY*normalPlaneVectorY*//* + flippedLineVectorZ*normalPlaneVectorZ*/;

	if (linePlaneNormalDotProduct >= -epsilon && linePlaneNormalDotProduct <= epsilon)
	{
		return returnPair;
	}

	// Parameter t is calculated (see wiki page).
	coordT t = (normalPlaneVectorX*(lFPX - pFPX)/* + normalPlaneVectorY*(lFPY - pFPY)*//* + normalPlaneVectorZ*(lFPZ - pFPZ)*/)/linePlaneNormalDotProduct;

	// Final intersection point is calculated.
	returnPair.emplace(lFPY + (-flippedLineVectorY)*t, lFPZ + (-flippedLineVectorZ)*t);

	return returnPair;
}

// Returns true is 3D line intersects rectangular cuboid.
// IMPORTANT - Works only for non-degenrate rectangular cuboids.
template<typename coordT>
bool DoesLineIntersectRectangularCuboid_3D(coordT lineFirstPointX, coordT lineFirstPointY, coordT lineFirstPointZ, coordT lineSecondPointX, coordT lineSecondPointY, coordT lineSecondPointZ, coordT cuboidBoundaryMinX, coordT cuboidBoundaryMaxX, coordT cuboidBoundaryMinY, coordT cuboidBoundaryMaxY, coordT cuboidBoundaryMinZ, coordT cuboidBoundaryMaxZ, coordT epsilon)
{
	// If any side of the rectangular cuboid intersects with the line, then the line intersects the rectangular cuboid.
	// If the line is colinear or coincides with the rectangular cuboid's side, then no value is returned for the intersection point.
	// But importantly, if there is an intersection, then at least one side intersects in a point.
	// In other words, the line cannot be colinear with three planes at a time, so if there is no value
	// returned, then the result can just be ignored. Other condition will catch the intersection.

	// XY planes.

	std::optional<std::pair<coordT, coordT>> minZSideIntersection = CalculateLineXYPlaneIntersectionPoint_3D_XY(lineFirstPointX, lineFirstPointY, lineFirstPointZ, lineSecondPointX, lineSecondPointY, lineSecondPointZ, cuboidBoundaryMinX, cuboidBoundaryMinY, cuboidBoundaryMinZ, cuboidBoundaryMinX, cuboidBoundaryMaxY, cuboidBoundaryMaxX, cuboidBoundaryMinY, epsilon);

	if (minZSideIntersection.has_value() == true &&
		minZSideIntersection.value().first >= cuboidBoundaryMinX - epsilon &&
		minZSideIntersection.value().first <= cuboidBoundaryMaxX + epsilon &&
		minZSideIntersection.value().second >= cuboidBoundaryMinY - epsilon &&
		minZSideIntersection.value().second <= cuboidBoundaryMaxY + epsilon )
	{
		return true;
	}

	std::optional<std::pair<coordT, coordT>> maxZSideIntersection = CalculateLineXYPlaneIntersectionPoint_3D_XY(lineFirstPointX, lineFirstPointY, lineFirstPointZ, lineSecondPointX, lineSecondPointY, lineSecondPointZ, cuboidBoundaryMinX, cuboidBoundaryMinY, cuboidBoundaryMaxZ, cuboidBoundaryMinX, cuboidBoundaryMaxY, cuboidBoundaryMaxX, cuboidBoundaryMinY, epsilon);

	if (maxZSideIntersection.has_value() == true &&
		maxZSideIntersection.value().first >= cuboidBoundaryMinX - epsilon &&
		maxZSideIntersection.value().first <= cuboidBoundaryMaxX + epsilon &&
		maxZSideIntersection.value().second >= cuboidBoundaryMinY - epsilon &&
		maxZSideIntersection.value().second <= cuboidBoundaryMaxY + epsilon)
	{
		return true;
	}

	// XZ planes.

	std::optional<std::pair<coordT, coordT>> minYSideIntersection = CalculateLineXZPlaneIntersectionPoint_3D_XZ(lineFirstPointX, lineFirstPointY, lineFirstPointZ, lineSecondPointX, lineSecondPointY, lineSecondPointZ, cuboidBoundaryMinX, cuboidBoundaryMinY, cuboidBoundaryMinZ, cuboidBoundaryMaxX, cuboidBoundaryMinZ, cuboidBoundaryMinX, cuboidBoundaryMaxZ, epsilon);

	if (minYSideIntersection.has_value() == true &&
		minYSideIntersection.value().first >= cuboidBoundaryMinX - epsilon &&
		minYSideIntersection.value().first <= cuboidBoundaryMaxX + epsilon &&
		minYSideIntersection.value().second >= cuboidBoundaryMinZ - epsilon &&
		minYSideIntersection.value().second <= cuboidBoundaryMaxZ + epsilon)
	{
		return true;
	}

	std::optional<std::pair<coordT, coordT>> maxYSideIntersection = CalculateLineXZPlaneIntersectionPoint_3D_XZ(lineFirstPointX, lineFirstPointY, lineFirstPointZ, lineSecondPointX, lineSecondPointY, lineSecondPointZ, cuboidBoundaryMinX, cuboidBoundaryMaxY, cuboidBoundaryMinZ, cuboidBoundaryMaxX, cuboidBoundaryMinZ, cuboidBoundaryMinX, cuboidBoundaryMaxZ, epsilon);

	if (maxYSideIntersection.has_value() == true &&
		maxYSideIntersection.value().first >= cuboidBoundaryMinX - epsilon &&
		maxYSideIntersection.value().first <= cuboidBoundaryMaxX + epsilon &&
		maxYSideIntersection.value().second >= cuboidBoundaryMinZ - epsilon &&
		maxYSideIntersection.value().second <= cuboidBoundaryMaxZ + epsilon)
	{
		return true;
	}

	// YZ planes.

	std::optional<std::pair<coordT, coordT>> minXSideIntersection = CalculateLineYZPlaneIntersectionPoint_3D_YZ(lineFirstPointX, lineFirstPointY, lineFirstPointZ, lineSecondPointX, lineSecondPointY, lineSecondPointZ, cuboidBoundaryMinX, cuboidBoundaryMinY, cuboidBoundaryMinZ, cuboidBoundaryMinY, cuboidBoundaryMaxZ, cuboidBoundaryMaxY, cuboidBoundaryMinZ, epsilon);

	if (minXSideIntersection.has_value() == true &&
		minXSideIntersection.value().first >= cuboidBoundaryMinY - epsilon &&
		minXSideIntersection.value().first <= cuboidBoundaryMaxY + epsilon &&
		minXSideIntersection.value().second >= cuboidBoundaryMinZ - epsilon &&
		minXSideIntersection.value().second <= cuboidBoundaryMaxZ + epsilon)
	{
		return true;
	}

	std::optional<std::pair<coordT, coordT>> maxXSideIntersection = CalculateLineYZPlaneIntersectionPoint_3D_YZ(lineFirstPointX, lineFirstPointY, lineFirstPointZ, lineSecondPointX, lineSecondPointY, lineSecondPointZ, cuboidBoundaryMaxX, cuboidBoundaryMinY, cuboidBoundaryMinZ, cuboidBoundaryMinY, cuboidBoundaryMaxZ, cuboidBoundaryMaxY, cuboidBoundaryMinZ, epsilon);

	if (maxXSideIntersection.has_value() == true &&
		maxXSideIntersection.value().first >= cuboidBoundaryMinY - epsilon &&
		maxXSideIntersection.value().first <= cuboidBoundaryMaxY + epsilon &&
		maxXSideIntersection.value().second >= cuboidBoundaryMinZ - epsilon &&
		maxXSideIntersection.value().second <= cuboidBoundaryMaxZ + epsilon)
	{
		return true;
	}

	return false;
}
