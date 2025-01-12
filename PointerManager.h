
#pragma once

#include <array>
#include <vector>


// Class that simulates and manages dynamic memory in order to improve performance.
// This static version internally creates a fixed size array (size N) of objects of type T.
// Instead of using new to allocate an object, the Create() method can be called instead.
// It returns an adress of a free element in the array. The usage of heap is thus avoided.
// Then, instead of using delete to deallocate an object, the Delete() method can be called instead.
// It frees the array element, so the memory can be reused.
template<typename T, size_t N>
class PointerManager_Static
{
private:

	// Array of stored objects.
	T* objects;
	// Array containing a list of free indexes in objects array.
	size_t* unusedIndexes;
	// Highest achieved number of allocated objects in objects array.
	size_t objectsMaxUsedSize;
	// Current number of unused indexes.
	size_t unusedIndexesSize;

public:

	// Constructor/destructor.
	PointerManager_Static();
	~PointerManager_Static();

	// Move constructor/assignment.
	PointerManager_Static(PointerManager_Static&& pointerManager) noexcept;
	PointerManager_Static& operator=(PointerManager_Static&& pointerManager) noexcept;

	// Pointer acquisition.
	template<typename... Args>
	T* Create(Args&&... args);

	// Pointer deletion.
	void Delete(T* pointerToDelete);

	// Pointer deletion with nullptr check.
	void Delete_Nullptr(T* pointerToDelete);

private:

	// Copy constructor/assignment doesn't make sense within this context.
	PointerManager_Static(const PointerManager_Static& pointerManager) = delete;
	PointerManager_Static& operator=(const PointerManager_Static& pointerManager) = delete;

	// Deallocation of all internal memory.
	void DeallocateMemory();
};

template<typename T, size_t N>
PointerManager_Static<T, N>::PointerManager_Static()
{
	// Arrays are allocated on the heap because of their potentially big size.
	// IMPORTANT - for objects array, operator new[] is used instead of just new.
	// This is done because we don't want the objects to be default constructed.
	this->objects = static_cast<T*>(operator new[](sizeof(T)*N));
	this->unusedIndexes = new size_t[N];

	// No object is allocated.
	this->objectsMaxUsedSize = 0;
	// No index can be reused.
	this->unusedIndexesSize = 0;
}

template<typename T, size_t N>
PointerManager_Static<T, N>::~PointerManager_Static()
{
	this->DeallocateMemory();
}

template<typename T, size_t N>
PointerManager_Static<T, N>::PointerManager_Static(PointerManager_Static&& pointerManager) noexcept
{
	this->objects = pointerManager.objects;
	this->unusedIndexes = pointerManager.unusedIndexes;
	this->objectsMaxUsedSize = pointerManager.objectsMaxUsedSize;
	this->unusedIndexesSize = pointerManager.unusedIndexesSize;

	pointerManager.objects = nullptr;
	pointerManager.unusedIndexes = nullptr;
	pointerManager.objectsMaxUsedSize = 0;
	pointerManager.unusedIndexesSize = 0;
}

template<typename T, size_t N>
PointerManager_Static<T, N>& PointerManager_Static<T, N>::operator=(PointerManager_Static&& pointerManager) noexcept
{
	this->DeallocateMemory();

	this->objects = pointerManager.objects;
	this->unusedIndexes = pointerManager.unusedIndexes;
	this->objectsMaxUsedSize = pointerManager.objectsMaxUsedSize;
	this->unusedIndexesSize = pointerManager.unusedIndexesSize;

	pointerManager.objects = nullptr;
	pointerManager.unusedIndexes = nullptr;
	pointerManager.objectsMaxUsedSize = 0;
	pointerManager.unusedIndexesSize = 0;

	return *this;
}

template<typename T, size_t N>
template<typename... Args>
T* PointerManager_Static<T, N>::Create(Args&&... args)
{
	if (this->unusedIndexesSize == 0)
	{
		// No unused index to reuse is available. New index from the objects array will be used.
		// IMPORTANT - no boundary checking is done.

		// Copy assignment version. Performance is the same.
		// ++this->objectsMaxUsedSize;
		// (this->objects)[this->objectsMaxUsedSize - 1] = T(std::forward<Args>(args)...);
		// return &((this->objects)[this->objectsMaxUsedSize - 1]);

		// Placement new version.
		// objectsMaxUsedSize represents the next not yet used index in the objects array.
		// By this amount, the objects array pointer is incremented, which gives us
		// the pointer to the element, where the new object should be constructed.
		// Arguments of this function are delegated to the constructor using perfect forwarding.
		return new(this->objects + this->objectsMaxUsedSize++) T(std::forward<Args>(args)...);
	}
	else
	{
		// Unused index is available, so it is going to be reused.

		// Index is retrieved (from the ed of the array).
		size_t unusedIndex = (this->unusedIndexes)[--this->unusedIndexesSize];

		// Same principle as in the other case, but with unusedIndex.
		return new(this->objects + unusedIndex) T(std::forward<Args>(args)...);
	}
}

template<typename T, size_t N>
void PointerManager_Static<T, N>::Delete(T* pointerToDelete)
{
	// Destructor is called manually for the pointer's object,
	// so the behaviour stays the same as with the standard delete usage.
	pointerToDelete->~T();

	// Index of the object (that is now free) in the object array can be calculated
	// from the difference of pointer values (given pointer - array start pointer).
	// The index is then pushed back into the unusedIndexes array.
	(this->unusedIndexes)[this->unusedIndexesSize++] = (pointerToDelete - this->objects);
}

template<typename T, size_t N>
void PointerManager_Static<T, N>::Delete_Nullptr(T* pointerToDelete)
{
	if (pointerToDelete != nullptr)
	{
		// Same as Delete().

		pointerToDelete->~T();

		(this->unusedIndexes)[this->unusedIndexesSize++] = (pointerToDelete - this->objects);
	}
}

template<typename T, size_t N>
void PointerManager_Static<T, N>::DeallocateMemory()
{
	// Array that for each index of objects array stores if it is used or not.
	bool* indexUsedLookup = new bool[N];

	// All indexes are set to unused by default.
	for (size_t i = 0; i < N; ++i)
	{
		indexUsedLookup[i] = false;
	}

	// All indexes that were used at least once are set to used.
	for (size_t i = 0; i < this->objectsMaxUsedSize; ++i)
	{
		indexUsedLookup[i] = true;
	}

	// All indexes present in unusedIndexes array will be set to unused.
	for (size_t i = 0; i < this->unusedIndexesSize; ++i)
	{
		indexUsedLookup[this->unusedIndexes[i]] = false;
	}

	// Destructor will be called for each object at used index.
	// Check can be done only up to objectsMaxUsedSize, because
	// after this threshold, all indexes are unused.
	for (size_t i = 0; i < this->objectsMaxUsedSize; ++i)
	{
		if (indexUsedLookup[i] == true)
		{
			this->objects[i].~T();
		}
	}

	// Objects inside the objects array are already destructed.
	// We just have to return the memory using operator delete[].
	operator delete[](this->objects);

	// Deallocation of other arrays proceeds as usual.
	delete[] this->unusedIndexes;
	delete[] indexUsedLookup;
}

//---------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------

// Class that manages dynamic memory in order to improve performance.
// This unlimited version internally keeps a vector of unused pointers.
// Instead of using new to allocate an object, the Create() method can be called instead.
// It returns a new pointer or a reused pointer, if it is available.
// Then, instead of using delete to deallocate an object, the Delete() method can be called instead,
// which stores the pointer for future reuse.
template<typename T>
class PointerManager_Unlimited
{
private:

	// Vector of unused pointers with allocated memory.
	std::vector<T*> unusedPointers;

public:

	// Constructor/destructor.
	PointerManager_Unlimited();
	~PointerManager_Unlimited();

	// Move constructor/assignment.
	PointerManager_Unlimited(PointerManager_Unlimited&& pointerManager) noexcept;
	PointerManager_Unlimited& operator=(PointerManager_Unlimited&& pointerManager) noexcept;

	// Pointer preallocation.
	void Preallocate(const size_t pointersCount);

	// Pointer acquisition.
	template<typename... Args>
	T* Create(Args&&... args);

	// Pointer deletion.
	void Delete(T* pointerToDelete);

	// Pointer deletion with nullptr check.
	void Delete_Nullptr(T* pointerToDelete);

private:

	// Copy constructor/assignment doesn't make sense within this context.
	PointerManager_Unlimited(const PointerManager_Unlimited& pointerManager_Unlimited) = delete;
	PointerManager_Unlimited& operator=(const PointerManager_Unlimited& pointerManager_Unlimited) = delete;

	// Deallocation of all internal memory.
	void DeallocateMemory();
};

template<typename T>
PointerManager_Unlimited<T>::PointerManager_Unlimited()
{
}

template<typename T>
PointerManager_Unlimited<T>::~PointerManager_Unlimited()
{
	this->DeallocateMemory();
}

template<typename T>
PointerManager_Unlimited<T>::PointerManager_Unlimited(PointerManager_Unlimited&& pointerManager) noexcept : unusedPointers(std::move(pointerManager.unusedPointers))
{
}

// Move assignment.
template<typename T>
PointerManager_Unlimited<T>& PointerManager_Unlimited<T>::operator=(PointerManager_Unlimited&& pointerManager) noexcept
{
	this->DeallocateMemory();

	this->unusedPointers = std::move(pointerManager.unusedPointers);

	return *this;
}

template<typename T>
void PointerManager_Unlimited<T>::Preallocate(const size_t pointersCount)
{
	this->unusedPointers.reserve(pointersCount);

	for (size_t i = 0; i < pointersCount; ++i)
	{
		// IMPORTANT - only memory is reserved to avoid the default construction of the object.
		this->unusedPointers.push_back(static_cast<T*>(operator new (sizeof(T))));
	}
}

template<typename T>
template<typename... Args>
T* PointerManager_Unlimited<T>::Create(Args&&... args)
{
	if (this->unusedPointers.empty() == true)
	{
		// No unused pointer is available, so a new one is allocated from the heap.

		return new T(std::forward<Args>(args)...);
	}
	else
	{
		// Unused pointer is available. Its memory is going to reused for the new object.

		T* unusedPointer = this->unusedPointers.back();
		this->unusedPointers.pop_back();

		return new(unusedPointer) T(std::forward<Args>(args)...);
	}
}

template<typename T>
void PointerManager_Unlimited<T>::Delete(T* pointerToDelete)
{
	// Destructor is called manually for the pointer's object,
	// so the behaviour stays the same as with the standard delete usage.
	pointerToDelete->~T();

	this->unusedPointers.push_back(pointerToDelete);
}

template<typename T>
void PointerManager_Unlimited<T>::Delete_Nullptr(T* pointerToDelete)
{
	if (pointerToDelete != nullptr)
	{
		// Same as Delete().

		pointerToDelete->~T();
		this->unusedPointers.push_back(pointerToDelete);
	}
}

template<typename T>
void PointerManager_Unlimited<T>::DeallocateMemory()
{
	// There is no tracking of pointers that were given out, so if the manager's memory
	// is deallocated before all of them are returned, then they won't be handled.

	for (T*& unusedPointer : this->unusedPointers)
	{
		// IMPORTANT - every unused pointer points to a destructed object
		// (either not even constructed inside Preallocate() or destructed during Delete()),
		// so only the memory has to be freed.
		operator delete(unusedPointer);
	}
}
