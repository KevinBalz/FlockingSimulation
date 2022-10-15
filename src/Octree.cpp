#include "Octree.hpp"

thread_local ExpandingPoolAllocator Octree::m_pool(8 * sizeof(Octree));
thread_local ExpandingPoolAllocator Octree::m_vecPool(8 * sizeof(Octree::OutsiderVec));