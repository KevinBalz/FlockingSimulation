#pragma once
#include <array>
#include "Rect.hpp"
#include "Boid.hpp"
#include "JobSystem.hpp"
#include "SmallVec.hpp"
#include "ExpandingPoolAllocator.hpp"
#include <algorithm>

constexpr size_t MAX_ELEMENTS_LEAF = 64;
constexpr size_t MAX_TREE_DEPTH = 8;

template<typename Callback, typename T, size_t arr_size>
void RemoveIf(tako::SmallVec<T, arr_size>& arr, Callback& c)
{
	size_t size;
	for (int i = 0; i < (size = arr.GetLength()); i++)
	{
		auto& b = arr[i];
		bool remove = c(b);
		if (remove)
		{
			if (i == size - 1)
			{
				arr.Pop();
			}
			else
			{
				std::swap(arr[i], arr[size - 1]);
				arr.Pop();
				i--;
			}
		}
	}
}


class Octree
{
	struct Node
	{
		Boid b;
		Boid* p;
	};
	using OutsiderVec = tako::SmallVec<Node, 128>;
public:
	Octree(Rect area, size_t depth = 0) : m_area(area), m_depth(depth)
	{

	}

	~Octree()
	{
		if (m_branch)
		{
			DestroyLeafs();
		}
	}

	void Insert(Boid* boid)
	{
		Insert({*boid, boid});
	}

	void Insert(Node boid)
	{
		if (!m_area.Contains(boid.b.position))
		{
			InsertArr(boid);
			return;
		}

		InsertContained(boid);
	}

	template<typename Callback>
	void Iterate(Rect area, Callback& callback)
	{
		if (m_branch)
		{
			for (int i = 0; i < 8; i++)
			{
				auto& l = m_leafs[i];
				if (Rect::Overlaps(l.m_area, area))
				{
					l.Iterate(area, callback);
				}
			}
		}
		else
		{
			for (int i = 0; i < m_containing.size(); i++)
			{
				callback(m_containing[i].b);
			}
		}
		
	}

	void RebalanceThreaded(OutsiderVec* outsiders = nullptr)
	{
		if (!m_branch || CheckCollapse())
		{
			Rebalance(outsiders);
			return;
		}

		tako::JobSystem::Schedule([this, outsiders]()
		{
			auto ownOutsiders = reinterpret_cast<OutsiderVec*>(m_vecPool.Allocate());
			std::array<bool, 8> leftToBalance{false};
			
			for (int i = 0; i < 8; i++)
			{
				Octree* leaf = &m_leafs[i];
				auto out = new (&ownOutsiders[i]) OutsiderVec();
				if (leaf->m_branch && !leaf->CheckCollapse())
				{
					leaf->RebalanceThreaded(out);
				}
				else
				{
					leftToBalance[i] = true;
				}
			}

			for (int i = 0; i < 8; i++)
			{
				if (leftToBalance[i])
				{
					m_leafs[i].Rebalance(&ownOutsiders[i]);
				}
			}

			tako::JobSystem::Continuation([this, outsiders, ownOutsiders]()
			{
				RebalanceContaining(outsiders);
				for (int l = 0; l < 8; l++)
				{
					m_totalChildElements -= ownOutsiders[l].size();
					for (int i = 0; i < ownOutsiders[l].size(); i++)
					{
						auto& b = ownOutsiders[l][i];
						if (m_area.Contains(b.b.position) || outsiders == nullptr)
						{
							InsertContained(b);
						}
						else
						{
							outsiders->push_back(b);
						}
					}
					std::destroy_at(&ownOutsiders[l]);
				}

				m_vecPool.Deallocate(ownOutsiders);
			});
		});
	}

	void Rebalance(OutsiderVec* outsiders = nullptr)
	{
		if (m_branch)
		{
			if (CheckCollapse())
			{
				Collapse();
			}
			else
			{
				OutsiderVec ownOutsiders;
				for (int i = 0; i < 8; i++)
				{
					m_leafs[i].Rebalance(&ownOutsiders);
				}
				RebalanceContaining(outsiders);
				m_totalChildElements -= ownOutsiders.size();
				for (int i = 0; i < ownOutsiders.size(); i++)
				{
					auto& b = ownOutsiders[i];
					if (m_area.Contains(b.b.position) || outsiders == nullptr)
					{
						InsertContained(b);
					}
					else
					{
						outsiders->push_back(b);
					}
				}
			}
		}
		else
		{
			RebalanceContaining(outsiders);
		}
	}

	size_t GetElementCount()
	{
		return m_containing.size() + m_totalChildElements;
	}

	Rect GetArea()
	{
		return m_area;
	}
private:
	Octree* m_leafs = nullptr;
	tako::SmallVec<Node, MAX_ELEMENTS_LEAF> m_containing;
	size_t m_totalChildElements = 0;
	Rect m_area;
	size_t m_depth;
	bool m_branch = false;
	thread_local static ExpandingPoolAllocator m_pool;
	thread_local static ExpandingPoolAllocator m_vecPool;

	void InsertArr(Node node)
	{
		m_containing.push_back(node);
	}

	void InsertContained(Node boid)
	{
		if (!m_branch)
		{
			if (m_containing.size() == MAX_ELEMENTS_LEAF && m_depth < MAX_TREE_DEPTH)
			{
				m_leafs = reinterpret_cast<Octree*>(m_pool.Allocate());
				//Subdivide
				InitSubtree(0, -1, -1, 1);
				InitSubtree(1, -1, 1, 1);
				InitSubtree(2, -1, -1, -1);
				InitSubtree(3, -1, 1, -1);
				InitSubtree(4, 1, -1, 1);
				InitSubtree(5, 1, 1, 1);
				InitSubtree(6, 1, -1, -1);
				InitSubtree(7, 1, 1, -1);
				m_branch = true;

				RemoveIf(m_containing, [&](Node& b)
				{
					return InsertIntoChild(b);
				});
				InsertContained(boid);
				return;
			}

			InsertArr(boid);
			return;
		}

		if (!InsertIntoChild(boid))
		{
			InsertArr(boid);
		}
	}

	bool InsertIntoChild(Node boid)
	{
		for (int i = 0; i < 8; i++)
		{
			auto& leaf = m_leafs[i];
			if (leaf.m_area.Contains(boid.b.position))
			{
				leaf.InsertContained(boid);
				m_totalChildElements++;
				return true;
			}
		}
		return false;
	}

	void InitSubtree(size_t i, int x, int y, int z)
	{
		auto size = 1.0f / 2 * m_area.size;
		auto center = m_area.center + tako::Vector3(x * size.x / 2, y * size.y / 2, z * size.z / 2);
		//LOG("{} {} {}, {} {} {}", center.x, center.y, center.z, size.x, size.y, size.z);
		new (&m_leafs[i]) Octree({ center, size }, m_depth + 1);
	}

	void RebalanceContaining(OutsiderVec* outsiders = nullptr)
	{
		if (m_branch)
		{
			if (outsiders)
			{
				RemoveIf(m_containing, [&](Node& b)
				{
					b.b = *b.p;
					if (!m_area.Contains(b.b.position))
					{
						outsiders->push_back(b);
						return true;
					}
					return InsertIntoChild(b);
				});
			}
			else
			{
				RemoveIf(m_containing, [&](Node& b)
				{
					b.b = *b.p;
					return InsertIntoChild(b);
				});
			}

		}
		else if (outsiders)
		{
			RemoveIf(m_containing, [&](Node& b)
			{
				b.b = *b.p;
				if (!m_area.Contains(b.b.position))
				{
					outsiders->push_back(b);
					return true;
				}
				return false;
			});
		}
	}

	bool CheckCollapse()
	{
		return m_totalChildElements + m_containing.size() < MAX_ELEMENTS_LEAF / 2;
	}

	void Collapse()
	{
		for (int i = 0; i < 8; i++)
		{
			m_leafs[i].ExtractSap(this);
		}
		m_branch = false;
		m_totalChildElements = 0;
		DestroyLeafs();
	}

	void ExtractSap(Octree* target)
	{
		if (m_branch)
		{
			for (int i = 0; i < 8; i++)
			{
				m_leafs[i].ExtractSap(target);
			}
		}

		for (int i = 0; i < target->m_containing.size(); i++)
		{
			InsertArr(target->m_containing[i]);
		}
	}

	void DestroyLeafs()
	{
		for (int i = 0; i < 8; i++)
		{
			std::destroy_at(&m_leafs[i]);
		}
		m_pool.Deallocate(m_leafs);
	}
};