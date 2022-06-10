#pragma once
#include <array>
#include <vector>
#include "Rect.hpp"
#include "Boid.hpp"
#include "JobSystem.hpp"
#include <algorithm>

constexpr size_t MAX_ELEMENTS_LEAF = 128;
constexpr size_t MAX_TREE_DEPTH = 256;

template<typename Callback, typename T>
void RemoveIf(std::vector<T>& vec, Callback& c)
{
	for (int i = 0; i < vec.size(); i++)
	{
		auto& b = vec[i];
		bool remove = c(b);
		if (remove)
		{
			if (i == vec.size() - 1)
			{
				vec.pop_back();
			}
			else
			{
				std::swap(vec[i], vec[vec.size() - 1]);
				vec.pop_back();
				i--;
			}
		}
	}
}

template<typename Callback, typename T, size_t arr_size>
void RemoveIf(std::array<T, arr_size>& arr, size_t& size, Callback& c)
{
	for (int i = 0; i < size; i++)
	{
		auto& b = arr[i];
		bool remove = c(b);
		if (remove)
		{
			if (i == size - 1)
			{
				size--;
			}
			else
			{
				std::swap(arr[i], arr[size - 1]);
				size--;
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
public:
	Octree(Rect area, tako::Allocators::PoolAllocator& pool, size_t depth = 0) : m_area(area), m_depth(depth), m_pool(pool), m_size(0)
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

		for (int i = 0; i < m_size; i++)
		{
			callback(m_containing[i].b);
		}
	}

	/*
	//TODO: Reevaluate, not more performance as expected
	void RebalanceThreaded()
	{
		if (!m_branch || CheckCollapse())
		{
			Rebalance();
			return;
		}

		for (int i = 0; i < 8; i++)
		{
			Octree* leaf = &m_leafs[i];
			tako::JobSystem::Schedule([leaf]()
			{
				leaf->RebalanceThreaded();
			});
		}

		tako::JobSystem::Continuation([this]()
		{
			RebalanceOutsiders();
			RebalanceContaining();
			FetchChildOutsiders();
		});
	}
	*/

	void Rebalance(std::pmr::vector<Node>* outsiders = nullptr)
	{
		if (m_branch)
		{

			if (CheckCollapse())
			{
				Collapse();
			}
			else
			{
				std::array<tako::U8, 1024 * sizeof(Node)> tmpData;
				std::pmr::monotonic_buffer_resource buffer(tmpData.data(), tmpData.size());
				std::pmr::vector<Node> ownOutsiders(&buffer);
				for (int i = 0; i < 8; i++)
				{
					m_leafs[i].Rebalance(&ownOutsiders);
				}
				RebalanceContaining();
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
			RebalanceContaining();
		}
	}

	size_t GetElementCount()
	{
		return m_size + m_totalChildElements;
	}
private:
	Octree* m_leafs = nullptr;
	std::array<Node, MAX_ELEMENTS_LEAF> m_containing;
	size_t m_size;
	size_t m_totalChildElements = 0;
	Rect m_area;
	size_t m_depth;
	tako::Allocators::PoolAllocator& m_pool;
	bool m_branch = false;

	void InsertArr(Node node)
	{
		//TODO: its possible if a large amount of child boids are on borders that the parent array could overfill
		m_containing[m_size] = node;
		m_size++;
	}

	void InsertContained(Node boid)
	{
		if (!m_branch)
		{
			if (m_size == MAX_ELEMENTS_LEAF)
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

				RemoveIf(m_containing, m_size, [&](Node& b)
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
		new (&m_leafs[i]) Octree({ center, size }, m_pool, m_depth + 1);
	}

	void RebalanceContaining(std::pmr::vector<Node>* outsiders = nullptr)
	{
		if (m_branch)
		{
			if (outsiders)
			{
				RemoveIf(m_containing, m_size, [&](Node& b)
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
				RemoveIf(m_containing, m_size, [&](Node& b)
				{
					b.b = *b.p;
					return InsertIntoChild(b);
				});
			}

		}
		else if (outsiders)
		{
			RemoveIf(m_containing, m_size, [&](Node& b)
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
		return m_totalChildElements + m_size < MAX_ELEMENTS_LEAF / 2;
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

		for (int i = 0; i < target->m_size; i++)
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