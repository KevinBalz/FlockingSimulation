#pragma once
#include <array>
#include <vector>
#include "Rect.hpp"
#include "Boid.hpp"
#include "JobSystem.hpp"
#include <algorithm>

constexpr size_t MAX_ELEMENTS_LEAF = 32;
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


class Octree
{
	struct Node
	{
		Boid b;
		Boid* p;
	};
public:
	Octree(Rect area, size_t depth = 0) : m_area(area), m_depth(depth)
	{
		m_containing.reserve(MAX_ELEMENTS_LEAF);
	}

	~Octree()
	{
		if (m_branch)
		{
			for (auto leaf : m_leafs)
			{
				delete leaf;
			}
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
			m_outside.push_back(boid);
			return;
		}

		InsertContained(boid);
	}

	template<typename Callback>
	void Iterate(Rect area, Callback& callback)
	{
		if (m_branch)
		{
			for (auto l : m_leafs)
			{
				if (Rect::Overlaps(l->m_area, area))
				{
					l->Iterate(area, callback);
				}
			}
		}

		for (auto b : m_containing)
		{
			callback(b.b, b.p);
		}

		for (auto b : m_outside)
		{
			callback(b.b, b.p);
		}
	}

	void RebalanceThreaded()
	{
		if (!m_branch || CheckCollapse())
		{
			Rebalance();
			return;
		}

		for (auto leaf : m_leafs)
		{
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

	void Rebalance()
	{
		if (m_branch)
		{
			if (CheckCollapse())
			{
				Collapse();
			}
			else
			{
				for (auto leaf : m_leafs)
				{
					leaf->Rebalance();
				}
			}
		}

		RebalanceOutsiders();
		RebalanceContaining();

		if (m_branch)
		{
			FetchChildOutsiders();
		}
	}

	size_t GetElementCount()
	{
		return m_containing.size() + m_outside.size() + m_totalChildElements;
	}
private:
	std::array<Octree*, 8> m_leafs;
	std::vector<Node> m_containing;
	std::vector<Node> m_outside;
	size_t m_totalChildElements = 0;
	Rect m_area;
	size_t m_depth;
	bool m_branch = false;

	void InsertContained(Node boid)
	{
		if (!m_branch)
		{
			if (m_containing.size() == MAX_ELEMENTS_LEAF && m_depth < MAX_TREE_DEPTH)
			{
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

			m_containing.push_back(boid);
			return;
		}

		if (!InsertIntoChild(boid))
		{
			m_containing.push_back(boid);
		}
	}

	bool InsertIntoChild(Node boid)
	{
		for (auto leaf : m_leafs)
		{
			if (leaf->m_area.Contains(boid.b.position))
			{
				leaf->InsertContained(boid);
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
		m_leafs[i] = new Octree({ center, size }, m_depth+1);
	}

	void RebalanceOutsiders()
	{
		for (int i = 0; i < m_outside.size(); i++)
		{
			auto& b = m_outside[i];
			b.b = *b.p;
			if (m_area.Contains(b.b.position))
			{
				InsertContained(b);
				if (i == m_outside.size() - 1)
				{
					m_outside.pop_back();
				}
				else
				{
					std::swap(m_outside[i], m_outside[m_outside.size() - 1]);
					m_outside.pop_back();
					i--;
				}
			}
		}
	}

	void RebalanceContaining()
	{
		if (m_branch)
		{
			RemoveIf(m_containing, [&](Node& b)
			{
				b.b = *b.p;
				if (!m_area.Contains(b.b.position))
				{
					m_outside.push_back(b);
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
				if (!m_area.Contains(b.b.position))
				{
					m_outside.push_back(b);
					return true;
				}
				return false;
			});
		}
	}

	void FetchChildOutsiders()
	{
		for (auto leaf : m_leafs)
		{
			m_totalChildElements -= leaf->m_outside.size();
			for (auto b : leaf->m_outside)
			{
				Insert(b);
			}
			leaf->m_outside.clear();
		}
	}

	bool CheckCollapse()
	{
		//TODO: what if existing outsiders would cause the tree to subdivide again
		return m_totalChildElements + m_containing.size() < MAX_ELEMENTS_LEAF / 4;
	}

	void Collapse()
	{
		for (auto leaf : m_leafs)
		{
			leaf->ExtractSap(this);
		}
		m_branch = false;
		m_totalChildElements = 0;
		for (auto leaf : m_leafs)
		{
			delete leaf;
		}
	}

	void ExtractSap(Octree* target)
	{
		if (m_branch)
		{
			for (auto leaf : m_leafs)
			{
				leaf->ExtractSap(target);
			}
		}

		std::move(std::begin(m_containing), std::end(m_containing), std::back_inserter(target->m_containing));
		m_containing.clear();
		std::move(std::begin(m_outside), std::end(m_outside), std::back_inserter(target->m_containing));
		m_outside.clear();
	}
};