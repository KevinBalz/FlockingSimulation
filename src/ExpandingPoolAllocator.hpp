#pragma once
#include "Allocators/Allocator.hpp"
#include "Utility.hpp"
#include "NumberTypes.hpp"
#include <cstdlib>


class ExpandingPoolAllocator final : public tako::Allocator
{
	struct Node
	{
		Node* next;
	};
public:
	ExpandingPoolAllocator(size_t blockSize) : m_head(nullptr), m_blockSize(blockSize)
	{
	}

	void* Allocate()
	{
		if (m_head == nullptr)
		{
			ExpandMemory();
		}

		Node* p = m_head;
		m_head = p->next;
		m_available--;
		return p;
	}

	virtual void* Allocate(size_t size) override
	{
		ASSERT(m_blockSize == size);

		return Allocate();
	}

	void Deallocate(void* p)
	{
		if (m_available >= 16)
		{
			free(p);
			return;
		}
		Node* node = reinterpret_cast<Node*>(p);
		node->next = m_head;
		m_head = node;
		m_available++;
	}

	virtual void Deallocate(void* p, size_t size) override
	{
		ASSERT(m_blockSize == size);

		Deallocate(p);
	}

private:
	size_t m_blockSize;
	size_t m_available;
	Node* m_head;

	void ExpandMemory()
	{
		size_t totalBlocks = 1;
		tako::U8* p = reinterpret_cast<tako::U8*>(malloc(totalBlocks * m_blockSize));
		for (size_t i = 0; i < totalBlocks; i++)
		{
			Node* n = reinterpret_cast<Node*>(p);
			n->next = m_head;
			m_head = n;
			p += m_blockSize;
		}
		m_available += totalBlocks;
	}
};
