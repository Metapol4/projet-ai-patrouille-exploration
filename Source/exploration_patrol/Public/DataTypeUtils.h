#pragma once
#include "CoreMinimal.h"

template <typename InElementType>
struct TPriorityQueueNode
{
	InElementType Element;
	float Priority;

	TPriorityQueueNode() { Priority = 0;}
	TPriorityQueueNode(InElementType InElement, float InPriority)
	{
		Element = InElement;
		Priority = InPriority;
	}

	bool operator<(const TPriorityQueueNode<InElementType> Other) const
	{
		return Priority <Other.Priority;
	}
};

template <typename InElementType>
class TPriorityQueue
{
public:
	TPriorityQueue()
	{
		Array.Heapify();
	}
public:
	InElementType Pop()
	{
		TPriorityQueueNode<InElementType> Node;
		Array.HeapPop(Node);
		return Node.Element;
	}
	TPriorityQueueNode<InElementType> PopNode()
	{
		TPriorityQueueNode<InElementType> Node;
		Array.HeapPop(Node);
		return Node;
	}
	void Push(InElementType Element, float Priority)
	{
		Array.HeapPush(TPriorityQueueNode<InElementType>(Element, Priority));
	}
	bool IsEmpty() const
	{
		return Array.Num() == 0;
	}
private:
	TArray<TPriorityQueueNode<InElementType>> Array;
};