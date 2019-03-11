#include "OpenList.h"

OpenList::OpenList(int capacity)
{
	vec_.reserve(capacity);
}

void OpenList::push(Node *n)
{
	vec_.push_back(n);
	perculate_up(vec_.size() - 1);
}

Node *OpenList::pop()
{
	Node *res = vec_.front();
	std::swap(vec_.front(), vec_.back());
	vec_.pop_back();
	perculate_down(0);
	return res;
}

Node *OpenList::top() const
{
	return vec_[0];
}

bool OpenList::empty() const
{
	return vec_.size() == 0;
}

int OpenList::size() const
{
	return vec_.size();
}

bool OpenList::contains(Node *target) const
{
	for (Node *var : vec_)
	{
		if (var == target)
		{
			return true;
		}
	}
	return false;
}

void OpenList::perculate_up(int idx)
{
	while (idx > 0 && *vec_[idx] < *vec_[(idx - 1) / 2])
	{
		std::swap(vec_[idx], vec_[(idx - 1) / 2]);
		idx = (idx - 1) / 2;
	}
}

void OpenList::perculate_down(int idx)
{
	while (idx * 2 + 1 < (int)vec_.size())
	{
		int minChildIdx = idx * 2 + 1;
		if (idx * 2 + 2 < (int)vec_.size() && *vec_[idx * 2 + 2] < *vec_[idx * 2 + 1])
		{
			minChildIdx = idx * 2 + 2;
		}
		if (*vec_[minChildIdx] < *vec_[idx])
		{
			std::swap(vec_[idx], vec_[minChildIdx]);
		}
		idx = minChildIdx;
	}
}

void OpenList::reset_type()
{
	for (Node *var : vec_)
	{
		var->SetType(DEFAULT);
	}
}

void OpenList::decrease_key(Node *node)
{
	for (unsigned i = 0; i < vec_.size(); i++)
	{
		if (vec_[i] == node)
		{
			perculate_up(i);
		}
	}
}
