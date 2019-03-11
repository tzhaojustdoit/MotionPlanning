#include "lib/Node.h"

Node::Node()
{
}

Node::Node(int id): id_(id)
{
}

int Node::GetId() const
{
	return id_;
}

double Node::GetG() const
{
	return g_;
}

double Node::GetH() const
{
	return h_;
}

double Node::GetF() const
{
	return f_;
}

int Node::GetParentId() const
{
	return parent_id_;
}

CellType Node::GetType() const
{
	return type_;
}

void Node::SetId(int id)
{
	id_ = id;
}

void Node::SetG(double g)
{
	g_ = g;
	f_ = g_ + h_;
}

void Node::SetH(double h)
{
	h_ = h;
	f_ = g_ + h_;
}

void Node::SetParentId(int id)
{
	parent_id_ = id;
}

void Node::SetType(CellType type)
{
	type_ = type;
}

int Node::GetMpId(){
	return mp_id_;
}

void Node::SetMpId(int mpId){
	mp_id_ = mpId;
}

bool Node::operator<(const Node &other)
{
	if (f_ < other.f_)
	{
		return true;
	}
	if (f_ > other.f_)
	{
		return false;
	}
	if (g_ > other.g_)
	{ // if f is a tie, break tie in favor of larger g
		return true;
	}
	if (g_ < other.g_)
	{
		return false;
	}
	return id_ < other.id_; // if both f and g are tie, break tie in favor of smaller id
}