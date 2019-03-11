#include "AStarPlanning.h"

AStarPlanning::AStarPlanning(int rows, int cols) : rows_(rows), cols_(cols)
{
	graph_.resize(rows * cols);
	for (unsigned i = 0; i < graph_.size(); i++)
	{
		graph_[i].SetId(i);
	}
}

AStarPlanning::~AStarPlanning()
{
}

void AStarPlanning::SetGoal(int goal)
{
	goal_location_ = goal;
	num_of_expanded_nodes_ = 0;
	num_of_searches_ = 0;
	// pre-compute heuristics for each node
	for (unsigned i = 0; i < graph_.size(); i++)
	{
		int h = GetHeuristic(i, goal);
		graph_[i].SetH(h);
	}
}

std::vector<int> AStarPlanning::FindPath(const std::vector<bool> &obstacles, int location)
{
	if (goal_location_ == -1)
	{
		std::cerr << "[planning] Goal location is not set." << std::endl;
		return std::vector<int>();
	}
	num_of_searches_++;
	std::cout << std::endl
			  << "[planning] Planning with A* , id: " << num_of_searches_ << "." << std::endl;
	// open list, contains generated nodes
	OpenList open(obstacles.size());
	// closed list, contains expanded nodes
	std::vector<Node *> closed;
	closed.reserve(obstacles.size());

	// path points from the current location(exclusive) to the goal location(exclusive)
	std::vector<int> path;
	path.reserve(obstacles.size());

	Node *current_node = &graph_[location];
	// set g value
	current_node->SetG(0);
	// set parent id
	current_node->SetParentId(-2);
	// add to the open list
	open.push(current_node);
	current_node->SetType(OPEN);
	while (!open.empty())
	{
		// get the node with the min f value.
		current_node = open.top();
		if (current_node->GetId() == goal_location_)
		{
			// reset type for each node in open and closed list
			for (Node *var : closed)
			{
				var->SetType(DEFAULT);
			}
			open.reset_type();
			// make path points from the current location(exclusive) to the goal location(exclusive)
			while (current_node->GetParentId() != -2)
			{
				path.push_back(current_node->GetId());
				current_node = &graph_[current_node->GetParentId()];
			}
			break;
		}
		// remove from the open list
		open.pop();
		// add the node to the closed list
		closed.push_back(current_node);
		current_node->SetType(CLOSED);
		// expand the node
		Expand(current_node->GetId(), obstacles, closed, open);
	}
	if (path.empty())
	{
		std::cout << std::endl
				  << "[planning] No path is found." << std::endl;
	}
	else
	{
		std::cout << std::endl
				  << "[planning] Successfully found a path." << std::endl;
	}
	Display::DisplayMap(rows_, cols_, obstacles, path, location, goal_location_);
	return path;
}

int AStarPlanning::GetNumOfSearches() const
{
	return num_of_searches_;
}

int AStarPlanning::GetNumOfNodesExpanded() const
{
	return num_of_expanded_nodes_;
}

int AStarPlanning::GetHeuristic(int a, int b)
{
	int ax = a / cols_;
	int ay = a % cols_;
	int bx = b / cols_;
	int by = b % cols_;
	// manhattan distance
	return std::abs(ax - bx) + std::abs(ay - by);
}

void AStarPlanning::Expand(int id, const std::vector<bool> &obstacles, std::vector<Node *> &closed, OpenList &open)
{
	num_of_expanded_nodes_++;
	// check traverability of 4 adjacent cells in this order: right, down, left, up
	// if traversable, generate the successor node
	if ((id + 1) % cols_ != 0 /* boundary check */ && !obstacles[id + 1] /* collision check */)
	{
		Generate(id + 1, id, closed, open);
	}
	if (id < (rows_ - 1) * cols_ && !obstacles[id + cols_])
	{
		Generate(id + cols_, id, closed, open);
	}
	if (id % cols_ != 0 && !obstacles[id - 1])
	{
		Generate(id - 1, id, closed, open);
	}
	if (id >= cols_ && !obstacles[id - cols_])
	{
		Generate(id - cols_, id, closed, open);
	}
}

void AStarPlanning::Generate(int id, int parent_id, std::vector<Node *> &closed, OpenList &open)
{
	Node &current_node = graph_[id];
	int g = graph_[parent_id].GetG() + 1;
	if (current_node.GetType() == DEFAULT)
	{
		current_node.SetG(g);
		current_node.SetParentId(parent_id);
		open.push(&current_node);
		current_node.SetType(OPEN);
	}
	else if (current_node.GetType() == OPEN)
	{
		if (g < current_node.GetG())
		{
			current_node.SetG(g);
			// update the node position in the open list
			open.decrease_key(&current_node);
			current_node.SetParentId(parent_id);
		}
	}
}
