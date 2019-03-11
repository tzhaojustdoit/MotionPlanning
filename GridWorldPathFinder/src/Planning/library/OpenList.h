/**
 * @file OpenList.h
 * @brief Defines the OpenList class.
 * @authur: Tianhua Zhao
 */

#ifndef OPEN_LIST
#define OPEN_LIST

#include <vector>

#include "Node.h"

/*
  *@class OpenList
  *
  *@brief A priority queue of node pointers. Min element is always on top and poped first.
  */
class OpenList
{
  public:
	OpenList() = default;
	OpenList(int capacity);
	/**
	 * @brief insert a node pointer
	 */
	void push(Node *);

	/**
	 * @brief remove top node pointer
	 */
	Node *pop();

	/**
	 * @brief access top node pointer
	 */
	Node *top() const;

	/**
	 * @brief test whether container is empty
	 */
	bool empty() const;

	/**
	 * @brief return size
	 */
	int size() const;

	/**
	 * @brief test whether a node pointer is in the priority queue
	 */
	bool contains(Node *) const;

	/**
	 * @brief reset the type of each node in the priority queue to DEFAULT
	 */
	void reset_type();

	/**
	 * @brief move the node to the correct location since the value decreased.
	 */
	void decrease_key(Node *node);

  private:
	/**
	 * @brief the underlying container
	 */
	std::vector<Node *> vec_;

	/**
	 * @brief perculate a node* up to the correct location
	 */
	void perculate_up(int);

	/**
	 * @brief perculate a node* down to the correct location
	 */
	void perculate_down(int);
};
#endif // !OPEN_LIST
