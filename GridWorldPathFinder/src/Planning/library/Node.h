/**
 * @file Node.h
 * @brief Defines the Node class.
 * @authur: Tianhua Zhao
 */

#ifndef NODE_
#define NODE_

#include <vector>

#include "CellType.h"

/**
 * @class Node
 *
 * @brief A node in a graph.
 * @invariants: f = g + h
 */
class Node
{
  public:
	/**
	 *@brief construct a node 
	 */
	Node();

	/**
	*@brief get id
	*/
	int GetId() const;

	/**
	 *@brief get g value
	 */
	int GetG() const;

	/**
	 *@brief get h value
	 */
	int GetH() const;

	/**
	 *@brief get f value
	 */
	int GetF() const;

	/**
	 *@brief get parent id
	 */
	int GetParentId() const;

	/**
	 *@brief get the cell type, i.e., in open list or closed list or neither
	 */
	CellType GetType() const;

	/**
	*@brief set id
	*/
	void SetId(int);

	/**
	 *@brief set g value
	 */
	void SetG(int);

	/**
	 *@brief set h value
	 */
	void SetH(int);

	/**
	 *@brief set parent id
	 */
	void SetParentId(int);

	/**
	 *@brief set the cell type, i.e., in open list or closed list or neither
	 */
	void SetType(CellType type);

	/**
	*@brief overloaded < operator
	*/
	bool operator<(const Node &);

  private:
	int id_ = -1;
	int g_ = 0;
	int h_ = 0;
	int f_ = 0;
	int parent_id_ = -1;
	CellType type_ = DEFAULT;
};
#endif // !NODE_
