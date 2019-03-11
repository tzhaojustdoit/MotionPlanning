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
	 *@brief construct a node with id
	 */
	Node(int id);
	
	/**
	*@brief get id
	*/
	int GetId() const;

	/**
	 *@brief get g value
	 */
	double GetG() const;

	/**
	 *@brief get h value
	 */
	double GetH() const;

	/**
	 *@brief get f value
	 */
	double GetF() const;

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
	void SetG(double);

	/**
	 *@brief set h value
	 */
	void SetH(double);

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
	double g_ = 0;
	double h_ = 0;
	double f_ = 0;
	int parent_id_ = -1;
	CellType type_ = DEFAULT;
};
#endif // !NODE_
