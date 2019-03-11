/**
 * @file CollisionChecker.h
 * @brief Defines the CollisionChecker class.
 * @authur: Tianhua Zhao
 */

#ifndef COLLISION_CHECKER_
#define COLLISION_CHECKER_

#include "HelperStructs.h"

/**
 * @brief A class for collision checking using bounding box
 * 
 */
class CollisionChecker
{
private:
    bool intersection(Line_s line1, Line_s line2);
    int order(Line_s line1, Point_s pt);
    bool onSegment(Line_s line1, Point_s pt);
public:
    CollisionChecker(/* args */);
    ~CollisionChecker();
    /**
     * @brief are the two squares(represented in 4 vertexes) in collision?
     * 
     */
    bool BoundingBoxCheck(Point_s, Point_s, Point_s, Point_s, Point_s, Point_s, Point_s, Point_s);
};



#endif // !COLLISION_CHECKER_