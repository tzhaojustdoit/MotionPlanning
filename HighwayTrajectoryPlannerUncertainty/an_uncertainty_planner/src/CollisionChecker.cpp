#include "lib/CollisionChecker.h"
CollisionChecker::CollisionChecker(/* args */)
{
}

CollisionChecker::~CollisionChecker()
{
}

bool CollisionChecker::BoundingBoxCheck(Point_s a1, Point_s a2, Point_s a3, Point_s a4, Point_s b1, Point_s b2, Point_s b3, Point_s b4){
    std::vector<Line_s> a;
    a.emplace_back(Line_s{a1, a2});
    a.emplace_back(Line_s{a1, a3});
    a.emplace_back(Line_s{a2, a4});
    a.emplace_back(Line_s{a3, a4});
    std::vector<Line_s> b;
    b.emplace_back(Line_s{b1, b2});
    b.emplace_back(Line_s{b1, b3});
    b.emplace_back(Line_s{b2, b4});
    b.emplace_back(Line_s{b3, b4});
    for (Line_s line1 : a){
        for (Line_s line2 : b){
            if(intersection(line1, line2)){
                return true;
            }
        }
    }
    return false;
}

bool CollisionChecker::intersection(Line_s line1, Line_s line2){
    int order1 = order(line1, line2.a);
    int order2 = order(line1, line2.b);
    int order3 = order(line2, line1.a);
    int order4 = order(line2, line1.b);
    if(order1 != order2 && order3 != order4){
        return true;
    }
    if(order1 == 0 && onSegment(line1, line2.a)){
        return true;
    }
    if(order2 == 0 && onSegment(line1, line2.b)){
        return true;
    }
    if(order3 == 0 && onSegment(line2, line1.a)){
        return true;
    }
    if(order4 == 0 && onSegment(line2, line1.b)){
        return true;
    }
    return false;
}
int CollisionChecker::order(Line_s line1, Point_s pt){
    double orientation = (line1.b.y - line1.a.y) * (pt.x - line1.b.x)
        - (line1.b.x - line1.a.x) * (pt.y - line1.b.y);
    if (orientation == 0){
        return 0;
    }
    if (orientation > 0){
        return 1;
    } 
    return -1;
}
bool CollisionChecker::onSegment(Line_s line1, Point_s pt){
    if((pt.x <= std::max(line1.a.x, line1.b.x)) 
    &&(pt.x >= std::min(line1.a.x, line1.b.x))
    &&(pt.y <= std::max(line1.a.y, line1.b.y))
    &&(pt.y >= std::min(line1.a.y, line1.b.y))){
        return true;
    } 
    return false;
}