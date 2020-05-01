#pragma once

#include <vector>
#include <Eigen/Dense>

class BVHCollisionDetector;

namespace Lobo
{


class CollisionWorld
{

private:
    /* data */
public:
    CollisionWorld(/* args */);
    ~CollisionWorld();

    virtual void doDetection();

    virtual void addCollisionDetector(BVHCollisionDetector* collisionobject);
    virtual void addCollisionPair(int idx,int idy);
    virtual void clearCollisionPair();

    bool all_pairs;

protected:  

    std::vector<BVHCollisionDetector*> collisionObjectList;
	std::vector<Eigen::Vector2i> collision_pair;

};


} // namespace Lobo