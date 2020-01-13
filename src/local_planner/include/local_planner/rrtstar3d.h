#ifndef RRTSTAR3D_H
#define RRTSTAR3D_H

#include <Eigen/Eigen>
#include <visualization_msgs/Marker.h>
#include <ewok/ed_ring_buffer.h>
#include <ewok/raycast_ring_buffer.h>
#include <algorithm>
#include <vector>
#include <iostream>
#include <memory>
#include <random>

template<int _POW, int _N, typename _Datatype = int16_t,
    typename _Scalar = double, typename _Flag = uint8_t>
class RRTStar3D
{
public:
    static const int N = (1 << _POW);
    static const int DEG = N - 1;

    static const int OFFSET = N / 2 - 1;

    typedef std::shared_ptr <RRTStar3D<_POW, _N, _Datatype, _Scalar, _Flag> > Ptr;

    typedef Eigen::Matrix<_Scalar, 3, 1> Vector3;
    typedef Eigen::Matrix<int, 3, 1> Vector3i;

    struct Node{

        std::vector<Node *> children_;
        Node *parent_ = NULL;
        Vector3 pos_;
        double cost_;
    };

    RRTStar3D(const _Scalar &step_size, const _Scalar &rrt_factor)
    {

        step_size_=step_size;
        rrt_factor_ = rrt_factor;
        nodes_.clear();
    }


    void setStartPoint(Vector3 start)
    {
        start_=start;
    }

    void setTargetPoint(Vector3 target)
    {
        target_=target;
    }

    _Scalar getCost(Node* n)
    {
        return n->cost_;
    }

    _Scalar getDistCost(Node* p, Node* q)
    {

    }

    _Scalar distance(Vector3 &p1, Vector3 &p2)
    {

    }

    bool isNear(Vector3 &point, _Scalar tol=10)
    {

    }

    Node* getRandomNode()
    {
        Vector3 point_min, point_max, rand_point;
        Vector3i point_idx;
        occupancy_buffer_.getVolumeMinMax(point_min, point_max);
        _Scalar x_max = point_max.x()/ occupancy_buffer_.resolution_;
        _Scalar y_max = point_max.y()/ occupancy_buffer_.resolution_;
        _Scalar z_max = point_max.z()/ occupancy_buffer_.resolution_;
        do{
        rand_point = Vector3(rand()%x_max*occupancy_buffer_.resolution_,
                                         rand()%y_max*occupancy_buffer_.resolution_,
                                         rand()%z_max*occupancy_buffer_.resolution_);
        occupancy_buffer_.getIdx(rand_point, point_idx);

        }while(occupancy_buffer_.isOccupied(point_idx));
        Node *rand_node = new Node;
        rand_node->position = rand_point;
        return rand_node;
    }

    Node* getNearestNode(Node* node_)
    {
        Vector3i min_vec, max_vec;
//        occupancy_buffer_.

//        double minDist = 1e9;
//        Node* closest = NULL;

//        for(int i=0; i<(int)nodes.size(); i++)
//        {
//            double dist = distance(node_->position_, nodes[i]->position_);
//            if(dist < minDist)
//            {
//                minDist = dist;
//                closest = nodes[i];
//            }
//        }
//        return closest;
    }


    void getNearestNodes(Node *node, _Scalar radius, std::vector<Node *> &near)
    {
//        for(auto n: nodes)
//        {
//            double dist = distance(node->position_, n->position_);
//            if(dist < radius*rrt_factor)
//                near.push_back(n);
//        }
    }

    Node* getConfigurationNode(Node* q, Node* nearest_)
    {
//        Vector2d q_pos = q->position_;
//        Vector2d near_pos = nearest_->position_;
//        Vector2d midPos = q_pos-near_pos;
//        midPos = midPos/midPos.norm();

//        Node* node_ = new Node;
//        node_->position_ = near_pos+step_size*midPos;
//        return node_;
    }

    void findPath()
    {
        solve();
    }

    void solve(const int iter=10000)
    {
//        int counter=0;
//        Node* final;
//        bool found = false;
//        while(counter<N)
//        {
//            Node* rand_node = getRandomNode();
//            if(rand_node)
//            {
//                Node* nearest_node = getNearestNode(rand_node);
//                Node* new_node = getConfigurationNode(rand_node, nearest_node);
//                if(!isCollision(new_node, nearest_node))
//                {
//                    std::vector<Node*> near_nodes;
//                    getNearestNodes(new_node, step_size,near_nodes);

//                    Node* min_node = nearest_node;
//                    double min_cost = getCost(nearest_node) + getDistCost(nearest_node, new_node);
//                    for(auto p: near_nodes)
//                    {
//                        if(!isCollision(p, new_node) && (getCost(p) + getDistCost(p, new_node)) < min_cost)
//                        {
//                            min_node = p; min_cost=getCost(p) + getDistCost(p, new_node);
//                        }
//                    }

//                    new_node->parent_=min_node;
//                    new_node->cost_ = min_node->cost_ + getDistCost(min_node, new_node);
//                    min_node->children_.push_back(new_node);
//                    edges.push_back(std::make_tuple(min_node->position_, new_node->position_, false));
//                    nodes.push_back(new_node);
//                    lastNode = new_node;

//                    for(auto p: near_nodes)
//                    {
//                        if(!isCollision(new_node,p) && (getCost(new_node) + getDistCost(new_node, p)) < p->cost_)
//                        {
//                            Node* n_parent = p->parent_;
//                            n_parent->children_.erase(std::remove(n_parent->children_.begin(), n_parent->children_.end(), p), n_parent->children_.end());
//                            edges.erase(std::remove(edges.begin(), edges.end(), std::make_tuple(n_parent->position_, new_node->position_, false)), edges.end());
//                            p->cost_=getCost(new_node)+getDistCost(new_node, p);
//                            p->parent_ = new_node;
//                            new_node->children_.push_back(p);
//                            edges.push_back(std::make_tuple(p->position_, new_node->position_, false));

//                        }
//                    }

//                }
//                else
//                    edges.push_back(std::make_tuple(nearest_node->position_, new_node->position_, true));

//            }

//            if(isNear(lastNode->position_, 25))
//            {
//                final = lastNode;
//                found = true;
//                break;
//            }

//            counter++;
//        }

//        if(found)
//            path_point.push_back(target);

//        while(final != NULL)
//        {
//            Vector2d pos = final->position_;
//                path_point.push_back(pos);
//                final = final->parent_;

//        }


    }



protected:

    ewok::EuclideanDistanceRingBuffer<6>::Ptr edrb_;
    ewok::RaycastRingBuffer <_POW, _Datatype, _Scalar, _Flag> occupancy_buffer_;

    Vector3 start_, target_;
    _Scalar step_size_;
    std::vector<Node *> nodes_;
    Node *root_, *lastNode_;
    _Scalar rrt_factor_;

};


#endif // RRTSTAR3D_H
