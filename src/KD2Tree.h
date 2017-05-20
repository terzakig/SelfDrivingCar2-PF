// **   2D KD Trees
// **
// ** George Terzakis 2017
// **

#ifndef KD2TREE_H
#define KD2TREE_H


#include <iostream>
#include <math.h>
#include <memory>


enum SPLIT_AXIS {X_SPLIT = 0, Y_SPLIT};

struct KD2Point {
 
  double x;
  double y;
  inline KD2Point():x(0), y(0) {}
  inline KD2Point(double _x_, double _y_): x(_x_), y(_y_) {}
  
  
  
  inline KD2Point& operator =(KD2Point p) {
    
    this->x = p.x;
    this->y = p.y;
    
    return *this;
  }
  
  inline double operator[](const SPLIT_AXIS axis) const {
    
    return axis == X_SPLIT ? this->x : this->y;
  }
  
};

inline bool operator ==(KD2Point p1, KD2Point p2) {
 
  return p1.x == p2.x && p1.y == p2.y;
}

inline double Distance(const KD2Point& p1, const KD2Point& p2) {
   
    return sqrt( (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) );
  
}

inline std::ostream& operator <<(std::ostream& os, const KD2Point& p) {
  return os << "( "<<p.x<<" , "<<p.y<<" )";
}

struct KD2TreeNode {

  static constexpr double INF_ = 9999999999.9;
  
  // Split Axis 
  SPLIT_AXIS split_axis;
  
  // The 2D point
  KD2Point point;
  
  // Left and Right branches
  std::shared_ptr<KD2TreeNode> left, right;
  
  inline KD2TreeNode(const KD2Point& p, SPLIT_AXIS spaxis): split_axis(spaxis), point(p)  { left = right = NULL; }
  
  bool Insert(KD2Point p);
 
  void NNSearch(const KD2Point& q, double& min_distance, KD2Point& best_point);
  
  KD2Point FindMin(SPLIT_AXIS coord);
  
  bool Delete(KD2Point q);
  
  // "distance:" between point and KD2Node in terms of the separating line
  inline double Point2NodeDistance(const KD2Point& p) const {
    
    return fabs(p[split_axis] - point[split_axis]);
}

};




















#endif