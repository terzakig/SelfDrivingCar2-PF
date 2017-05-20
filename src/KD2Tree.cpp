
// George Terzakis 2017
//
// 2D KD tree inserion, deletion and nearest neighbor search

#include "KD2Tree.h"
#include <vector>
#include <assert.h>


// Find the point with minimum designated coordinate
KD2Point KD2TreeNode::FindMin(SPLIT_AXIS coord) {
  
  if (this->split_axis == coord) {
    
   if (this->left == NULL) return this->point;
   else 
     return this->left->FindMin(coord);
  
    
  }
  else {
  
    KD2Point min_left(INF_, INF_);
    KD2Point min_right(INF_, INF_);
    if (this->left != NULL)  this->left->FindMin(coord);
    if (this->right != NULL) min_right = this->right->FindMin(coord);
    
    double min_coord = std::min( std::min( min_left[coord], min_right[coord] ), this->point[coord] );
    
    if (min_left[coord] == min_coord ) return min_left;
    if (min_right[coord] == min_coord ) return min_right;
    if (this->point[coord] == min_coord ) return this->point;
  }
  
  // Should never be here
  assert(false && "Unreachable region");
  
  return KD2Point(INF_, INF_);
}



// Insert a new point
bool KD2TreeNode::Insert(KD2Point p) {
  
  if (p == point) return false; // duplicate entry attempt
  
  if ( p[this->split_axis] < this->point[this->split_axis] ) { // Inserting on the left
    
    if (this->left != NULL) return this->left->Insert(p); // proceed recursively
    else { // create the left branch and make it a leaf
      
      this->left.reset(new KD2TreeNode(p, this->split_axis == X_SPLIT ? Y_SPLIT : X_SPLIT ) );
      return true;
    }
    
  } else { // Inserting on the right. NOTE: If a point is on the split axis, then it goes right!!!!
    
    if (this->right != NULL) return this->right->Insert(p); // proceed recursively
    else { // create the right branch as a leaf
      
      this->right.reset(new KD2TreeNode(p, this->split_axis == X_SPLIT ? Y_SPLIT : X_SPLIT ) );
      return true;
    }
    
  }
  // Should never reach here...
  return false;
  
}


// Deletion! 
// Note that this function can perform much better with NNSearch(), but hey... 
bool KD2TreeNode::Delete(KD2Point q) {
  
  // if the point is the current node, we cannot delete and exit with error
  if (q == this->point || ( this->left == NULL && this->right == NULL ) ) return false;
  
  for (int i = 0; i < 2; i++) {
    
     std::shared_ptr<KD2TreeNode>& node = i == 0 ? this->right : this->left;
     
     if (node == NULL) continue;
     
     // now we focus on node...
     // suppose that the node is the point we wish to delete...
     if (node->point == q) {
	
       // Case #1: The node is a leaf, so simply setting it to NULL
       if (node->left == NULL && node->right == NULL) {
	 
	 node = NULL;
	 return true;
       }
       
       // Case #2: The right branch exists and we therefore find the substitute from there,
       //          as the point with the minimum coordinate in the split axis of node.
       if (node->right != NULL) {
	  // find the mininimum split-coordinate point on the right branch
	  KD2Point min_point = node->right->FindMin(node->split_axis);
	  // and deleting recursively
	  bool result = node->Delete(min_point);
	  if (result) {
	    // now copying the data
	    node->point = min_point;
	    return true;
	  }
	  else return false;
	  
       }
       // Case 3: The right branch is NULL and we therefore need to to the following trick:
       //         Find the point wiuth the minimum coordinate on the split axis of node and then 
       //         move the entire branch on the right!
       else {
	 // find the mininum split-coordinate point on the left branch
	 KD2Point min_point = node->left->FindMin(node->split_axis);
	 // Delete recursively
	 bool result = node->Delete(min_point);
	 if (result) {
	  
	   // now copy the data
	   node->point = min_point;
	   // And DON'T FORGET! We need to mocve the branch to the right!!!
	   node->right = node->left;
	   node->left = NULL;
	 
	   return true;
	 } 
	 else
	   return false;
	   
       }
     }
     else {
	bool result = node->Delete(q);
	if (result) return true;
     }
       
  }
  
      
  // This is NOT unreachable region, but should always return false
  // if control has come to this point...
  return false;
      
}


// Nearest neighbor
void KD2TreeNode::NNSearch(const KD2Point& q, double& min_distance, KD2Point& best_point)
{


  // distance to the data of the node
  double dist = Distance(this->point, q);
  // update if this is a good distance
  if (dist < min_distance) {
   
    min_distance = dist;
    best_point = this->point;
    if (dist == 0) return; // exact point match, return
  }
  

  // Now (maybe) going through the left-right branches
  // CASE #1 : min_dist > distance-2-separator (split axis)
  //           This means we need to check both sides. 
  double dist2barrier = Point2NodeDistance(q);
  if (dist2barrier < min_distance ) {
   
    if (this->left != NULL) this->left->NNSearch(q, min_distance, best_point);
    if (this->right != NULL) this->right->NNSearch(q, min_distance, best_point);
      
  } else { // Need to search the side we or on ONLY
    
    if (q[this->split_axis] < this->point[this->split_axis]) { // we are on the left
	
      if (this->left != NULL) this->left->NNSearch(q, min_distance, best_point);
    
    } else { // We are on the right
      
	if (this->right != NULL) this->right->NNSearch(q, min_distance, best_point);
    
    }
    
  }
  
  
}