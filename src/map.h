/*
 * map.h
 *

 George

*/

#ifndef MAP_H_
#define MAP_H_

#include "KD2Tree.h"

class Map {
public:
	
	struct single_landmark_s{

		int id_i; // Landmark ID
		float x_f; // Landmark x-position in the map (global coordinates)
		float y_f; // Landmark y-position in the map (global coordinates)
	};


	// The KD tree to store the landmark coordinates for fast searches
	std::shared_ptr<KD2TreeNode> kd_tree;
	
	
	
	
	std::vector<single_landmark_s> landmark_list; // List of landmarks in the map

	inline Map():kd_tree(NULL) {}
	
	inline bool InsertLandmark(const single_landmark_s& l) {
	  
	  // add it to the list
	  landmark_list.push_back(l);
	  
	  // now insert it in the 2D KD tree
	  KD2Point point( l.x_f, l.y_f );
	  if (kd_tree == NULL) 
	    kd_tree.reset<KD2TreeNode>(new KD2TreeNode(point, X_SPLIT) ); // using the x-axis for the first split
	  else
	    kd_tree->Insert(point);
	}
	// and the nearest pair of coordinates
	inline KD2Point FindNearest(double x, double y) {
	    
	  KD2Point q(x, y); // query point
	  KD2Point best;
	  double min_distance = KD2TreeNode::INF_;
	  kd_tree->NNSearch(q, min_distance, best);
	  
	  return best;
	    
	}
};



#endif /* MAP_H_ */
