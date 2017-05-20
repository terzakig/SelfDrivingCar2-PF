/*
 * particle_filter.h
 *
 * 
 * George Terzakis
 * 
 * 
 */

#ifndef PARTICLE_FILTER_H_
#define PARTICLE_FILTER_H_

#include "helper_functions.h"

struct Particle {

	//int id;
	double x;
	double y;
	double theta;
	double weight;
	
	
};

// write a particle to an output stream
inline std::ostream& operator << (std::ostream& os, const Particle& p) {
	 
	  return os << " Pose : "<<"( "<<p.x<<" , "<<p.y<<" , "<<p.theta<<" )"<<" - Weight : "<<p.weight;
}

class ParticleFilter {
	
	// Number of particles to draw
	int num_particles; 
	
	// The best particle index
	int best_particle_index;
	
	// Flag, if filter is initialized
	bool is_initialized;
	
	// Cumulative sum of particle weights
	std::vector<double> weights;
	
	std::default_random_engine gen_;
	std::random_device rd_;
	
public:
	
	// Set of current particles
	std::vector<Particle> particles;

	// Constructor
	// @param M Number of particles
	ParticleFilter(int n_particles = 100) : num_particles(n_particles), is_initialized(false) {}

	// Destructor
	~ParticleFilter() {}

	
	/**
	 * init Initializes particle filter by initializing particles to Gaussian
	 *   distribution around first position and all the weights to 1.
	 * @param x Initial x position [m] (simulated estimate from GPS)
	 * @param y Initial y position [m]
	 * @param theta Initial orientation [rad]
	 * @param std[] Array of dimension 3 [standard deviation of x [m], standard deviation of y [m]
	 *   standard deviation of yaw [rad]]
	 */
	void init(double x, double y, double theta, double std[]);

	/**
	 * prediction Predicts the state for the next time step
	 *   using the process model.
	 * @param Dt Time between time step t and t+1 in measurements [s]
	 * @param std_pos[] Array of dimension 3 [standard deviation of x [m], standard deviation of y [m]
	 *   standard deviation of yaw [rad]]
	 * @param velocity Velocity of car from t to t+1 [m/s]
	 * @param omega Yaw rate of car from t to t+1 [rad/s]
	 */
	void prediction(double Dt, double std_pose[], double velocity, double omega);
	
	
	/**
	 * updateWeights Updates the weights for each particle based on the likelihood of the 
	 *   observed measurements. 
	 * @param sensor_range Range [m] of sensor
	 * @param std_landmark[] Array of dimension 2 [standard deviation of range [m],
	 *   standard deviation of bearing [rad]]
	 * @param observations Vector of landmark observations
	 * @param map Map class containing map landmarks
	 */
	void updateWeights(double sensor_range, 
			   double std_landmark[], 
			   std::vector<LandmarkObs> observations,
			   Map map_landmarks);
	
	/**
	 * resample Resamples from the updated set of particles to form
	 *   the new set of particles.
	 */
	void resample();
	
	/*
	 * write Writes particle positions to a file.
	 * @param filename File to write particle positions to.
	 */
	void write(std::string filename);
	
	/**
	 * initialized Returns whether particle filter is initialized yet or not.
	 */
	const bool initialized() const {
		return is_initialized;
	}
};



#endif /* PARTICLE_FILTER_H_ */
