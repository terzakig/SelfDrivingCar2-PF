/*
 * particle_filter.cpp
 *
 *  Created on: April 29th
 *      Author: George Terzakis
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	double std_x = std[0],
	       std_y = std[1],
	       std_theta = std[2];
	
	normal_distribution<double> NormDist_x(x, std_x);
	normal_distribution<double> NormDist_y(y, std_y);
	normal_distribution<double> NormDist_theta(theta, std_theta);
	
	Particle p; // Particle cache
	p.weight = 1.0; // this is fixed for all particles 
	
	for (int i = 0; i < num_particles; i++) {
	  // generate the pose variables independently
	  p.x = NormDist_x(gen_);
	  p.y = NormDist_y(gen_);
	  p.theta = NormDist_theta(gen_);
	  
	  // add the particle to the list
	  particles.push_back(p);
	  
	  //std::cout <<"New particle : "<<particles[particles.size()-1]<<endl;
	  
	  weights.push_back(p.weight);
	 
	}
	
	best_particle_index = 0;
	// set the initialized flag
	is_initialized = true;
}

/// Generate the prediction marginal by sampling the transition conditional distribution(s)
void ParticleFilter::prediction(double Dt, double std_pose[], double velocity, double omega) {
	
	
	double Dt2 = Dt * Dt;
	double std_x = std_pose[0],
	       std_y = std_pose[1],
	       std_theta = std_pose[2];
	       
	// Normal distributions for the stochastic linear-angular accelerations
	// in which case, are actually the transition conditional distributions.
	// NOTE: I am using decorelated accelerations in order to accommodate easy particle weighting.
	normal_distribution<double> NormDist_n_x(0, std_x);
	normal_distribution<double> NormDist_n_y(0, std_y);
	
	normal_distribution<double> NormDist_n_theta(0, std_theta);
	
	
	// Now exaustively transforming N-particles into new particles using the transition/motion model equations
	for (int i = 0; i < num_particles; i++) {
	  
	  // obtain the pose from the particle
	  double  x = particles[i].x,	
		  y = particles[i].y,
		  theta = particles[i].theta;
	  
	  // Now sample the noise variables: 
	  double x_noise = NormDist_n_x(gen_),
		 y_noise = NormDist_n_y(gen_),
		 theta_noise = NormDist_n_theta(gen_);
	  	 
	  
	  // Now compute the predicted pose for this particle:
	  if ( fabs(omega) < 0.00001 ) {
		  
	    // Linear motion transition
	    particles[i].x = x + velocity * Dt * cos(theta) 	+   x_noise;
	    particles[i].y = y + velocity * Dt * sin(theta)  	+   y_noise;
	    particles[i].theta = theta 				+   theta_noise;
	
	  } else {
	    
	    // Rotational motion transition
	    particles[i].x = x + velocity * (  sin(theta + omega * Dt) - sin(theta) ) / omega    +    x_noise;
	    particles[i].y = y + velocity * ( -cos(theta + omega * Dt) + cos(theta) ) / omega    +    y_noise;
	    particles[i].theta = theta + omega * Dt                                              +    theta_noise;
	    
	  }
	 
	  
	} // end for

}



void ParticleFilter::updateWeights(double sensor_range, 
				   double std_landmark[], 
				   std::vector<LandmarkObs> observations, 
				   Map map_landmarks) {
  

  // Going through each particle and transforming the locations of the observations
  // to the coordinate frame of the map
  double std_x = std_landmark[0],
	 std_y = std_landmark[1];
  double sigma2_x = std_x * std_x,
	 sigma2_y = std_y * std_y;
  double inv_sigma2_x = 1.0 / sigma2_x,
	 inv_sigma2_y = 1.0 / sigma2_y;
  
  for (int i = 0; i < num_particles; i++) {
    // The axes of the local coordinate frame in of the particle in the world frame
    // are the columns of the following matrix:
    //     
    // R = [ cosθ , -sinθ;
    //       sin(θ), cos(θ)]
    //
    // Thus, the coordinates of the (locally) observed landmarks 
    // in world reference are:
    //
    // Lw = R * Lp + b
    // 
    // where Lp is the observation in local coordinates and b is the position of the particle
    double log_weight = 0.0; // the weight logarithm of the particle
    double _c = -log(2*M_PI * std_x * std_y); // Logarithm of the global constant multiplying the exponential in the measuremewnt likelihood pdf.
    // loop over the observations
    for (int j = 0; j < observations.size(); j++) {
      
      // get the observed landmark in world coordinates
      double x_t = cos(particles[i].theta) * observations[j].x - sin(particles[i].theta) * observations[j].y + particles[i].x;
      double y_t = sin(particles[i].theta) * observations[j].x + cos(particles[i].theta) * observations[j].y + particles[i].y;
      
      // Now obtaining the nearest landmark in the map
      KD2Point n = map_landmarks.FindNearest(x_t, y_t);
      
      //cout << "Search point : "<<"( "<<x_t<<" , "<<y_t<<" )"<<" - Closest point : "<<n<<endl;
      
      // Now cumulating the logarithm of the measurement likelihood pdf (as product of single-variates due to independence)
      log_weight += _c -0.5 * ( (n.x - x_t) * (n.x - x_t) * inv_sigma2_x + (n.y - y_t) * (n.y - y_t) * inv_sigma2_y  );
      
    } // end observation-for
    
    // so now we obtain the weight from the log-sum
    double weight = exp(log_weight);
    // store the weight in the weight vector
    weights[i] = weight;
    // update the particle weight as well...
    particles[i].weight = weight;
  } // end particle-for
	
}

void ParticleFilter::resample() {
	// NOTE: A cool way of sampling a discrete distribution without sweat! Thanks!
	//     http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
  
	 
	 std::mt19937 gen(rd_());
	 // construct distribution on unnormalized weights
	 std::discrete_distribution<int> posterior( weights.begin(), weights.end() );
	 
	 // sample new particles
	 std::vector<Particle> new_particles;
	 for (int i = 0; i<num_particles; i++) {
	   // sample the posterior
	   int index = posterior(gen);
	   // now store the particle
	   new_particles.push_back(particles[index]);
	 }
	 // finally, update the particles
	 particles = new_particles;

}

void ParticleFilter::write(std::string filename) {
	// You don't need to modify this file.
	std::ofstream dataFile;
	dataFile.open(filename, std::ios::app);
	for (int i = 0; i < num_particles; ++i) {
		dataFile << particles[i].x << " " << particles[i].y << " " << particles[i].theta << "\n";
	}
	dataFile.close();
}
