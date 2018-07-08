/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[])
{
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	
	// Tune this value
	num_particles = 100;

	// Set the correct size of the vectors
	weights.resize(num_particles);
	particles.resize(num_particles);

	// Used for generation of particles
	random_device rand;
	default_random_engine gen(rand());

	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);

	// Particle initialization
	for(int i=0; i<num_particles; i++)
	{
		particles[i].id = i;
		particles[i].x = dist_x(gen);
		particles[i].y = dist_y(gen);
		particles[i].theta = dist_theta(gen);
		particles[i].weight = 1.0;
	}

	// Mark particle filter instance as initialized
	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate)
{
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	
	// Engines for noise
	default_random_engine gen;
	normal_distribution<double> dist_x(0, std_pos[0]);
	normal_distribution<double> dist_y(0, std_pos[1]);
	normal_distribution<double> dist_theta(0, std_pos[2]);

	// Predict motion each article
	for(int i=0; i<num_particles; i++)
	{
		// First apply motion model
		// Use different equations for yaw_rate = 0 and yaw_rate != 0
		if( abs(yaw_rate) != 0 )
		{
			particles[i].x += (velocity/yaw_rate) * (sin(particles[i].theta + (yaw_rate * delta_t)) - sin(particles[i].theta));
			particles[i].y += (velocity/yaw_rate) * (cos(particles[i].theta)  - cos(particles[i].theta + (yaw_rate * delta_t)));
			particles[i].theta += yaw_rate * delta_t;
		}
		else
		{
			particles[i].x += velocity * delta_t * cos(particles[i].theta);
			particles[i].y += velocity * delta_t * sin(particles[i].theta);
			// yaw_rate is zero, so theta does not change
		}

		// Second add noise
		particles[i].x += dist_x(gen);
		particles[i].y += dist_y(gen);
		particles[i].theta += dist_theta(gen);
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks)
{
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

	// Following parts of the multi-variate gaussian probability density are always the same
	const double K = 1 / (2 * M_PI * std_landmark[0] * std_landmark[1]);
	const double x_den = 2 * std_landmark[0] * std_landmark[0];
	const double y_den = 2 * std_landmark[1] * std_landmark[1];

	// Iterate over all particles
	for(int i=0; i<num_particles; i++)
	{
		double P = 1.0; // Resulting multi-variate gaussian probability, calculated for each particle
		
		// Vectors for saving the observed landmarks for each particle
		vector<int> associations;
		vector<double> sense_x, sense_y;

		// For each particle, iterate over all observations
		for(int j=0; j<observations.size(); j++)
		{
			// First transform observation to be relative to the current particle
			// using the homogeneous transformation equations from lesson 14
			double tobs_x = observations[j].x * cos(particles[i].theta) - observations[j].y * sin(particles[i].theta) + particles[i].x;
			double tobs_y = observations[j].x * sin(particles[i].theta) + observations[j].y * cos(particles[i].theta) + particles[i].y;

			// Next we need to figure out which landmark is nearest to this transformed, current observation

			vector<Map::single_landmark_s> landmarks = map_landmarks.landmark_list; // This the map
			// This vector will contain the distance between the current observation and the map landmarks
			vector<double> landmark_distances(landmarks.size()); // Same size as the number of landmarks

			// Calculate the distance between the current observation and each of the landmarks
			for(int k=0; k<landmarks.size(); k++)
			{

				// First calculate the distance between the current particle and the landmark
				double distance_to_particle = dist(particles[i].x, particles[i].y, landmarks[k].x_f, landmarks[k].y_f);

				if(distance_to_particle > sensor_range)
				{
					// The landmark is outside of the sensor range, so no point in calculating precise distance, just use large number
					landmark_distances[k] = 100000.0;
				}
				else
				{
					// The landmark is in range, calculate distance
					landmark_distances[k] = dist(tobs_x, tobs_y, landmarks[k].x_f, landmarks[k].y_f);
				}
			}

			// Now find the position of the landmark with smallest distance to the current observation
			int min_pos = distance(landmark_distances.begin(), min_element(landmark_distances.begin(), landmark_distances.end()));
			float x_nearest_landmark = landmarks[min_pos].x_f;
			float y_nearest_landmark = landmarks[min_pos].y_f;

			associations.push_back(landmarks[min_pos].id_i);
			sense_x.push_back(tobs_x);
			sense_y.push_back(tobs_y);

			// Finally calculate the contribution to the multi-variate gaussian probability from this observation
			// This is the exponent in the multi-variate gaussian probability formula
			double x_diff = tobs_x - x_nearest_landmark;
			double y_diff = tobs_y - y_nearest_landmark;
			double exponent = ((x_diff * x_diff) / x_den) + ((y_diff * y_diff) / y_den);

			// The total multi-variate gaussian probability is the product of all the observations
			P *= K * exp(-exponent);
		}	

		// Update the weigths
		particles[i].weight = P;
		weights[i] = particles[i].weight;

		// Set the associations to get the simulator to display the observations if they were made by the 
		// particle with the highest weight. These should be very close to the real measurements
		// but not 100% the same due to noise.
		particles[i].associations = associations;
		particles[i].sense_x = sense_x;
		particles[i].sense_y = sense_y;
	}
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	vector<Particle> new_particles(num_particles);

	random_device rand;
	default_random_engine gen(rand());

	for(int i=0; i<num_particles; i++)
	{
		discrete_distribution<int> idx(weights.begin(), weights.end());
		new_particles[i] = particles[idx(gen)];
	}

	particles = new_particles;
}

// Comment out to avoid warning when building the project
#if 0
Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
}
#endif

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
