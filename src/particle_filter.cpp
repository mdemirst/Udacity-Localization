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

std::vector<LandmarkObs> ParticleFilter::transformLandmarks(Particle particle, std::vector<LandmarkObs> observations)
{
    std::vector<LandmarkObs> transformed;
    for(int l = 0; l < observations.size(); ++l)
    {
        double x = observations[l].x*cos(particle.theta) - observations[l].y*sin(particle.theta);
        double y = observations[l].x*sin(particle.theta) + observations[l].y*cos(particle.theta);

        x = particle.x + x;
        y = particle.y + y;

        LandmarkObs landmark;
        landmark.x = x;
        landmark.y = y;
        landmark.id = observations[l].id;

        transformed.push_back(landmark);
    }

    return transformed;
}

std::vector<LandmarkObs> ParticleFilter::invTransformLandmarks(Particle particle, std::vector<LandmarkObs> observations)
{
    std::vector<LandmarkObs> transformed;
    for(int l = 0; l < observations.size(); ++l)
    {
        double x = observations[l].x * cos(particle.theta) + observations[l].y * sin(particle.theta) -
                particle.x * cos(particle.theta) - particle.y * sin(particle.theta);

        double y = -observations[l].x * sin(particle.theta) + observations[l].y * cos(particle.theta) +
                particle.x * sin(particle.theta) - particle.y * cos(particle.theta);

        LandmarkObs landmark;
        landmark.x = x;
        landmark.y = y;
        landmark.id = observations[l].id;

        transformed.push_back(landmark);
    }

    return transformed;
}


void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

    num_particles = 250;

    default_random_engine gen;
    normal_distribution<double> dist_x(x, std[0]);
    normal_distribution<double> dist_y(y, std[1]);
    normal_distribution<double> dist_theta(theta, std[2]);


    particles.clear();

    for (int i = 0; i < num_particles; ++i)
    {
        Particle new_particle;
        new_particle.id = i;
        new_particle.x = dist_x(gen);
        new_particle.y = dist_y(gen);
        new_particle.theta = dist_theta(gen);
        new_particle.weight = 1.0;

        particles.push_back(new_particle);
    }

    is_initialized = true;

    cout << "Particles initialized" << endl;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate)
{
    default_random_engine gen;

    // TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

    for(int p = 0; p < particles.size(); ++p)
    {
        double theta = particles[p].theta + yaw_rate*delta_t;


        normal_distribution<double> dist_theta(theta, std_pos[2]);

        theta = dist_theta(gen);

        double x = 0;
        double y = 0;

        if(yaw_rate == 0.0)
        {
            x = particles[p].x;
            y = particles[p].y;
        }
        else
        {
            x = particles[p].x + (velocity/yaw_rate)*(sin(particles[p].theta + yaw_rate*delta_t) - sin(particles[p].theta));
            y = particles[p].y + (velocity/yaw_rate)*(cos(particles[p].theta) - cos(particles[p].theta + yaw_rate*delta_t));
        }


        normal_distribution<double> dist_x(x, std_pos[0]);
        normal_distribution<double> dist_y(y, std_pos[1]);

        x = dist_x(gen);
        y = dist_y(gen);

        particles[p].x = x;
        particles[p].y = y;

        particles[p].theta = theta;
    }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

    for(int obs_ind = 0; obs_ind < observations.size(); ++obs_ind)
    {
        double min_dist = dist(observations[obs_ind].x, observations[obs_ind].y, predicted[0].x, predicted[0].y);
        int min_dist_id = 0;

        for(int pred_ind = 0; pred_ind < predicted.size(); ++pred_ind)
        {
            double cur_dist = dist(observations[obs_ind].x, observations[obs_ind].y, predicted[pred_ind].x, predicted[pred_ind].y);
            if(cur_dist < min_dist)
            {
                min_dist = cur_dist;
                min_dist_id = pred_ind;
            }
        }

        observations[obs_ind] = predicted[min_dist_id];
    }

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {

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

    vector<LandmarkObs> landmarks;
    for(int i = 0; i < map_landmarks.landmark_list.size(); ++i)
    {
        LandmarkObs landmark;
        landmark.id = map_landmarks.landmark_list[i].id_i;
        landmark.x = map_landmarks.landmark_list[i].x_f;
        landmark.y = map_landmarks.landmark_list[i].y_f;

        landmarks.push_back(landmark);
    }

    for(int p = 0; p < particles.size(); p++)
    {
        particles[p].associations.clear();
        particles[p].sense_x.clear();
        particles[p].sense_y.clear();

        vector<LandmarkObs> predicted_landmarks = invTransformLandmarks(particles[p], landmarks);
        vector<LandmarkObs> transformed_observations = transformLandmarks(particles[p], observations);
        vector<LandmarkObs> closest_landmarks(observations);

        dataAssociation(predicted_landmarks, closest_landmarks);


        double weight = 0.0;
        for(int i = 0; i < closest_landmarks.size(); i++)
        {
            double obs_range = get_range(observations[i].x, observations[i].y);

            if(obs_range < sensor_range)
            {
                double obs_angle = get_bearing(observations[i].x, observations[i].y);

                if(obs_angle < 0)
                    obs_angle += 2*M_PI;

                double pred_range = get_range(closest_landmarks[i].x, closest_landmarks[i].y);
                double pred_angle = get_bearing(closest_landmarks[i].x, closest_landmarks[i].y);

                if(pred_angle < 0)
                    pred_angle += 2*M_PI;

                weight += multiVariatePdf(obs_range, obs_angle, pred_range, pred_angle, std_landmark[0], std_landmark[1]);
            }

            particles[p].sense_x.push_back(transformed_observations[i].x);
            particles[p].sense_y.push_back(transformed_observations[i].y);
            particles[p].associations.push_back(closest_landmarks[i].id);
        }

        particles[p].weight = weight;

    }



}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

    std::vector<double> weights(num_particles);
    for (int i = 0; i < num_particles; ++i)
    {
        weights[i] = particles[i].weight;
    }

    std::default_random_engine gen;
    discrete_distribution<> dist(weights.begin(), weights.end());

    vector<Particle> new_particles;
    for(int i = 0; i < num_particles; ++i)
    {
        int rand_ind = dist(gen);
        Particle new_particle = particles[rand_ind];

        new_particles.push_back(new_particle);
    }

    particles = new_particles;

}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations= associations;
 	particle.sense_x = sense_x;
 	particle.sense_y = sense_y;

 	return particle;
}

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
