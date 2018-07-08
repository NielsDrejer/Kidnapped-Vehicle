## Udacity Self-Driving Car Engineer Nanodegree, Term 2, Project 3: Kidnapped Vehicle

---

The goal of this project is to implement a 2 dimensional particle filter in C++.

The particle filter is exercised with the term 2 simulator application and is through that given a map and some initial localization information (analogous to what a GPS would provide). Furthermore, at each time step the filter is also fed observation and control data.

As usual seed code for the project is provided, and the task is to implement a set of empty functions. Seed code is taken from here:

https://github.com/udacity/CarND-Kidnapped-Vehicle-Project

[//]: # (Image References)
[image1]: ./writeup_images/passed.png

## Notes about Installation

I am using a Mac for development.

The uWebSockets were already installed in connection with project 1 of term 2. So I could directly execute the instructions:

1. mkdir build
2. cd build
3. cmake ..
4. make all

As in the previous projects of term 2 I had to modify the generated CMakeLists.txt to use the following line to enable make to find the linker:

link_directories(/usr/local/Cellar/libuv/1.20.3/lib)

The original line referred to the folder /usr/local/Cellar/libuv/1*/lib, and that does not work on my system.

After that everything worked as expected. I used the Sublime text editor and make on the command line to build.

## Notes about Implementation

I used the provided file structure and implemented the following functions in the file particle_filter.cpp:

1. init()

2. prediction()

3. updateWeights()

4. resample()

I did not use the function SetAssociations(). Instead I removed it from particle_filter.cpp to remove the annoying warning it produced.

I ended up using 100 particles. I read in the Slack channel for the project that the optimal number of particles is between 50 and 100.

## Rubric Points

---

#### 1. Does your particle filter localize the vehicle to within the desired accuracy?  

The simulator application says so:

![alt text][image1]

#### 2. Does your particle run within the specified time of 100 seconds?

It runs in around 50 seconds on my system (MacMini). This depends slightly upon the number of particles used.

#### 3. Does your code use a particle filter to localize the robot?

I think it does, as I used the provided software structure.

## Description of the 2 Dimensional Particle filter

I felt the course material actually made the particle filter more complicated than it really is, and therefore decided to write my own short description.

### Problem description

You have a known map containing known landmark points, and a vehicle whose location on the map is unknown. The vehicle can measure its velocity and turn rate, as well as distances to landmarks. The purpose of the particle filter is to use these measurements to calculate the most likely location of the vehicle on the map.

### Method of Solution

The idea of the particle filter is to use a set of vehicle candidates, called particles, and apply a recursive calculation on the set of particles. Each recursive step goes like this:

The particles are located at different positions around the map. Then vehicle motion (velocity and turn rate) and measurements of positions of landmarks (made by the vehicle we are trying to locate) are applied to each particle (candidate vehicle). Applied means each particle is moved, and the landmark measurements are transformed to be relative to each particle. Then it is calculated how likely it is that each particle would have made exactly these observations. This likelihood is called the weight of the particle. Finally a new set of candidate vehicles are sampled by selecting from the present set of particles with likelihood based on the just calculated weights of the particles. The result of this is a new set of particles which are closer to the position of the sought-after vehicle.

After a few iterations as described above the particle set homes in on the sought-after vehicle.

This steps of method involves a few mathematical calculations described below.

### Initial Location of the Particles

Before the first iteration the initial set of particles must be placed on the map. In the project an initial GPS position is provided (x and y position). Using this and a gaussian distribution based on the standard deviation of the x and y GPS measurements, the initial set of particles are assigned positions. There is also a noisy indication of the initial heading of the vehicle which is used to generate a gaussian distribution for the heading of the particles.

### Moving the Particles

At each time step the particle filter is provided information about the velocity (in the x and y direction) plus the turn rate of the vehicle. The first step of the filter is now to apply these movements to each particle, to calculate a prediction of where each particle would be located when it is moved with the same velocity and turn rate as the sought-after vehicle.

To calculate this the so called bicycle motion model is applied. This is a set of 2 times 3 equations calculating the new values of the x, y position plus the heading angle from the original position and heading angle and the velocity, turn rate and time change measurements. There are 2 set of 3 equations because they are different for the case where the measured turn rate is zero and when it is non-zero.

The equations are derived via straightforward geometry and go like this:

#### Turn rate is zero
x1 = x0 + v \* dt \* cos(theta0)

y1 = y0 + v \* dt \* sin(theta0)

theta1 = theta0

x1, y1, theta1 is the new position and heading. x0, y0 and theta0 is the previous position and heading. v is the velocity and dt is the amount of time passed. The turn rate is zero, so the vehicle does not change its heading during the dt time step. The assumption is that the velocity is constant during dt. For our filter gaussian noise is added to the new x1, y1 and theta1 values to model noise in the measurements of v and dt, plus to model that the v may not have been entirely constant all the time.

#### Turn rate is not zero

In this case the vehicle is also changing its direction, so the equations become slightly more complicated:

x1 = x0 + (v / thetadot) \* (sin(theta0 + thetadot \* dt) - sin(theta0))

y1 = y0 + (v / thetadot) \* (cos(theta0) - cos(theta0 + thetadot \* dt))

theta1 = theta0 + thetadot \* dt

The components mean the same as above. In addition thetadot is the change in turn rate, i.e. the derived of theta. Also here gaussian noise must be added to model measurement noise and the fact that the velocity and the turn rate change may not have been entirely constant all the time.

With these equations we calculate the new position of each particle using the measured velocity and turn rate change from the sought-after vehicle, and the time passed since the last update.

### Transform Landmark Measurements
Besides the movement information the particle filter also receives measurements of detected landmark positions at each time step. These landmark positions are relative to the sought-after vehicle, meaning the x and y values of each position is relative to a coordinate system with 0, 0 in the sought after vehicle, x-axis pointing in the heading of the vehicle and y-axis pointing 90 degrees to the left of the x-axis.

What we want to do is for each of our particles to calculate a measure of how likely it is that the particle made the same set of measurements. This likelihood measures how likely it is that the particle is at the same position as the sought-after vehicle.

In order to calculate this measure we need for each particle to do 2 things.

- transform the measurements to be relative to the particles

- associate each measurement with one of the possible landmarks; i.e. determine which landmark we think the measurement corresponds to for the particle

Transforming vehicle observations to particle observations is, as above, a standard geometric task, and the equations look like this:

xm = xc \* cos(theta) - yc \* sin(theta) + xp

ym = xc \* sin(theta) + yc \* cos(theta) + yp

where xm, ym is the observation relative to the particle in map coordinates, xc, yc is the observation relative to the sought-after vehicle, xp, yp is the position of the particle, and theta is the heading of the particle.

With the above equations we transform the observations made by the sought-after vehicle to be as if they were made by the particle (candidate vehicle).

The second problem is to decide which landmark we believe a transformed measurement actually corresponds to. And here we use the simple assumption that a measurement corresponds to the nearest landmark. Since we know the position of each landmark we can simply calculate the Euclidean distance between an observation and every landmark, and then pick the landmark corresponding to the shortest distance.

The result of these 2 calculation is that we have transformed the landmark observations of the sought-after vehicle into observations made the candidate vehicle (the particle) and associate each observation with a real landmark.

### Calculate Weights

The next step is now to calculate a measure for how likely it is that the candidate vehicle would actually have made these (transformed) observations. Intuitively, a candidate vehicle located far away from the sought-after vehicle would very unlikely make the transformed set of observations (the same observations as the sought-after vehicle). The closer the candidate vehicle is to the position of the sought-after vehicle, the higher the likelihood that it would make the transformed observations.

The measure we calculate is the multi variate gaussian probability using the measurement standard deviation values. We calculate this probability for each observed landmark, and finally calculate the total multi variate gaussian probability of all the measurements by multiplying the values for the individual measurements.

So for each transformed observation, and associated landmark, we calculate the following value:

P = a * exp(-b)

where

a = 1 / (2 \* pi \* rhox \* rhoy)

b = ((xm - xu) \* (xm - xu)) / (2 \* rhox \* rhox) + ((ym - yu) \* (ym - yu)) / (2 \* rhoy \* rhoy)


rhox and rhoy are the standard deviations of the measurements in the x and y directions, (xm, ym) is the measurement in map coordinates, and (xu, yu) is the coordinates of the associated landmark.

Finally we multiply all these P values (one for each observation) and the result is what we call the weight of the particle.

### Resampling

Having now calculated a new position of each particle, and a weight measuring how likely each particle is to have made the landmark observations of the sought-after car, it is time to do the so called resampling. This means building a new set of particles, by sampling from the existing set, using the weight as likelihood for picking each particle. The statistical consequence of this resampling is that the new set of particles will contain more versions of particles with higher weights and will not contain particles with lower weights. In other words the new resampled set of particles will be closer to the position and heading of the sought-after vehicle.

Repeating the process of moving the particles, transforming the observations and calculating new weights, and finally resampling to create a new set of particles, leads to the particles zooming in on the position and heading of the sought-after vehicle. This is quite intuitive because in each step we throw away particles which are the most unlikely to be the ones having made the landmark observations of the sought-after vehicle.

### Implementing the Particle filter

The implemented particle filter is actually nothing more than the implementation of the above equations and algorithms, inside various loops over all particles, all observations and all landmarks.
