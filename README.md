# Feedback
Feel free to use the helper function draw_random_sample() to help with the resampling step. The estimated robot pose should be computed not using the weights of the particles but simply the positions and orientations of the particles themselves. You might want to think about ways to ADD noise to thesystem.



# particle_filter_project
Kailin Wu & Hakim Lahlou

### Initializing Particle Cloud
*  We will randomly distribute the particles throughout the space, having a discrete set that is less than the total size of the area. We will implement this by using a random number generator to designate where the particles will go, and will avoid distributing particles to spaces that already have a particle. 
* We will test this by running this algorithm N times, where N is the number of possible spaces, to see if there are any biases or trends in the distribution, ensuring that it is truly random. This can also be tested visually; by looking at the distribution of particles we can see if there is an area clearly not covered by enough particles compared to others.

### Updating Particle Position
* We will apply the same desired transformation/movement of the robot to the particle’s position. If the robot plans to move X distance at some angle A, then we will apply the same movement to all particle’s positions.
* We will test this by using preselected values that have predetermined/given answers, so that we may compare the results of the algorithm against expected values.

### Update Weights With Measurement
* Weight for each particle is computed by the measurement model used in class: 1 / (sum(abs(difference between particle and the robot sensor measurement for each component of the coordinates)
* If there is map data that would influence our robot’s understanding of its location (ie a landmark), then we will also incorporate that *sense* data and update the weights of the particles in those relevant locations.
* We will test this by using preselected values that have predetermined/given answers, so that we may compare the results of the algorithm against expected values.

### Normalize Weights 
* We will sum the weights of all the particles, then divide each weight by this sum. 
* We will test this by using preselected values that have predetermined/given answers, so that we may compare the results of the algorithm against expected values.
* We can also test this by ensuring that the summation of weights is equal to one

### Resample Particles 
* We will create a replacement set proportional to the importance weights
* This can be done by distributing ranges for each space within the area based on their previously calculated weights. Areas with a higher weight will in turn have a larger range. That way, we can “randomly” distribute particles by selecting a random number and determining which range this particle falls into to determine its location.
* This can be tested by performing a test action and seeing the corresponding result match our expected distribution. This can also be tested visually; by looking at the new distribution of particles we can see if there are areas with a higher density of particles, as a result of resampling them based on the weights. 

### Update Estimated Pose
* The estimated position will be the particle with the highest weight; this updates with each new resampling. If multiple points have the same weight, one is randomly chosen; after further resampling one particle weight should eventually be definitively greater than the others.
* For testing, we will provide a sample with a predetermined robot location and see if this algorithm can correctly determine where it is.

### Incorporate Noise 
* There could be noise in the movement of the robot and in the sensor measurements. To account for sensor measurement, the robot will take many measurements of the sensor data and average them. To account for movement noise, we will use other sensors to determine the robot moved as we expected; we compare the expected location to the information from other sensors. 

### Timeline 
* Monday 2/1 - *Initializing Particle Cloud*
* Tuesday 2/2 - *Updating Particle Position*
* Wednesday 2/3  -  *Update Weights With Measurement*
* Thursday 2/4  - *Initializing Particle Cloud*
* Monday 2/8 -  *Resample Particles *
* Tuesday 2/9 -  *Update Estimated Pose, incorporate noise*
* Wednesday 2/10 - *Confirm submission* 
