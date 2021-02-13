# Feedback
Feel free to use the helper function draw_random_sample() to help with the resampling step. The estimated robot pose should be computed not using the weights of the particles but simply the positions and orientations of the particles themselves. You might want to think about ways to ADD noise to thesystem.



# particle_filter_project
Kailin Wu & Hakim Lahlou

## Initial Plan and Timeline

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


## Writeup

### Objectives Description
The goal of this project is to use a particle filter algorithm to locate the position and orientation of the robot when it has a map of its environment. The localization is performed by randomly generating a particle cloud of possible positions and orientations; as the robot moves, the algorithm compares the particles' sensor data to the robot’s actual sensor data and resamples the particles so that they eventually converge on the robot’s actual location. 

### High-level Description
We first initialize a cloud of 10,000 particles in the map area. Each particle each starts with a weight of 1 and a random pose (position and orientation) that is a valid location on the map. As the robot moves, we apply that change in position and angle to every particle in the cloud. After the movement, we compare the sensor data of the robot with the hypothetical sensor data of each particle. We assign the particle a weight based on how similar that particle’s sensor data is to that of the robot’s actual sensor data; then, we normalize all of the weights so that they all add up to 1. The current estimate of the robot’s location is updated to the average orientations and positions of all the particles. Next, we create a new cloud by probabilistically resampling the particles based on their weights. These steps are repeated until the particles eventually converge on the robot’s true position.

### Movement

#### Code Location
We move the particles in update_particle_weights_with_motion_model() by moving each particle the same direction and distance of the robot. This function uses the helper function get_yaw_from_pose(), provided in the starter code. 

#### Functions
We use the robot’s odometry data to get delta_x, delta_y, and delta_yaw of the robot’s last movement. The total distance traveled by the robot, delta_move, is calculated from delta_x and delta_y. 
For each particle in the cloud, we apply the following steps: 
* So that the particle moves relative to its own position angle and not that of the robot, we calculate theta by adding delta_yaw and the particle’s current yaw.
* We add delta_move * math.cos(theta) to the x position move the particle in the x direction, and delta_move * math.sin(theta) to move the particle in the y direction. This updates the position of the particle. 
    * We also added random noise to these movements to simulate a more realistic environment. 
* We convert theta into a quaternion value and update the orientation of the particle by setting it equal to theta. 

### Computation of Importance Weights

#### Code Location
We compute the importance weights for each particle in the function update_particle_weights_with_measurement_model(), based on the code from class. This function uses the helper functions compute_prob_zero_centered_gaussian()  and get_closest_obstacle_distance()  from the class code. 

#### Functions
We check for the nearest object at the following 8 angles around the robot: 0, 45, 90, 135, 180, 225, 270, 315.
For each particle in the cloud, we apply the following steps: 
* Set value q = 1
* For each angle of that particle, we apply the following steps 
    * We get the scanner data of the given position and put it in to a variable called Z_t_k, so that we may know what is the closest object on that side of the robot
    * We translate and rotate the robot’s laser scan reading so that they originate at the particle’s location and orientation
    * We get the distance to the closest obstacle using the helper function get_closest_obstacle_distance().
        * If this value is nan, the scan reading of the particle at that angle is out of bounds  and will not contribute to the weight. To account for this out-of-bounds value, we set q equal to 0 and use the continue command to move onto the next angle in the loop.
    * The helper function compute_prob_zero_centered_gaussian() calculates the probability that the robot is at this particle’s location and orientation based on a zero-centered Gaussian with a standard deviation of 0.1 
    * We multiply q by that probability; as we iterate through the angles, we are multiplying all of the probabilities together 
* After iterating through all of the angles, the weight of the particle is the final value of q.

### Resampling

#### Code Location
We implement this step in the resample_particles() and normalize_particles() functions. This function also uses the helper draw_random_sample() from the project starter code. 

#### Functions
In normalize_particles(), we make all of the particle weights sum to 1.0.
We set a constant called lowerBound to equal .00001, for the purpose of avoiding DIV_BY_ZERO errors. We first add up the weight of every particle to the variable total; If the weight is below the lowerBound variable, we set the weight equal to lowerBound. 
We then make sure the total is nonzero by adding lowerBound to the total. 
Lastly, we divide the weight of every particle by total. 
In resample_partilces(), we first create an array called weightlist, which contains just the normalized weights of each particle. 
To resample the particles, we use the draw_random_sample() helper function. draw_random_sample() returns 10,000 particles drawn from the existing particle cloud; the chance that a particle is resampled is equal to its corresponding normalized weight in weightlist. These 10,000 particles become the new particle cloud. 

### Challenges
One challenge we had with initialize_cloud was to get the particles to only appear in the light gray area of the room, and not all over the entire RViz space. We solved this by creating an array called mapspace. The map data from OccupancyGrid() is a row-major order array, where values of 0 indicate that coordinate someplace the robot could be located, i.e. where a particle could randomly be. We add every index with a value of zero from the map data to mapsace. To generate a random location for the particle, we randomly select an index from mapspace, and translate that index into the x and y coordinates. The next challenge, once the cloud had the right shape, was that the cloud was scaled larger than and not centered relative to the actual map area. With some trial and error, we solved this by using the map origin and resolution information to scale and translate the calculated x and y coordinates to fit the actual map area. A small challenge we had was with getting the particles to move relative to their starting angle, not the robot’s starting angle. We solved this by calculating the robot’s total movement and calculating the x and y change in the particle based on that total and the particle’s angle.
Another challenge came from get_closest_obstacle_distance() as it was providing nan values when provided coordinates that were not valid/acceptable positions on the map. Once we discovered that this was an issue and was propagating an issue throughout the entire weight calculation, we decided to circumvent the issue by recognizing nan outputs from this function and negating them, effectively setting the weight for that particle to zero.

### Future Work
If we had more time, we would find a clearer way to address getting nan values. Additionally, the particle filter takes a while to run when we have 10,000 particles and look at the scan data for 8 angles around each particle, so we would try to improve the efficiency of our calculations. We currently circumvent issues of DIV_BY_ZERO and other issues that are propagated from having a zero total weight, and would use further time to investigate how to remedy this issue without having to directly manipulate the weight to avoid it

### Takeaways
* Focus on running and debugging smaller chunks of code as you go instead of all at once. We wrote a ton of our code all at once before running it, which made debugging more difficult. It was harder to locate which areas were causing the bugs, so we had to write lots of print statements and try commenting out various areas to find them. We should have started running the code as soon as we programmed something that would show results, such as just initializing the cloud and seeing it on RViz before moving on. 
* Not robot programming specific, but related to working in groups: keep up with communication! In the beginning portion of the project, both of us had circumstances that kept us from working on this as much as we would’ve liked to (hence the use of extension time). However, when we finally got started, we were very clear on what we were going to work on when we worked separately and when we worked together, and this helped us a lot. It was also helpful to have one person write the base of code, and then have the other person help debug and correct things with a fresh perspective.

### Gifs
RViz: Particles converging on the robot 
https://drive.google.com/file/d/1nTxVJd42PdpYRsL58dTQeJnZZg8TA1-H/view?usp=sharing

Gazebo: navigating out of the room! https://drive.google.com/file/d/1d0BUKg75F8im4g26W9YqTdzj9MbvXl7V/view?usp=sharing

*NOTE: We were unsure of how many gifs or images we needed, if more are needed, they can be available upon request*