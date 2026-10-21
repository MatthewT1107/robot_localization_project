# Particle Filter
**Matthew Townsend and Pia Swarup**
*Computational Introduction to Robotics*

<img width="400" height="177" alt="PF_gazebo_sim (1)" src="https://github.com/user-attachments/assets/b86c8f27-bf57-4487-bb93-192b333a3219" />

 

## Goal

Given a known map, estimate the robot’s pose from lidar and odometry using a particle filter. This is useful because there are several issues with just using odometry or lidar data. Odometry by itself drifts. Wheel slip and small heading errors add up so after a few meters the robot’s idea of where it is can be quite inaccurate. A lidar scan tells you what the robot sees at a particular point in time but on its own it can’t tell two similar looking places apart (eg. 2 similar looking alcoves in the MAC). Using odometry and lidar together against a known map lets the robot correct its drift and know where it is, which is the first step for anything else it could do like path planning, navigating to a goal, or other functionality

A particle filter is a good fit because it can hold many guesses at once. When the robot isn't sure (when it starts or after it gets lost), the particles can be spread over several possible places, and the scans narrow it down over time. 

## How it works

The filter keeps a cloud of 1050 particles. Each particle is one guess of the robot’s pose in the map frame, with a weight for how good it is. An update only runs once the robot has moved at least 0.2m or turned pi/6 rad, so we don't waste time reweighing the same scan. 

<img width="1280" height="960" alt="Comprobo Particle Filter Conceptual Overview" src="https://github.com/user-attachments/assets/e28584d7-180c-4ce7-839a-5b9ffb096733" />

*Figure 1. Visualization of Particle Filter Functionality*


### Initialization

When a 2D pose estimate is set in rviz (/initialpose), we rebuild the cloud using:
- 1000 particles from a Gaussian around the clicked pose
- 50 particles spread uniformly over the whole map, in case the initial pose was not very accurate
Every particle is checked against the occupancy field and redrawn if it lands outside the map or inside an obstacle.


### Motion update

Odometry returns the change in the robot’s pose in the odom frame. Each particle has its own heading, so we can't just add dx, dy to every particle. Instead we have to rotate dx, dy into the robot’s own frame using the old odom heading. This gives us how much we moved forward and sideways. Then, we rotate the robot frame motion back out using each particle’s heading and add it to the particle. Next, we add dθ to the particle’s heading and add some Gaussian noise (σ=0.02) to x, y, and , since odometry isn’t perfect.


<img width="1024" height="768" alt="Comprobo Particle Filter Conceptual Overview (2)" src="https://github.com/user-attachments/assets/d2104112-4729-4760-970c-1450a8be3015" />
*Figure 2. Example of Correct and Incorrect Poses in Odom Frame*


### Laser weighting

To weigh the particles, we use the laser scan. We project the laser scan to every particles coordinate frame and check how well it lines up with the map. For every 5th beam, we project the beam’s endpoint into the map from the particle’s pose:
```
x = p.x + r·cos(p.θ + θ_beam)
y = p.y + r·sin(p.θ + θ_beam)
```
Then we look up d, the distance from that end point to the nearest obstacle in the likelihood field. If the particle is in the right place, the endpoints land on the walls and d will be close to 0. Each beam scores `exp(−d² / 2σ²)` with σ = 0.1 m, and we
sum the scores over all beams to get the particle's likelihood. Invalid readings (0 or
inf) are skipped.

### Pose estimate

The robot’s pose is the weighted mean of the particles. X and y are a normal weighted average, but theta can’t be averaged directly since angles wrap around. Averaging them directly would give errors whenever the heading fall on both sides of the 180° wraparound. Therefore, we average the weighted `sin θ` and `cos θ` and take `atan2` of the result since vectors don’t have the wraparound problem. This pose estimate is used to update the map to odom transform. 

### Resampling

In the resampling function, we draw a new cloud from the old one with each particle’s chance of getting picked equal to its weight. Highest weight particles get copied and low ones die out. We add some noise (σ=0.05) to every particle so the copies don't all end up in the same position. Then, we remove any particle that ended up inside a wall or off the map, and replace it with a  random one. We also implemented some adaptive random injection, replacing some particles with random ones spread over the whole map depending on how ‘lost’ the filter seems. Last, we set all weights equal.



<img width="1280" height="960" alt="Comprobo Particle Filter Conceptual Overview (3)" src="https://github.com/user-attachments/assets/d3183949-4d9d-4808-845e-bf772437483d" />

*Figure 3. Visualization of ROS nodes and topics*

## Design Decisions


### Gaussian instead of grid sampling for init

We started off using a grid-like initial sampling method that ignored the initial pose and put the particles evenly spaced throughout the map. Particles that ended up in objects were randomly placed in valid spots on the map. After testing, we found that various particle clusters on different spots on the map would form (usually around four) instead of one general cluster that is agreed on. As a result, the Neato guess was being placed in the middle of the clusters because we utilized the weighted mean of all particles in our guess. 
We then pivoted to the method of using a Gaussian distribution of particles around a set initial pose which overall resulted in a faster converging particle filter. Accurate initial pose guesses would cause the filter to converge on the right spot quickly while inaccurate guesses take more time (odometry and lidar updates) to converge on the accurate spot.

### Summing beam likelihoods instead of multiplying

Usually, in particle filters, beam likelihoods are multiplied. However, we ran into some issues when we implemented this: 
- Multiplying many numbers less than 1 makes the weights tiny and they can easily round to 0 for almost every particle.
- One bad beam can give a score near 0, which can completely wipe out particles that were otherwise lined up perfectly.
Therefore, we chose to sum the weights because even with a few bad beams, a particle would still score well if most of its beams line up. This means that the weights will be less distinct from each other, which makes it harder to distinguish between good and bad particles since they are closer together. We added resampling noise to keep the cloud distribution good.

### Adaptive random injection instead of fixed

At first we injected 50 random particles every resample, but this was too few to recover when the cloud gets lost and made it noisy even when it was working well. So instead we decided to track two running averages of how well the particles match the scan:
- `w_slow` (α = 0.05): how well the filter has been doing over a long time.
- `w_fast` (α = 0.2): how well it's doing right now.
If the current match is much worse than usual (i.e. w_fast drops below w_slow), the robot is probably lost (kidnapped) so we inject more random particles:
```
p_random = max(0, 1 − w_fast / w_slow)
n_random = max(5, p_random · N)
```
The baseline is 5 random particles being added, but when the scan suddenly stops matching a larger part of the cloud gets replaced with random particles to search for the robot’s pose. This is the same idea as Augmented Monte Carlo Localization. 

### Using every 5th beam

A standard lidar scan gives 360 beams per scan. Scoring all of them for 1050 particles is around 378000 likelihood field lookups per update, which runs slowly in Python. Therefore, we chose to use every 5th beam which made each update around 5 times faster without a noticeable drop in accuracy. 

### Particle count

We first tested the particle filter with 625 particles (600 in the gaussian distribution and 25 spread around the entire map for initialization). The filter worked well with this amount, but by tuning the number of particles we found that an increased particle count allowed the filter to converge faster. Increasing the particle count to 1050 came with no trade off on simulation speed and provided an overall improvement.

## Testing and Results


### Staged testing

We tested each part of the filter after we wrote it. For initialization, we checked in rviz that the cloud appeared around the clicked pose and no particles were inside walls. For motion update, we drove the robot and checked that the cloud moved with it and slowly spread out. This is how we found sign errors in the odom update. For laser weighting, we checked that the particles closest to the true pose got the highest weights. For resampling, we checked that the cloud collapsed onto the highest weight particles. Lastly, we tested the filter as a whole using the provided bags and gazebo sim with teleop. 


### Unit tests

For unit testing, we only tested pure logic elements of our code. This was to not force ourselves to do unit tests with ros running. We call real methods on fake stand-in classes, and use a stand-in node, InertNode, which has all the methods and attributes of the particle filter. The stand-in node uses a fake 5 m x 5 m map where part of the map counts as an obstacle and anything outside returns NaN, plus a copy of TFHelper that keeps the real angle math but skips the tf2 setup. Since the filter adds random noise that we do not want for the purpose of testing, we create a fixture to replace np.random.normal with a function that returns 0. The tests run with ‘python3 -m pytest test_pf.py -v’, and they only cover logic, so the bag files and simulator are still how we tested the filter as a whole. 
We created four tests:

**Normalized particles sum to one and handle weights of 0**
- Asserts the sum of weights in the particle cloud adds to 1
- Asserts weight ratios are kept between different particles.
- Asserts particles with weight 0 are still normalized

**Test that angles wrap around**
- Asserts a particle facing 3.0rad while robot turns +0.5rad would have a theta of 3.5rad- 2*pi rad
- Assert the particle’s theta is between -pi and pi radians

**Test that the filter averages headings that sit on opposite sides of the -pi and +pi boundary correctly**
- Asserts headings of +3.1 and -3.1 average to ~pi

**Make sure initial particle cloud always creates 1050 valid particles**
- Asserts length of particle cloud list is 1050
- Asserts every particle in list isn’t inside an object
- Assert all weights of particles sum to 1


### Bag results

We ran the filter on the 2 provided MAC floor bags and measured each run with a script called scripts/pf_metrics.py. The script records multiple metrics while the filter is running. Additionally, it checks if the cloud has converged. It counts the cloud as converged when the main cluster (the closest 90% of particles, so the injected random ones don’t stop convergence) has a position spread under 0.25m and a heading spread under 0.25 rad. Full details about the bags are in `bags/README.md` and the raw data is in `results/`. The following table shows a summary of the tests on both takes of the MAC 1st floor.
| Run | Converged after | Main cluster spread (avg / max) | Heading spread |
|---|---|---|---|
| Take 1 | 8 updates (9.1 s) | 0.22 m / 0.33 m | ~6° |
| Take 2 | 7 updates (12.7 s) | 0.22 m / 0.33 m | ~6° |

In both runs the filter stayed converged until the end of the bag and the laser scan lined up with the walls. Take 2 was slightly less stable than take 1 but still tracked well.

Another attempt on take 2 where the initial pose was clicked about 11m away ended up failing since the filter never recovered. This shows that a reasonable starting guess matters. Increasing the number of random particles could probably improve convergence with bad initial pose guesses, however, we did not implement this in this project.

To test recovery, we let the filter converge on take 1 and then clicked a wrong 2D pose estimate mid run. This rebuilds the cloud around the wrong pose. The error was measured against the normal take 1 run at the same time in the bag and the results are shown below.
| Kidnapping | Offset | Result | Updates to recover | Error after |
|---|---|---|---|---|
| #1 (~21 s) | 0.7 m | Recovered to the correct position | 22 (18.9 s) | 0.10-0.20 m |
| #2 (~51 s) | 2.8 m | Locked onto the wrong spot | — | 7.4 m |

This shows that the adaptive random injection handles small kidnappings quite well. After the larger kidnapping (2.8m), the cloud converged again but 7.4 m way in a part of the hallway that looks the same to the lidar. Because we could only rely on the lidar for this part of the filter, it was hard to converge to the correct part of the hallway because several parts give almost the same scan. 

### Simulator tests

We also ran several tests using the gazebo simulator with the neato. We used the gauntlet world and teleop to drive around the neato. It was slightly harder for the particle cloud to converge in parts of the gauntlet map due to the similar looking corners, however, it mostly converged well after around 10 updates. We also tested kidnapping in this map, and the particle filter had quite a hard time converging, but was able to converge after driving it around the center of the map (with the objects around it). 


### Comparison of fixed and adaptive random injection

In our initial implementations of the resampler, it injected a fixed 25 random particles (4% of the 625 particle cloud at the time) every update. We replaced it with an adaptive injection logic, which adds at least 5 random particles and up to the whole cloud, depending on how bad the current scan match is compared to usual (scaled by `1- w_fast / w_slow`). With fixed injection, the amount of searching never changes. When the filter is tracking well, the random particles are just acting as noise and die out the next resample, so they cost computing power and time without contributing to the filter. When the robot is lost, the 25 random particles spread over the whole map but are almost never enough for one of them to be near the real pose to be able to recover the cloud.  

> **TODO:** ADD VIDEO OF FIXED INJECTION

The adaptive injection fixes both cases. When the filter is confident, almost the whole cloud is used to track. When the scans suddenly match worse than how they have been, the filter assumes it is lost and replaces a big part of the cloud with random particles to search the map for the new pose. 

> **TODO:** ADD VIDEO OF ADAPTIVE INJECTION

There are still some limitations to our implementation of the adaptive injection. It still can’t recover well from larger kidnaps (more than 3 m). The random particles are spread over the whole map, and the hallway has multiple places that  have similar looking lidar scans. Once the cloud was locked onto one of them, the scans matched well again, so `w_fast` increased and the filter stopped injecting random particles even though it was converging in the wrong place. The model can only tell that something is wrong when the scan match gets worse, so it can’t detect a wrong pose that looks right to the lidar.

## Challenges


### Coordinate frame sign errors in odom update

Our first motion update moved particles in the wrong direction whenever their heading was
different from the robot's odom heading. The odom delta is in the odom frame, but each particle needs it in its own frame. For the rotation, we had to first move it from the odom frame to the robot frame to the particle frame, and we had a few flipped signs in the `sin` terms along the way. We caught it by turning off the laser update and watching the cloud drive sideways in rviz.

### Theta averaging

Our method of determining the guess for the robot uses the weighted means of position and heading. Weighted means were simple to calculate for position, but heading had to be done differently because of how angles wrap around. To take the weighted  mean for each particle, we used trigonometry to convert each particle angle into horizontal and vertical components using sin and cosine, took the weighted sum of each component, and then used arctan to convert the components back into an angle, giving us a correct heading guess.

### Particle converging too fast

While developing the particle filter, we came across an issue of particles in clusters stacking very closely on top of each other, creating a kidnapping problem of many particles going to and staying in the wrong spot. To combat this, we add random noise to particles when they are resampled. This made it so particles are less likely to layer on top of each other and distribute in different areas of a cluster. Kidnapping was reduced as particles were not bound to a specific position, and covered more area to create a greater chance of a particle spawning in a position that is closer to the actual Neato.

### Git issues

We had some issues while using Git. It was mostly late or poor communication when either of us pushed to the repository. This caused a function overwrite of update_robot_pose and some merge conflicts. These were rather minor but still hindered workflow a bit. After the conflict, we communicated better and made sure to pull before making any new commits.

## Future Improvements

We have several ideas for how we can improve the particle filter in the future. One idea is to make the pose output smoother since we found that our estimate jumps a little after each resample. Running it through a Kalman filter would give a smoother pose. 

Our current implementation only checks random particles against obstacles, which are designated with an outline on the map. Some particles land inside the walls of obstacles where the robot cannot be. It could be improved by only sampling free cells that aren’t enclosed within the map to make more of the random particles useful.

We used a likelihood field for our particle filter but a beam that passes through a wall could still score well. Using Ray casting would trace each beam through the map and give a more accurate score but it is a lot slower.

To improve the particle filter’s performance for robot kidnapping, we should reimplement the grid method for particle sampling. We would have to alter the resampling logic to converge the particle cloud faster to the cluster with the highest weighted score.


## Lessons Learned


### Test in stages

This project particularly showed us how important testing the code in stages is. Testing each function individually after writing it proved valuable since we were able to find errors in our code relatively easily.

### Visualize everything

Using Rviz to visualize our functions and particle filter allowed us to see what was going on in every stage which helped us understand our code and errors a lot easier. We could also see what was happening to the particle cloud and whether the amount of noise or amount of random samples was enough.

### Need to include good amount of randomness

Specific to particle filter projects, we learned the importance of including randomness. In developing this project, we didn’t start out considering including many aspects of randomness to our filter. However in development through watching the initial uniform grid not work to our liking, particle clusters focusing on essentially one point, and the Neato being “kidnapped” by incorrect clusters with no way of finding the right location, we implemented multiple features that gave particles stochasticity that resolved these initial problems. In any future localization problems, we will consider the importance of entropy.

### Claim functions before starting to code separately and Pull before every commit

We didn’t clearly divide the work between each other initially, which caused some minor issues. Specifically, we both wrote the `update_robot_pose` function which caused git merge errors and wasted time. After this incident, we communicated more clearly and made sure to let each other know every time we pushed a new commit. 

### Use bags for debugging

Using bags for debugging and tuning parameters was very useful in this project, because we could see how the particle filter behaved differently while keeping all the sensor input values constant. This allowed us to debug the particle filter code a lot easier and made tuning of the parameters easier. Additionally, the data collection script allowed us to quantify the filter’s performance instead of having to guess the performance based on the visuals.


## How to run

Build the package and source the workspace:

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select robot_localization
source install/setup.bash
```

Start the map server and the particle filter:

```bash
ros2 launch robot_localization test_pf.py \
    map_yaml:=$HOME/ros2_ws/src/robot_localization/maps/mac_1st_floor_final.yaml
```

In a second terminal, open rviz:

```bash
rviz2 -d ~/ros2_ws/src/robot_localization/rviz/turtlebot_bag_files.rviz
```

In a third terminal, play a bag:

```bash
ros2 bag play ~/ros2_ws/src/robot_localization/bags/macfirst_floor_take_1 --clock
```

Then use **2D Pose Estimate** in rviz to click the robot's starting position. The particle
cloud should appear there and converge as the robot drives.

To measure a run, start the metrics script in its own terminal while the filter runs:

```bash
python3 scripts/pf_metrics.py --out results/my_run.csv
```

To replay one of our recorded result bags, see [`bags/README.md`](bags/README.md).
