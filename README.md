# Path Planner Project

The goal of the path planner project is to make the self driving vehicle drive on a 3 lane highway simulator.
The self  car should be able to complete a lap wihtout any incidents - collision, exceeding maximum velocity, maximum acceleration & maximum jerk.
The car should be able make lane changes whenever it needs to. 

To complete this project I focused on two areas:
* Behaviour planning
* Trajectory generation

## Behaviour Planning

Following behavioral decisions are taken based on the feed back from sensor fusion and self localization:
* To accelerate on the same lane
* To decelerate on the same lane
* To make a lane change to left lane
* To make a lane change to right lane

With the help of sensor fusion, we would get the following parameters of other cars:
* Location in Frenet co-ordinates system
* x- & y-components the car's velocity

With self localization, we get the current location in both x,y & Frenet Coordinates.
Also as part of trajectory generation, 50 calculated waypoints are fed to the simulator. 
Each waypoint is visited by the car every 0.02 seconds. So in each cycle when the simulator gives control to the path_planner program, 
it has some unused waypoints from the trajectory of previous cycle. For each cycle the unused waypoints from the previous cycle are used as
base.


For behaviour planning, the basic state machine that I have used:
 * Check if the front car on the same lane is at a safe distance from my car (State 1)
 * If there front car is not close, then keep same lane and accelerate (State 2)
 * If the front car is not at a safe distance, then check if lane can be changed (State 3)
 * If lane can be changed then execute the lane change at the same velocity (State 3)
 * If the lane change is not possible then decelerate (State 4)
 
State 1:
From sensor fusion feedback, the future location of the other cars in the same are predicted.
If their distance is more than a safe distance from the self car's future location, then do a transition to State 2.
If their distance is less than a safe distance from the self car's future location, then do a transition to State 3.

State 2:
Increment the velocity by maximum acceleration possible which is 0.448 mph which is equivalent to 10 m/s^2.

State 3:
For lane change, check if any car's future location in the front or back is at a safe distance from the self car's future location in the adjacent lane.
If their gap is lesser than the safe distance, then goto State 4
If their gap is more than the safe distance then check the relative velocity of the front car in the adjacent lane with respect to the front car in the current lane.
If the relative velocity is positive then make the lane change and go to State 1. If relative is negative then go to State 4, unless the front car's gap in the adjacent is too high.
In that case, still make the lane change and go to State 1. The idea here is that is the front car in the adjacent lane is slower than the front car in the same lane, there is no
benefit of making the lane change unless the gap is too high which would give an opportunity to make the lane change to the adjacent lane and then another lane change back to the current lane.

In case if the current lane is the middle lane and both the adjacent lanes are safe for lane change, select the fastest lane.

State 4:
Decrement the velocity by 0.224 mph which is equivalent to -5m/s^2.

## Trajectory generation
Taking the last 2 unused waypoints from the previous cycle trajectory as the base and 3 new predicted points beyond the last unused waypoint, I am using spline library to generate a smooth trajectory.
The 3 new waypoints are predicted by using the Frenet coordinates of last unused waypoint and as per the global Frenet coordinates of the track:
end_s + 30
end_s + 60
end_s + 90
If a lane change trajectory is expected for the future waypoints , then it is taken into consideration.

I have followed the steps discussed in the Project walk through:
 * Convert the new 3 (s,d) waypoints into x,y coordinates
 * Transform these (x,y) global coordinates to local car coordinates with last unused waypoint as the origin &  car orientation at the last unused waypoint as the rotation angle.
 * Now use the 5 points to for smooth curve generation using spline library
 * After generating the spline, select 2 points on the spline - origin and a point 30 meters in x-direction on the spline.
 * Assuming the car goes with a constant velocity on the direct path between these 2 points, select equidistant points on this direct path.
 * Project these points directly on the spline and use the necessary points required for the new trajectory.

Result:
The car is able to complete 4.32 miles without any incidents. The car is able to make lane changes smoothely.

Possible Improvements:
 * As described in the Behaviour Planning lesson, cost functions can be defined. Total cost could be calculated based on different possible actions.
 Best action could be decided based which has the least total cost.
 * A mechanism to predict whether other cars would do a lane change or not would be useful information as we are relying on future locations of other cars.




