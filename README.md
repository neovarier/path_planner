# Path Planner Project

The goal of the path planner project is to make the self driving vehicle drive on a 3 lane highway simulator.
The self car should be able to complete a lap without any incidents - collision, exceeding maximum velocity, maximum acceleration & maximum jerk. The car should be able make lane changes whenever it needs to. The project walk through was very much helpful.

To complete this project I focused on following two areas:
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
* x- & y-components of the car's velocity

With self localization, we get the current location in both x,y & Frenet Coordinates.
Also as part of trajectory generation, 50 calculated waypoints are fed to the simulator. 
Each waypoint is visited by the car every 0.02 seconds. So in each cycle when the simulator gives control to the path_planner program, 
it has some unused waypoints from the trajectory of previous cycle. For each cycle the unused waypoints from the previous cycle are used as base and starting portion of the next trajectory.

For behaviour planning, the basic state machine that I have used:
 * Check if the front car on the same lane is at a safe distance from my car (State 1).
 * If the front car is not close, then keep the same lane and accelerate (State 2).
 * If the front car is not at a safe distance, then check if lane can be changed (State 3).
 * If lane can be changed then execute the lane change at the same velocity (State 3).
 * If the lane change is not possible then decelerate (State 4).
 
### State 1:
From sensor fusion feedback, the future location of the other cars in the same lane are predicted.
The following steps are taken:
A) To check if front cars in the same lane are too close to self car or not in the future
* Calculate the future location FL of the other cars in the same lane assuming the current velocity does not change
* If the distance Fl of all cars in front are more than the safe distance from the self car's future location, then transition to State 2 is possible.
* If the distance Fl of any cars in front is less than the safe distance from the self car's future location, then do a transition to State 3.
B) To check if front cars in the adjacent lane are too close to do a lane change to the self lane or not in the future
* Calculate the future location OFL of the other cars in the adjacent lanes assuming current velocity does not change.
* If the distance OFL of all front cars in adjacent lane is  more than the safe distance from the self car's future location then transition to State 2 to possible.
* If the distance OFL of any front car in adjacent lane is less than the safe distance from the self car's future location then check Frenet d coordinate of the car - If the d coordinate less than the 1 unit from the self lane boundary then do a transition to State 3. If not then transition to State 2 is possible.

If transition to State 2 is possible from both A) & B) then make a transition to State 2.
### State 2:
Increment the velocity by 0.224 mph which is equivalent to 5 m/s^2. This is within the bounds of maximum acceleration possible -> 10 m/s^2. When accelerating use all the unused waypoints of the previous cycle as the base for the new set of waypoints.

### State 3:

A)To lane change if there is safe gap in the adjacent lane
* Calculate the future location FL1 of the other cars in the adjacent lane assuming the current velocity does not change
* Calculate the future location FL2 of the other cars in the adjacent lane assuming the current velocity increases once in the beginning by applying maximum acceleration - 10 m/s^2
* Calculate the future location FL3 of the other cars in the adjacent lane assuming  the current velocity decreases once in the beginning by applying maximum deceleration - 10 m/s^2
* If the three distances Fl1,Fl2 & Fl3 of all front and back cars in the adjacent lane are more than the safe distance from the self car's future location, then a LANE CHANGE is possible.
* If any of the three distances Fl1,Fl2 & Fl3 of front and back cars in the adjacent lane are less than the safe distance from the self car's future location, then do a transition to State 4.
* If LANE CHANGE is possible then check the relative velocity of closest front car in the adjacent lane w.r.t the front car in the same lane and check the gap of the closest front car in the adjacent lane w.r.t self car - If the relative velocity is positive OR the distance gap is greater than a maximum threshold then executing lane change and going to State 1 is possible. If the relative velocity is negative AND the distance gap is lesser than a maximum threshold then go to State 4. The idea here is that if the front car in the adjacent lane is slower than the front car in the same lane, there is no benefit of making the lane change unless the gap is too high which would give an opportunity to make the lane change to the adjacent lane and then another lane change back to the current lane.
* If the current lane is the middle lane and excuting lane change to both the adjacent lanes is possible as per the above conditions, then check which lane is the fastest one. The fastest lane is decided by the speed of slowest vehicle ahead in either lanes. If the speed of slowest vehicle ahead in one lane is more than that of the other, then choose the first lane.

B)To check if front cars in the adjacent lanes are too close to do a lane change to the target lane or not in the future
* Calculate the future location OFL of the other cars in the adjacent lanes to the target lane assuming current velocity does not change.
* If the distance OFL of all front cars in adjacent lane to the target lane is  more than the safe distance from the self car's future location then executing the lane change and going to State 1 is possible.
* If the distance OFL of any front car in adjacent lane to the target lane is less than the safe distance from the self car's future location then check Frenet d coordinate of the car - If the d coordinate less than the 1 unit from the target lane boundary then lane change is dangerous and do a transition to State 4. If not then executing lane change and going to State 1 is possible.

When executing lane change and going to State 1 is possible from both A) & B) then execute the lane change and goto State 1.

When making the lane change use all the unused waypoint of the previous cycle.

### State 4:
Decrement the velocity by 0.224 mph which is equivalent to -5m/s^2. This is within the bounds of maximum deceleration. When decelerating use the first 10 unused waypoints from the previous cycle and flush the rest. This will make sure that the velocity decrement will take into effect quickly in the future (0.2 seconds). This will rule out cases where front car is decelerating very fast.

### Safe Distance Parameters
Safe Distance for Front Car in the same lane       : 30 mts.
Safe Distance for Front Car in the adjacent lane   : 30 mts.
Safe Distance for Back Car in the adjacent lane    : 30 mts.
Max Distance for the Front Car in the adjacent lane: 100 mts.

## Trajectory generation
In case of deceleration, use only the first 10 unused waypoints from the previous cycle to generate the future trajectory. If accelerating or making lane change use all the unused waypoints from the previous cycle.
Taking the last 2 waypoints from the set of usused waypoints to be used from the previous cycle trajectory as the base and 3 new predicted points beyond the last unused waypoint, I am using spline library to generate a smooth trajectory.
The 3 new waypoints are predicted by using the Frenet coordinates of last unused waypoint and as per the global Frenet coordinates of the track:
end_s + 30.
end_s + 60.
end_s + 90.
If a lane change trajectory is expected for the future waypoints, then it is taken into consideration.

I have followed the steps discussed in the Project walk through:
 * Convert -.the new 3 (s,d) waypoints into x,y coordinates
 * Transform these (x,y) global coordinates to local car coordinates with last unused waypoint as the origin &  car orientation at the last unused waypoint as the rotation angle.
 * Now use the 5 points for a smooth curve generation using spline library
 * After generating the spline, select 2 points on the spline - origin and a point 30 meters ahead in x-direction on the spline.
 * Assuming the car goes with a constant velocity on the direct path between these 2 points, select equidistant points on this direct path.
 * Project these points directly on the spline and use the necessary points required for the new trajectory.

## Result:
The car is able to complete 4.32 miles without any incidents. The car is able to make lane changes smoothely. A recorded video of a lap is also attached - video.mp4

## Possible Improvements:
 * As described in the Behaviour Planning lesson, cost functions can be defined. Total cost could be calculated based on different possible actions. Best action could be decided based on which action has the least total cost.
 
 ## Review Comments Addressed 
 From the previous review I have addressed the comments:
 * Self Car was colliding the front car in the same lane when the front car was decelerating. In the last submission, I was using all the unused waypoints from the previous cycle in case of deceleration. Since there would be ususally 45-47 unused waypoints from the previous cycle. So the decelerated velocity was taking into effect only after the 45-47 waypoints (~ 1 second) for the next cycle. In some cases, this would be too late if the front car is decelerating quickly. In the current submission I am using the first 10 unused waypoints from the previous cycle and flushing out the rest. In this case the decelerated velocity will take into effect after 10 waypoints (0.2 sec) and the next 40 waypoints will use decelerated velocity. This should avoid the collision when the front car is decelerating.

* Self Car is making a right lane change and the back car in the right lane is colliding from behind
In the last submission, while making lane change I was assuming the future location of front and back cars in the adjacent lanes assuming that their velocity will not change. But if the car is accelerating from behind, then it may lead to collision. In the current submission, I am calculating the future locations of  front and back cars in the adjacent lanes assuming that it has 1) constant velocity 2) accelerates in the beginning once & 3) decelerated in the beginning once. By checking these distances against the safe distance it will taking into consideration the acceleration case of back car & deceleration case of front car as well in the adjacent lane. This should avoid collisions during lane change from back cars in case they are accelerating.

* Lane Change mechanism is elaborated

* Compilation warning is fixed
