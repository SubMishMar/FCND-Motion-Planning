## Project: 3D Motion Planning
![Quad Image](./misc/enroute.png)

---


# Required Steps for a Passing Submission:
1. Load the 2.5D map in the colliders.csv file describing the environment.
2. Discretize the environment into a grid or graph representation.
3. Define the start and goal locations.
4. Perform a search using A* or other search algorithm.
5. Use a collinearity test or ray tracing method (like Bresenham) to remove unnecessary waypoints.
6. Return waypoints in local ECEF coordinates (format for `self.all_waypoints` is [N, E, altitude, heading], where the drone’s start location corresponds to [0, 0, 0, 0].
7. Write it up.

---
### Writeup / README


### Explain the Starter Code

#### 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`
These scripts contain a basic planning implementation that includes...

And here's a demostration of my results:
[![Check this video](http://img.youtube.com/vi/v=nLBZ6U3HlFw/0.jpg)](https://www.youtube.com/watch?v=nLBZ6U3HlFw)


### Implementing Your Path Planning Algorithm

#### 1. Set your global home position
        f = open("colliders.csv")
        line = f.readline()
        f.close()
        lat_lon = re.findall(r"[-+]?\d*\.\d+|\d+", line)
        lat0 = float(lat_lon[1])
        lon0 = float(lat_lon[3])
        self.set_home_position(lon0, lat0, 0)
        
#### 2. Set your current local position
        local_position = global_to_local(self.global_position, self.global_home)

#### 3. Set grid start position from local position
        grid_start = (int(local_position[0]-north_offset), int(local_position[1]-east_offset))

#### 4. Set grid goal position from geodetic coords
        goal_local_pos = global_to_local([-122.39628088, 37.79698094, self.global_home[2]], self.global_home)
        goal_x = int(np.ceil(goal_local_pos[0]) - north_offset)
        goal_y = int(np.ceil(goal_local_pos[1]) - east_offset) 		 
        grid_goal = (goal_x, goal_y)

#### 5. Modify A* to include diagonal motion (or replace A* altogether)
        NE = (-1, 1, np.sqrt(2))
        SE = (1, 1, np.sqrt(2))
        SW = (1, -1, np.sqrt(2))
        NW = (-1, -1, np.sqrt(2))

Besides this, I added a new a_star() implementation here which uses the voronoi approach. Because the grid based A* algorithm is very slow. The implementation that can be found in planning_utils.py, the link is: https://github.com/SubMishMar/FCND-Motion-Planning/blob/f0efb8aa5de7372633af35b9177601e5e5aeb570/planning_utils.py#L217-L266
#### 6. Cull waypoints 
Used a basic collinearity check: https://github.com/SubMishMar/FCND-Motion-Planning/blob/f0efb8aa5de7372633af35b9177601e5e5aeb570/planning_utils.py#L283-L297



### Execute the flight
#### 1. Does it work?
It works!
