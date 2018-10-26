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

Besides this, I added a new a_star() implementation here which uses the voronoi approach. Because the grid based A* algorithm is very slow.

def a_star2(graph, h, start, goal):


    path = []
    path_cost = 0
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)


    branch = {}
    found = False
    
    while not queue.empty():
        item = queue.get()
        current_node = item[1]
        if current_node == start:
            current_cost = 0.0
        else:              
            current_cost = branch[current_node][0]
            
        if current_node == goal:        
            print('Found a path.')
            found = True
            break
        else:
            for node in graph[current_node]:
                # get the tuple representation
                next_node = node
                cost = graph.edges[current_node, next_node]['weight']
                branch_cost = current_cost + cost
                queue_cost = branch_cost + h(next_node, goal)
                
                if next_node not in visited:                
                    visited.add(next_node)               
                    branch[next_node] = (branch_cost, current_node)
                    queue.put((queue_cost, next_node))
             
    if found:
        # retrace steps
        n = goal
        path_cost = branch[n][0]
        path.append(goal)
        while branch[n][1] != start:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************') 
    return path[::-1], path_cost
#### 6. Cull waypoints 
For this step you can use a collinearity test or ray tracing method like Bresenham. The idea is simply to prune your path of unnecessary waypoints. Explain the code you used to accomplish this step.



### Execute the flight
#### 1. Does it work?
It works!

### Double check that you've met specifications for each of the [rubric](https://review.udacity.com/#!/rubrics/1534/view) points.
  
# Extra Challenges: Real World Planning

For an extra challenge, consider implementing some of the techniques described in the "Real World Planning" lesson. You could try implementing a vehicle model to take dynamic constraints into account, or implement a replanning method to invoke if you get off course or encounter unexpected obstacles.


