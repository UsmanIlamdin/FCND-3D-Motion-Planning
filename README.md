# 3D Motion Planning
Udacity FCND-Term1-Project-2 - 3D Motion Planning
![Drone flying](./images/drone_flying.gif)

# Required Steps for a Passing Submission:
1. Load the 2.5D map in the colliders.csv file describing the environment.
2. Discretize the environment into a grid or graph representation.
3. Define the start and goal locations.
4. Perform a search using A* or other search algorithm.
5. Use a collinearity test or ray tracing method (like Bresenham) to remove unnecessary waypoints.
6. Return waypoints in local ECEF coordinates (format for `self.all_waypoints` is [N, E, altitude, heading], where the drone’s start location corresponds to [0, 0, 0, 0].
7. Write it up.
8. Congratulations!  Your Done!

# Project description

The following are the main code used on the project:

- [motion_planning_from_udacity.py](./motion_planning_from_udacity.py): This is the based implementation for this project provided by Udacity on [its seed project](https://github.com/udacity/FCND-Motion-Planning).
- [planning_utils_from_udacity.py](./planning_utils_from_udacity.py): It was also provided by Udacity on the [its seed project](https://github.com/udacity/FCND-Motion-Planning). It contains an implementation of the [A* search algorithm](https://en.wikipedia.org/wiki/A*_search_algorithm) in addition to utility functions.
- [motion_planning.py](./motion_planning.py): This version extends the provided implementation with the following features:
  - The global home location is read from the [colliders.csv](./colliders.csv) file.
  - The goal location is set from command line arguments (--goal_lat, --goal_lon, --goal_alt).
  - The calculated path is pruned with a collinearity function to eliminate unnecessary waypoints.
- [planning_utils.py](./planning_utils.py): This file is used by [motion_planning.py](./motion_planning.py) instead of the seed version. It provides support for the features mention above and also extends the A* search algorithm to include diagonals actions.

Here are some examples of trajectories found with this code:

![A* grid with diagonals and collinearity prune](./images/a_star_grid_diagonals_prune.png)

In addition to that implementation using a grid, the following code use a graph to search for the path to the goal:

- [graph_motion_planning](./graph_motion_planning.py): It has the same features as [motion_planning.py](./motion_planning.py), but detegates all the searching functionality to the method [calculate_waypoints](./graph_planning_utils.py#L144-L174).
- [graph_planning_utils.py](./graph_planning_utils.py): Implements a A* search algorithm using a graph calculated from the colliders information.

Here are some examples of trajectories found/no found in this code:

![A* graph](./images/graph_a_star.png)

It is interesting to see how much faster this algorithm is compared to an A* on a grid. It is also interesting to see the path was not found in the upper-right position in this case. Another characteristic is waypoint count, in this case, was higher than the one found with A* on a grid.


# Run the Project

To run the code first need to change to the repo directory and activate the environment with the following command:

```
source activate fcnd
```
Now is time to run the code,for the graph implementation:

```
python graph_motion_planning.py --goal_lon -122.40195876 --goal_lat 37.79673913 --goal_alt -0.147
```
 For the A* grid implementation:
```
python motion_planning.py --goal_lon -122.40195876 --goal_lat 37.79673913 --goal_alt -0.147
```

There are examples for different goal coordinates in the following file
[graph_and_motion_planning_cordinates.txt](./graph_and_motion_planning_cordinates.txt)




# [Project Rubric](https://review.udacity.com/#!/rubrics/1534/view)

## Writeup

### Provide a Writeup / README that includes all the rubric points and how you addressed each one. You can submit your writeup as markdown or pdf.

You're reading it! Below I describe how I addressed each rubric point and where in my code each point is handled.

## Explain the Starter Code

### Test that `motion_planning.py` is a modified version of `backyard_flyer_solution.py` for simple path planning. Verify that both scripts work. Then, compare them side by side and describe in words how each of the modifications implemented in `motion_planning.py` is functioning.

Both version are similar in the sense they implement a [finite-state machine](https://en.wikipedia.org/wiki/Finite-state_machine) to command the drone. They are similar but not exactly the same. On the [`backyard_flyer_solution.py`](./backyard_flyer_solution.py) the states and transitions represented are:

![backyard_flyer_solution.py state machine](./images/backyard_flyer_state_machine.png)

The state machine implemented on [`motion_planning.py`](./motion_planning_from_udacity.py), adds another state to the previous one:

![motion_planning_from_udacity.py state machine](./images/motion_planning__state_machine.png)

There is a new state, `PLANNING`, between  `ARMING` and `TAKEOFF`. When the drone is at the state `ARMING` and it is actually armed ((./motion_planning_from_udacity.py#L66)) on the `state_callback` method ([lines 61 to 72](./motion_planning_from_udacity.py#L61-L72)), the transition to `PLANNING` is executed on the method [`plan_path`](./motion_planning_from_udacity.py#L114-160). This method responsibility is to calculate the waypoints necessary for the drone to arrive at its destination.

On the `plan_path` method:
- The map is loaded ([line 133](./motion_planning_from_udacity.py))
- The grid is calculated at [line 136](./motion_planning_from_udacity.py#L136) using the method [`create_grid`](./planning_utils_from_udacity.py#L6-L41) from the module [`planning_utils.py`](./planning_utils_from_udacity.py).
- The goal grid is set 10 north and east from local position on [line 144]((./motion_planning_from_udacity.py#L144)).
- To find the path to the goal, [A* search algorithm](https://en.wikipedia.org/wiki/A*_search_algorithm) is executed on [line 151](./motion_planning_from_udacity.py#L151) using the [`a_star`](./planning_utils_from_udacity.py#L91-L140) method from the module [`planning_utils.py`](./planning_utils_from_udacity.py).
- The waypoints are generated at [line 157](./motion_planning_from_udacity.py#L157), and they are sent to the simulator using the method [`send_waypoints`](././motion_planning_from_udacity.py#L109-L112) at [line 161](./motion_planning_from_udacity.py#L161).

## Implementing Your Path Planning Algorithm

### In the starter code, we assume that the home position is where the drone first initializes, but in reality, you need to be able to start planning from anywhere. Modify your code to read the global home location from the first line of the `colliders.csv.` file and set that position as global home (`self.set_home_position()`)

The home position is read at [motion_planning.py line 124](./motion_planning.py#L124). It use the function [`read_home`](./planning_utils.py#L145-L155) added to `planning_utils.py`.

### In the starter code, we assume the drone takes off from map center, but you'll need to be able to takeoff from anywhere. Retrieve your current position in geodetic coordinates from `self._latitude()`, `self._longitude()` and `self._altitude()`. Then use the utility function `global_to_local()` to convert to local position (using `self.global_home()` as well, which you just set)

This coordinates transformation is done at [line 130](./motion_planning.py#L130).

### In the starter code, the start point for planning is hardcoded as map center. Change this to be your current local position.

The grid star point is calculated from [line 144 to 146](./motion_planning.py#L144-L146).

### In the starter code, the goal position is hardcoded as some location 10 m north and 10 m east of map center. Modify this to be set as some arbitrary position on the grid given any geodetic coordinates (latitude, longitude)

Three new parameters were added to the [motion_planning.py](./motion_planning.py#L189-L191) to accept goals coordinates. The coordinates are converted to local coordinates at [lines 151 to 152](./motion_planning.py#L151-L152) to be used on the search algorithm.

### Write your search algorithm. Minimum requirement here is to add diagonal motions to the A* implementation provided, and assign them a cost of sqrt(2). However, you're encouraged to get creative and try other methods from the lessons and beyond!

The diagonals movements were implemented by adding them to the [Action enum](./planning_utils.py#L59-L62). The [valid_actions method](./planning_utils.py#L92-L99) was modified to take those actions into account. Here is an example of the A* trajectories on a grid:

![A* grid](./images/a_star_grid.png)

When the diagonal actions are implemented, the trajectories to the same goals changed:

![A* grid with diagonals](./images/a_star_grid_diagonals.png)

### Cull waypoints from the path you determine using search.

The path was pruned at(./motion_planning.py#L162) using collinearity([collinearity_prune function](./planning_utils.py#L170-L201)) with the method provided by the lectures. The trajectories after this transformation are:

![A* grid with diagonals and collinearity prune](./images/a_star_grid_diagonals_prune.png)

## Executing the flight

The code runs successfully, allowing the drone to go from grid_start to grid_goal without hitting any obstacles along the way. One example of successful trial is presented in the images on the first page of this document. Moreover, different goal locations have been tried, starting from various points in the city. The only issue that has been encountered is that, if the start and goal locations are too far apart from each other, the simulator cuts off the connection without performing the flight. This is probably due to the quite intensive calculations that the simulator has to go through. One solution was to simply use a closer goal location.

# FCND-3D-Motion-Planning
