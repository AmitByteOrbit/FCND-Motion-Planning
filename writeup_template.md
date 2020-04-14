## Project: 3D Motion Planning
![Quad Image](./misc/main.png)

---


### Writeup / README
Below I describe how I addressed each rubric point and where in my code each point is handled.

### Explain the Starter Code

#### 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`
These scripts contain a basic planning implementation that includes...
* Reading data from a CSV file
* Creating a grid (2D)
* A* grid search (2D)
* Sending waypoints to the Drone / Simulator


### Implementing Your Path Planning Algorithm

#### 1. Set your global home position
Here students should read the first line of the csv file, extract lat0 and lon0 as floating point values and use the self.set_home_position() method to set global home. Explain briefly how you accomplished this in your code.

I used the `readline()` command to read just the first line of the file and split each cell into a list variable but incorporating the `split()` method to delimit the commas.
I then used a regular expression to exract the floats from the strings.
```python
        lat_lon = []
        with open('colliders.csv') as f:
            lat_lon = (f.readline()).split(",")
        lat0 = float(re.findall(r'-?\d+\.?\d*', lat_lon[0])[1])
        lon0 = float(re.findall(r'-?\d+\.?\d*', lat_lon[1])[1])
        print("Lat: {0}, Lon: {1}".format(lat0,lon0))
```


#### 2. Set your current local position
Here as long as you successfully determine your local position relative to global home you'll be all set. Explain briefly how you accomplished this in your code.

Here I used the Drone.frame_utils method `global_to_local` to convert from lat lon at to NED.
```python
        self.set_home_position = (lon0, lat0, 0)
        start_pos = global_to_local(self.global_position,self.global_home)
```

#### 3. Set grid start position from local position
This is another step in adding flexibility to the start location. As long as it works you're good to go!

To get the start position I used the NED position mentioned above and offeset using the grid offsets.
```python
   grid_start = (int(start_pos[0]) - north_offset, int(start_pos[1]) - east_offset)
```
Since I used A* Graph Search I then had to figure out the closest Node and I also re-introduced the altitude at this point.
```python
   node_start = np.linalg.norm(np.array(grid_start) - np.array(nodes), axis=1).argmin()
   g_start = tuple(nodes[node_start]) + (self.global_home[2],)
```

#### 4. Set grid goal position from geodetic coords
This step is to add flexibility to the desired goal location. Should be able to choose any (lat, lon) within the map and have it rendered to a goal location on the grid.

First I created a class to make goals easier to store and randomize for testing. I flew the drone manually in the simulator to record the Lat, Lon coorodinate for the goals which I decided to call PointsOfInterest. 
```python
class PointsOfInterest():
    def __init__(self):
        self.poi_dict = {
        "Mission Spear" : (float(-122.39314), float(37.79270), 5.0 ),
        "Washington Street" : (float(-122.40131), float(37.79666), 5.0 ),
        "California Street" : (float(-122.39624), float(37.79391), 5.0 ),
        "Sacramento Front" : (float(-122.39931), float(37.79471), 5.0 ),
        "Roof of building" : (float(-122.39876), float(37.79553), 136.0 )
        }

    def get(self, poi_name):
        return (self.poi_dict.get(poi_name))

    def get_random(self):
        key = random.choice(list(self.poi_dict.keys()))
        print("Destination: ",key)
        return (self.poi_dict.get(key))
```

Next I converted the points into local and then offset using grid offsets.  - this was still in 2D
```python
        goal = global_to_local(self.poi.get("Washington Street"), self.global_home)
        grid_goal = (int(goal[0]) - north_offset, int(goal[1]) - east_offset)
```
Since I used A* Graph Search I then had to figure out the closest Node and I also re-introduced the altitude at this point.
```python
  node_goal = np.linalg.norm(np.array(grid_goal) - np.array(nodes), axis=1).argmin()
  g_goal =  tuple(nodes[node_goal]) + (-1*(int(goal[2])),)
```
Lastly I check if there is a clear point from the end node and actual goal, if there is I pop this back on the path.
```python
  if len(path) > 0 and (path[-1] != grid_goal) and not(collision_check(path[-1], g_goal, grid)):
      print("Adding end position")
      path.append(g_goal)
```
Here is an image of the goal added to the last node.
![Goal Image](./misc/goal.png)

#### 5. Modify A* to include diagonal motion (or replace A* altogether)
Minimal requirement here is to modify the code in planning_utils() to update the A* implementation to include diagonal motions on the grid that have a cost of sqrt(2), but more creative solutions are welcome. Explain the code you used to accomplish this step.

Initially I implemented the diagonals but later moved to A*. For the diagonals I added the following code:
```python

    SOUTH = (1, 0, 1)
    NORTH_WEST = (-1, -1, np.sqrt(2))
    NORTH_EAST = (-1, 1, np.sqrt(2))
    SOUTH_WEST = (1, -1, np.sqrt(2))
    SOUTH_EAST = (1, 1, np.sqrt(2))
    
    #Then if valid actions
    if (x - 1 < 0 or y - 1 < 0) or grid[x - 1, y - 1] == 1:
        valid_actions.remove(Action.NORTH_WEST)
    if (x - 1 < 0 or y + 1 > m) or grid[x - 1, y + 1] == 1:
        valid_actions.remove(Action.NORTH_EAST)
    if (x + 1 > n or y - 1 < 0) or grid[x + 1, y - 1] == 1:
        valid_actions.remove(Action.SOUTH_WEST)
    if (x + 1 > n or y + 1 > m) or grid[x + 1, y + 1] == 1:
        valid_actions.remove(Action.SOUTH_EAST)

```
Here is an image of the A-Star to the top of a building
![A Star Image](./misc/a_star.png)

#### 6. Cull waypoints 
For this step you can use a collinearity test or ray tracing method like Bresenham. The idea is simply to prune your path of unnecessary waypoints. Explain the code you used to accomplish this step.

Firstly I converted my grid to a 2.5D grid. This would allow be to check altitude. I used this to check if two waypoint could be connected even if there was an obstacle between them - just lower.
Using Bresenham I found the cells between the points I wanted to connect and then check the average altidude of those points agains the altitude recorded on my grid.

```python
#This is used to detect an obstacle between two waypoints
def collision_check(p1,p3,grid):
    alt = int((p3[2] + p1[2]) / 2)
    cells = list(bresenham(int(p1[0]), int(p1[1]), int(p3[0]), int(p3[1])))
    collision = False
    for cell in cells:
        if grid[cell] > alt:
            return True
    return collision

#Further reduce waypoints if there are no obstacles between them 
def prune_path(path,grid):
    pruned_path = [p for p in path]
    i = 0
    while i < len(pruned_path) - 2:
        p1 = point(pruned_path[i])
        p2 = point(pruned_path[i+1])
        p3 = point(pruned_path[i+2])
        
        if not collision_check(p1[0], p3[0], grid):
            pruned_path.remove(pruned_path[i+1])
        else:
            i += 1
    return pruned_path
```
I also implemented a routine to gradually increase altitude as I tried to land on roof tops in response to Sebastain's challenge.
```python
# Add altitude to the cordinates. This method attempts to add the altitude gradually.
def add_altitude_gradient(path, start, goal,min_alt=5, max_alt=200):

    if len(path) == 0:
        return path

    alt = int(np.clip((goal[2] - start[2]), min_alt, max_alt))
    alt_increment = int((alt+5) / (len(path) - 1))
    #print(alt, len(path), alt_increment)
    if (alt_increment < 1):
        alt_increment = 1
    altitude = (5,)

    path_with_alt = []

    for p in path:
        path_with_alt.append(p + altitude)
        altitude = (int(np.clip((altitude[0]+alt_increment,), 5, alt)),)

    return path_with_alt
```

Here is an image showing the reduced waypoints after culling
![Pruning Image](./misc/pruning.png)

### Execute the flight
#### 1. Does it work?
It works!

### Double check that you've met specifications for each of the [rubric](https://review.udacity.com/#!/rubrics/1534/view) points.
  


