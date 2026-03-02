# F1Tenth Hackathon - Team latifi_the_goat 🐐

## Team Members
* Sarvajana Hari (CE/Y1) - @im_hariii
* ABHIRAM (CE/Y1)
* ARAVIND SIVAKUMAR (IEM/Y1)

## 📂 Where to find our code
We uploaded our entire ROS2 package so it can be built easily. 
* **Main Algorithm:** Navigate to `my_package/my_package/gap_finder.py` to see our actual logic.
* The root folder contains our `package.xml` and `setup.py`, so you can just drop this folder straight into your `ros2_ws/src` and run `colcon build`.

##  Our Algorithm & Thought Process
We originally tried a standard Follow the Gap algorithm, but during testing, we noticed the car kept clipping the inside walls on tight corners, especially at the final chicane. It was turning way too early into the apex.

To fix this, we completely switched to a **Disparity Extender** algorithm. 

**1. Corner Padding (The Fix for Wall Clipping):** Our code looks for sudden jumps in LiDAR depth (anything > 0.5m), which usually indicates an edge or a tight corner. We then artificially pad that edge by 0.50m (our car's physical width). This forces the car to take a wider, much safer racing line instead of diving into the wall.

**2. Dynamic Braking Curve:** We wanted to push the speed on the straights but survive the sharp turns. We mapped the steering ratio directly to the speed using a power-law curve. If the path ahead is clear for more than 8 meters and the wheels are relatively straight, we push the speed to 12.0 m/s. As the steering angle increases, the speed drops exponentially so the car can smoothly carry momentum without understeering off the track.

**3. The "Panic Brake" for the Final Chicane:**
Even with the disparity extender, pushing 12 m/s meant the car sometimes carried too much speed into the sharpest corners. Instead of making the car slow down throughout the entire track, we added a hard panic threshold. If the LiDAR detects a wall closer than 1.1 meters, the speed instantly drops to 2.0 m/s. This let us survive the tricky final chicane consistently without sacrificing straight-line speed.
