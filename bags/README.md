# Bag Files

  All result bags were recorded while our particle filter (`pf.py`) ran on the provided
  MAC 1st floor bags, using the map `maps/mac_1st_floor_final.yaml`.

  Each result bag contains: `/particle_cloud`, `/scan`, `/tf`, `/tf_static`, `/map`,
  `/initialpose`, `/clock`. Results were measured with `scripts/pf_metrics.py`, and the
  data for each bag are in `results/`.

  ## How to replay a result bag

   ```bash
  rviz2 -d rviz/turtlebot_bag_files.rviz
  ros2 bag play bags/pf_result_take1_run1 --clock
  ```

  ## Input bags

  We used the provided bags as inputs:

  - **`macfirst_floor_take_1`**: Turtlebot driving about 20 m through the MAC 1st floor 
  hallway.
  - **`macfirst_floor_take_2`**: a shorter drive through a different part of the hallway.

  ## Result bags

  ### `pf_result_take1_run1`: normal run on take 1

  **How it was collected:** Played `macfirst_floor_take_1` through our filter. Set the
  2D Pose Estimate once at the robot's correct starting position, right after starting
  the bag.

  **Results:**
  - Converged after **8 updates (9.1 s)** and stayed converged for the whole bag.
  - Main cluster spread averaged **0.22 m** (max 0.33 m), heading spread about **6°**.
  - The laser scan lined up with the walls for the whole run.

  **What we learned:** With a correct starting pose, the filter tracks reliably over a
  long route. We also use this run as the reference for measuring error in the kidnapping
  test.

  ### `pf_result_take2_run1`: normal run on take 2

  **How it was collected:** Played `macfirst_floor_take_2` through our filter. Set the
  2D Pose Estimate once at the correct start, around (6.9, −9.0) in the map.

  **Results:**
  - Converged after **7 updates (12.7 s)** and stayed converged until the bag ended.
  - Main cluster spread averaged **0.22 m** (max 0.33 m), heading spread about **6°**.
  - The convergence was not as stable in this take as in take 1, but it was still quite good throughout the run. 

  **What we learned:** The filter works on both takes. An earlier attempt on
  this bag failed because the initial pose was clicked about 11 m from the robot's real
  position. The filter was not able to recover and converge. This shows a reasonable starting guess matters.

  ### `pf_result_take1_kidnap`: fake kidnapping test on take 1

  **How it was collected:** Played `macfirst_floor_take_1`. Set the correct starting
  pose, let the filter converge, then simulated two kidnappings by clicking a **wrong**
  2D Pose Estimate mid-run. That throws away the good particle cloud and rebuilds it
  around the wrong spot:
  - **Kidnap #1 (~21 s):** clicked about **0.7 m** from the true position.
  - **Kidnap #2 (~51 s):** clicked about **2.8 m** from the true position.

  The error was measured against `pf_result_take1_run1` at the same moments in the bag.

  **Results:**

  | Kidnapping | Offset | Result | Updates to recover | Error after |
  |---|---|---|---|---|
  | #1 | 0.7 m | Recovered to the correct position | 22 (18.9 s) | 0.10–0.20 m |
  | #2 | 2.8 m | Locked onto the wrong spot | — | 7.4 m |

  **What we learned:** Our adaptive random injection recovers from small kidnappings.
  After the larger one, the cloud became tight again but **7.4 m away**, in a part
  of the hallway that looks the same to the laser.