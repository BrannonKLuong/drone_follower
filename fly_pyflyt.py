import numpy as np
import asyncio
from pyflyt.core import Aviary

async def run_mission():
    # Step 1-3 from docs: Instantiate the aviary
    # We start the drone at Z=0.2 so it's on the ground
    start_pos = np.array([[0.0, 0.0, 0.2]])
    env = Aviary(start_pos=start_pos, render=True)

    print("Simulation started.")

    # Step 4 from docs: Set the flight mode for the first drone (index 0)
    # Mode 7 is Position Control mode.
    print("Setting mode to Position Control.")
    env.set_mode(7)

    # --- Takeoff Phase ---
    # We will tell the drone to go to a setpoint 20m above the ground
    print("Commanding takeoff to 20 meters.")
    takeoff_setpoint = np.array([0.0, 0.0, 20.0, 0.0]) # Target: [x, y, z, yaw]
    
    # Step 5 from docs: Set the setpoint for the drone
    env.set_setpoint(0, takeoff_setpoint)

    # Step the simulation for a few seconds to let it take off and stabilize
    for i in range(360):  # 3 seconds at 120hz control rate
        await env.step()

    # --- Fly Forward Phase ---
    # Now we command the drone to a new setpoint 1000m forward (on the x-axis)
    print("Commanding drone to fly 1000m forward.")
    forward_setpoint = np.array([1000.0, 0.0, 20.0, 0.0]) # Target: [x, y, z, yaw]
    env.set_setpoint(0, forward_setpoint)

    # Let the simulation run for a long time to allow the drone to reach the target
    # We can monitor its progress by printing its state
    for i in range(2400): # 20 seconds
        await env.step()
        if i % 120 == 0: # Print status every second
            # Get the state of the first drone using the state() method from the docs
            state = env.state(0)
            position = state[3] # state[3] is the ground frame linear position [x, y, z]
            print(f"Current Position: X={position[0]:.2f}, Y={position[1]:.2f}, Z={position[2]:.2f}")

    # --- Landing Phase ---
    # To land, we simply set the target Z position to a low value
    print("Commanding drone to land.")
    land_setpoint = np.array([1000.0, 0.0, 0.2, 0.0]) # Target: [x, y, z, yaw]
    env.set_setpoint(0, land_setpoint)

    # Step the simulation to allow it to land
    for i in range(600): # 5 seconds
        await env.step()

    print("Mission Complete.")
    await asyncio.sleep(5) # Pause to admire the landing
    
    # Step 6 from docs: Gracefully close
    await env.close()

if __name__ == "__main__":
    asyncio.run(run_mission())