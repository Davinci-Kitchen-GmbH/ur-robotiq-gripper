# ur-robotiq-gripper
A simple and easy to use python module to control the Robotiq Hand-E gripper attached to a Universal Robots robot arm.
This driver depends on a installed Robotiq grippers URCap on the UR arm.

The code was tested on Robotiq Hand-E and was originally developed for the Gripper 2F-85. Since the 2F-140 is controlled just as the 2F-85 it is save to assume that the code works for all these 3 types of Robotiq grippers.

The code was originally written by the team of [SDU Robotics](https://gitlab.com/sdurobotics) and adapted by us to use with asyncio. You can find further thoughts and explanations regarding the control of Robotiq grippers attached to a UR arm on their [Website](https://sdurobotics.gitlab.io/ur_rtde/guides/guides.html#use-with-robotiq-gripper).

## Usage

```
import asyncio
from ur_robotiq_gripper import Gripper

async def log_info(gripper):
    print(f"Pos: {str(await gripper.get_current_position()): >3}  "
          f"Open: {await gripper.is_open(): <2}  "
          f"Closed: {await gripper.is_closed(): <2}  ")

async def run():
    gripper = Gripper('10.10.2.2')  # actual ip of the ur arm

    await gripper.connect()
    await gripper.activate()  # calibrates the gripper

    await gripper.move_and_wait_for_pos(255, 255, 255)
    await log_info(gripper)
    await gripper.move_and_wait_for_pos(0, 255, 255)
    await log_info(gripper)

asyncio.run(run())
```

