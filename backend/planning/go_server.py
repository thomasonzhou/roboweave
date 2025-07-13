# Copyright 2022 DeepMind Technologies Limited
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import mujoco
from mujoco_mpc import agent as agent_lib
from mujoco_mpc import mjpc_parameters
import pathlib
import time
import math

model_path = (
    pathlib.Path(__file__).parent.parent
    / "robot/tasks/quadruped/task_flat.xml"
)
model = mujoco.MjModel.from_xml_path(str(model_path))

CIRCLE = True  # Set to True for circular motion, False for static goal
# Run GUI
with agent_lib.Agent(
    server_binary_path=pathlib.Path(agent_lib.__file__).parent
    / "mjpc"
    / "ui_agent_server",
    task_id="Quadruped Flat",
    model=model,
    extra_flags=["--planner_enabled"]
) as agent:
  print(f"Current mode: {agent.get_mode()}")
  print(f"Available modes: {agent.get_all_modes()}")
  print(f"Current task parameters: {agent.get_task_parameters()}")



  MODES = {'Quadruped', 'Flip'}
  TARGETS = {"Home", "Circle"}

  target = "Circle"

  state = agent.get_state()

  def get_state():
    state = agent.get_state()
    return state
  
  def flip():
    agent.set_mode("Flip")
    time.sleep(1.0)
  
  def set_goal(x, y):
    agent.set_mocap({"goal": mjpc_parameters.Pose(pos=[x, y, 0.26], quat=[1, 0, 0, 0])})

  def stay_still():
    agent.set_mode("Quadruped")
    state = get_state()
    set_goal(state.qpos[0] +0.3, state.qpos[1])

  def circle(run_time=None):
    if run_time is None:
      run_time = 10000

    start_time = time.time()
    while time.time() - start_time < run_time:
      current_time = time.time() - start_time
            
      # Create a circular motion for the goal, counter-clockwise
      radius = 3.7
      x = radius * math.cos(-current_time * 0.5)
      y = radius * math.sin(-current_time * 0.5)
      z = 0.26
      set_goal(x, y)

  while True:
    # stay_still()
      # flip()
      circle(1000000000000)
