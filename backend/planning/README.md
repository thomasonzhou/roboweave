# MCP (Model Context Protocol) for MPC (Model Predictive Control)

stand_up_joint_pos = np.array([
    0.00571868, 0.608813, -1.21763, -0.00571868, 0.608813, -1.21763,
    0.00571868, 0.608813, -1.21763, -0.00571868, 0.608813, -1.21763
],
                              dtype=float)

stand_down_joint_pos = np.array([
    0.0473455, 1.22187, -2.44375, -0.0473455, 1.22187, -2.44375, 0.0473455,
    1.22187, -2.44375, -0.0473455, 1.22187, -2.44375
],
                                dtype=float)

Mujoco MPC build with clang 17, quadruped walk

## Notes

`go_server.py` will only work in an environment with python support for MuJoCo MPC

Activate with something like:
```sh
source ~/src/mujoco_mpc/.venv/bin/activate
```