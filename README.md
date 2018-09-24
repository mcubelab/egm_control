# EGM Control for ABB irb120

### Testing steps
Common steps (alternatively, run script `run_example` in `pman`):

1. Load an instance of `roscore`.
2. Open the position/velocity controller (e.g. `example.py`) which will publish the desired position or velocity to the ROS topic `/commanded_pose`:

		rosrun egm_control example.py

For each testing session:

1. Stop all previous running programs (local Python programs and in FlexPendant, pressing the Stop button).
2. In FlexPendant, select _PP To Main_ and then press _Yes_.
3. Start the main program (`EGMControl.py`), either via `pman` or by typing:

		rosrun egm_control EGMControl.py
4. Press the Play button in the FlexPendant to start the RAPID code.

In order to cancel the execution of `EGMControl.py`, either:

- Press `Ctrl+C` on the terminal.
- Stop it via `pman`.
- Set ROS parameter `egm_status` to 0.
- Press the Stop button in the FlexPendant (not recommended).
