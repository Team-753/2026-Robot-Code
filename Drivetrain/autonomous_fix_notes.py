# This file is documentation only.
# It exists to capture robot-specific context and explain the logic bugs that
# were fixed or intentionally left alone in the autonomous patches.
#
# Robot context
# -------------
# 1. Front of robot = intake side.
# 2. Back of robot = launcher / shooter side.
# 3. Gyro is believed to be mounted 180 degrees opposite the robot front.
# 4. Teleop field-oriented driving has had minimal problems.
#    Practical meaning:
#    - The drive team is already comfortable with the current field controls.
#    - "Forward on the joystick" already feels like "robot goes forward".
#    - Because teleop is mostly acceptable, do not casually remove hardware-
#      facing sign inversions just because the math looks unusual on paper.
#
# Canonical field-frame decision
# ------------------------------
# The autonomous cleanup treats blue-origin field coordinates as the shared
# frame for:
# - poseEstimator state
# - Choreo trajectory samples / initial poses
# - Limelight botpose_orb_wpiblue data
# - targeting calculations
#
# Why this matters:
# Before the patch, alliance-specific rotations were being applied in more than
# one layer. Choreo was already flipping red trajectories, and drivetrain code
# was also rotating red headings / poses. That double-handling created startup
# heading errors and made red debugging harder to reason about.
#
# Bug 1: autonomous startup heading was wrong
# -------------------------------------------
# Symptom:
# - Blue autos could begin with an immediate heading error even after resetting
#   to the trajectory's initial pose.
#
# Root cause:
# - The auto follower compared sample.heading against pose.rotation() + pi.
# - That extra pi shifted the robot heading by 180 degrees before control.
#
# Fix:
# - Compare the estimated robot heading directly to sample.heading.
# - If the estimator is reset to the Choreo initial pose, the first heading
#   error should be near zero.
#
# Files involved:
# - autonomousDriveSubsys.py
# - robotContainer.py
# - swerveSubsys.py
#
# Bug 2: alliance transforms were duplicated
# ------------------------------------------
# Symptom:
# - Auto behavior differed by alliance in ways that were larger than normal
#   path mirroring.
# - Dashboard / Field2d could disagree with what the controller was doing.
#
# Root cause:
# - getPoseState() rotated red pose by pi.
# - getRobotYaw() rotated red gyro by pi.
# - auto follower also asked Choreo for red-flipped samples.
# - auto output path also had alliance-specific sign branches.
#
# Fix:
# - Keep poseEstimator in one blue-origin field frame for both alliances.
# - Let Choreo own the red flip through sample_at(..., flipForRedAlliance)
#   and get_initial_pose(flipForRedAlliance).
# - Remove red-only pose/yaw rewrites from odometry-facing methods.
#
# Important caution:
# - This does NOT mean every minus sign in the drivetrain should be deleted.
# - Some signs are hardware adapters, not alliance logic.
#
# Bug 3: Targeting2 lead compensation never accumulated real velocity
# -------------------------------------------------------------------
# Symptom:
# - "Lead" targeting acted like plain targeting because velocity kept falling
#   back to zero.
#
# Root cause:
# - pastPosition / pastTime were local variables, not persistent command state.
# - Every execute() effectively started over.
#
# Fix:
# - Store last pose and timestamp on the command instance.
# - Compute velocity from current sample minus previous sample.
# - Keep that state alive across scheduler loops.
#
# File involved:
# - Targeting2.py
#
# Bug 4: auto shooting recreated the targeting command every loop
# ---------------------------------------------------------------
# Symptom:
# - Targeting could not settle consistently while shooting in auto.
#
# Root cause:
# - autoDriveTrainCommand.execute() built a new targetPointWithLeadCommand each
#   cycle, so PID state and velocity history were discarded immediately.
#
# Fix:
# - Keep one targetPointWithLeadCommand on the auto command instance.
# - Initialize it when shooting turns on.
# - Reuse it every loop while shooting remains on.
# - End it when shooting turns off or the auto command ends.
#
# Files involved:
# - autonomousDriveSubsys.py
# - Targeting2.py
#
# Bug 5: targeting needs to aim with the launcher, not the intake
# ---------------------------------------------------------------
# Robot-specific context:
# - The launcher is on the back of the robot.
# - The intake is the front of the robot.
#
# Practical meaning:
# - If the robot should shoot at a target behind it, the robot front heading
#   must usually be 180 degrees away from the target bearing.
#
# Current targeting assumption:
# - desiredFrontHeading = targetBearing + pi
#
# That assumption is correct ONLY if:
# - the launcher truly shoots out the back, and
# - the estimator heading represents the robot front direction.
#
# If future mechanical changes move scoring to the front:
# - remove the + pi offset from the targeting commands.
#
# Remaining "do not change blindly" items
# ---------------------------------------
# 1. swerveSubsys.py line near fromFieldRelativeSpeeds(..., -gyro)
#    Why left alone:
#    - Teleop field-relative driving already feels acceptable to the drivers.
#    - That minus sign may be compensating for a gyro mounted backwards or for
#      the sign convention returned by the sensor.
#    How to validate:
#    - Put robot on the field.
#    - Face it downfield.
#    - Push joystick forward.
#    - If it drives downfield correctly, the sign may be intentional.
#
# 2. autonomousDriveSubsys.py line near setState(vx, -vy, omega)
#    Why left partly intact:
#    - The drivetrain public API is fb/lr/rot, not a pure WPILib vx/vy API.
#    - The team's left/right sign convention may already be baked into
#      setState() usage across teleop and auto.
#    How to validate:
#    - Run one short blue auto with a lateral component.
#    - Run the same auto on red.
#    - If forward heading is now correct but lateral motion is mirrored, this
#      is the next place to inspect.
#
# Suggested test order on the real robot
# --------------------------------------
# 1. Verify teleop still feels normal:
#    - forward, backward, left, right, rotate
# 2. Verify auto start orientation on blue:
#    - robot should begin aligned with the previewed path
# 3. Verify the same path on red:
#    - mirrored path, but no extra 180-degree facing mistake
# 4. Verify manual targeting:
#    - launcher side should face the target
# 5. Verify auto targeting during a shooting event:
#    - command should hold state long enough to settle
#
# Short rule of thumb for future debugging
# ----------------------------------------
# - If the problem changes by alliance, check coordinate frames first.
# - If teleop is fine but auto is mirrored, check auto vx/vy sign handling.
# - If targeting points the intake at the goal, revisit the + pi launcher
#   offset.
# - If targeting "looks alive" but never leads, inspect persistent velocity
#   state.
