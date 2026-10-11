================================================================================
PATHRAGE / VisionNav - MERGED PHASE PLAN (v2)
SIH 2026 | PS 26126 | Team 194940 | Bharat Electronics Limited
Vision-based autonomous navigation of a UGV in GPS-denied outdoor terrain
ROS 2 Humble, Gazebo Classic 11, Ubuntu 22.04. Stereo camera + IMU + wheel odom.
No LiDAR, no GPS.
================================================================================

This file merges two plans:
  - the 07 Oct plan (day estimates, exit-test targets, Phase 10, decisions), and
  - the current-state plan (repo audit of pathrage-main5.zip, viewer sync,
    ROS-free testable code, honest-noise EKF test).
Written from: PS_26126.pdf, the 6-slide idea submission, README.md,
docs/INTERFACES.md and a read of the repo. NOTHING IN THIS PLAN WAS RUN.
"Done" below means "the code/doc says so", to be confirmed in Phase 0.


--------------------------------------------------------------------------------
A. WHAT YOU ARE JUDGED ON
--------------------------------------------------------------------------------
PS 26126 asks for three capabilities and three modules:

  Challenge              Expected module                Your answer (idea PDF)
  --------------------   ----------------------------   --------------------------
  Path detection         Perception AI (lightweight)    Tier 1 depth + Tier 2 Fast-SCNN
  Visual localization    Visual SLAM / odometry         ORB-SLAM3 + EKF
  Collision avoidance    Planner -> wheel/motor cmds    Nav2 global + local planner

SUCCESS CRITERION: collision-free navigation from Point A to Point B across
outdoor scenarios (plural), including sudden obstacles.

CORE IDEA (keep it): two-tier perception.
  Tier 1 = geometry from stereo depth, no learning = safety floor.
  Tier 2 = learned terrain segmentation, may only ADD cost when confident.
  Tier 2 uncertain -> fall back to Tier 1. Safety never depends on the AI.


--------------------------------------------------------------------------------
B. CURRENT STATE (from the repo)
--------------------------------------------------------------------------------
DONE (per code and docs)
  - URDF (0.60 x 0.40 m skid steer), Gazebo spawn, teleop.
  - Stereo camera 640x480, 30 Hz, baseline 0.12 m, f ~ 381 px.
  - IMU /camera/imu (200 Hz); wheel /odom; /ground_truth/odom (evaluation only).
  - Stereo depth (stereo_image_proc): /points2, /disparity, image_rect.
  - RTAB-Map: /map and map->odom (stereo + wheel odom).
  - ORB-SLAM3 + zang09 wrapper built; third_party patch exists.
  - 3D browser viewer (ugv_viewer). 3 worlds + tools/make_world.py.
  - docs/INTERFACES.md contract; full_stack.launch.py (slam, vision, viewer,
    nav, world, x, y, yaw args).
NOT DONE (stubs)
  - localization_node (passthrough), depth_obstacle_node, segmentation_node,
    costmap_fusion_node, vision_costmap_layer (stub, probably unused),
    nav2_params.yaml (29 lines), training/*.py (~30 lines), EKF untuned
    (publish_tf false).
  - ORB-SLAM3 publishes NO pose (stock wrapper).

PROBLEMS IN THE REPO / SLIDES (resolve in Phase 0)
  1. Idea PDF: "stereo-inertial SLAM", cites VINS-Fusion. Code: stereo-only
     ORB-SLAM3; the wrapper patch removes the stereo-inertial target.
  2. Idea PDF: "A*/D* Lite" and "TEB". Repo: NavFn A* and MPPI (TEB is not a
     Humble apt package; Nav2 has no D* Lite plugin).
  3. Idea PDF: "GPU [model]" placeholder; "Fast-SCNN / BiSeNetV2" undecided.
  4. Two ORB-SLAM3 yamls disagree (Rectified + Stereo.b vs PinHole +
     Stereo.T_c1_c2) and a .bak has different feature settings.
  5. Focal length 381.347 (yamls) vs 381.36 (INTERFACES.md). Read camera_info.
  6. Gazebo IMU orientation is the TRUE orientation; wheel odom is almost
     perfect. Fusion proves nothing without injected noise (Phase 2).
  7. A Python node cannot be a Nav2 costmap layer (C++ pluginlib). Use Nav2's
     ObstacleLayer with PointCloud2 instead.
  8. Stereo depth error ~ z^2*dd/(f*b): ~0.27 m at 5 m, ~0.70 m at 8 m
     (0.5 px). Trusted range 0.8-6 m only.
  9. Sim-to-real gap for any RUGD/RELLIS-trained model on Gazebo images.
 10. README says the plan is kept outside the repo -> keep a copy in docs/PLAN.md.


--------------------------------------------------------------------------------
C. DECISIONS (record the answers at Gate 0)
--------------------------------------------------------------------------------
  D1 IMU            DECIDED: simulated IMU exists. Use gyro only in the EKF.
                    ORB-SLAM3 stays stereo-only; inertial mode = Phase 10 stretch.
  D2 Controller     MPPI or Regulated Pure Pursuit (TEB not on Humble apt).
  D3 Costmap path   Nav2 built-in ObstacleLayer fed by PointCloud2. Custom C++
                    layer only in Phase 10.
  D4 Segmentation   Fast-SCNN (smaller, easier to train/export). BiSeNetV2 only
                    as a fallback.
  D5 TF owners      map->odom: RTAB-Map. odom->base_footprint: Gazebo plugin
                    until Phase 2, then the EKF. Below base_footprint:
                    robot_state_publisher. Switch the odom TF owner TOGETHER:
                    URDF publish_odom_tf=false AND EKF publish_tf=true.
  D6 Ground truth   DECIDED: /ground_truth/odom (world -> base_link). Used only
                    by evaluation scripts, never by the robot.
  D7 Phase order    Strict numeric order (0..9). Optional reorder if time is
                    short: 0 -> 1 -> 3 -> 4 -> 2 -> 5 -> 8 -> 6 -> 7 -> 9
                    (first A->B demo on wheel odom, then upgrade localization;
                    Phase 2 must still finish before Phase 8).


--------------------------------------------------------------------------------
D. RULES FOR EVERY PHASE
--------------------------------------------------------------------------------
  - Phase ends with an EXIT TEST and a GATE. Stop at the gate and wait for
    your approval.
  - Change docs/INTERFACES.md first, then code. README stays true to the code.
  - Last task of each phase = verify (run tests, re-read diffs, check claims).
  - Pure maths in ROS-free modules with unit tests (like viewer_core.py).
  - Anything measurable gets a tools/ script or a test/ (ATE, rates, latency).
  - Seed all noise and scenarios. Keep results/ (CSV + plots) from the start.
  - Every phase that adds a topic or result adds it to the viewer. The viewer
    is a dev tool; nothing depends on it.
  - Never write "works" unless it was run; say "not run".
  - One owner per TF transform.
  - No new dependencies without telling you.


--------------------------------------------------------------------------------
E. PHASES
--------------------------------------------------------------------------------

=== PHASE 0 - BASELINE, VERIFY, CLEAN UP        (about 1 day; was 1-2 days) ===
Goal: a base where a fresh clone builds, the existing pipeline runs, and every
topic/frame/number is MEASURED, not assumed.

  0.1 Run the README end to end on your machine; record what works and fails.
  0.2 Verify claims in INTERFACES.md ("Gate 0" items still open):
        - /ground_truth/odom frame, rate (world -> base_link, 50 Hz)
        - camera_info: fx, cx, cy, baseline (fix 381.347 vs 381.36)
        - stereo depth error at 1, 3, 5, 8 m on a known object
        - view_frames: one publisher per transform today
        - /ros2 topic list matches INTERFACES.md
  0.3 Resolve the contradictions in section B: one ORB-SLAM3 yaml (delete or
      explain the .bak), slides story (stereo-only, A*, MPPI, model name).
  0.4 Confirm colcon build from a fresh clone; confirm setup_orbslam3.sh runs
      with the patch present.
  0.5 Add a scripted route driver (publishes /cmd_vel for a route at a set
      speed) and the three standard routes used in every later phase:
        R1 straight 20 m, R2 square 10x10 m, R3 figure-8.
  0.6 Add tools/record_bag.sh (bag the topics needed for evaluation).
  0.7 Write docs/PLAN.md (short copy of this plan). Update README to the true
      status; remove claims the code does not back.
  EXIT TEST: fresh clone -> colcon build -> one launch command starts Gazebo +
    stereo depth + RTAB-Map with zero errors; topic list matches INTERFACES.md;
    view_frames shows one publisher per transform; R1 runs scripted and the
    ground truth is recorded.
  GATE 0: you approve D1-D7, the measured numbers and the interface table.


=== PHASE 1 - ORB-SLAM3 POSE IN ROS           (about 2-3 days) ===============
Goal: ORB-SLAM3 tracks the robot from /stereo_camera/{left,right}/image_rect
and publishes its pose into ROS 2. (PS: visual localization.)

  1.1 Config: one yaml; fx = fy, cx, cy, 640x480, 30 fps, baseline 0.12 m in the
      format your build expects (Stereo.T_c1_c2 or Stereo.b); check against
      camera_info.
  1.2 Run the wrapper; confirm it initialises (needs texture and some motion;
      add textured objects if tracking fails).
  1.3 Patch stereo-slam-node.cpp: after TrackStereo publish PoseStamped on
      /orb_slam3/pose, SAME stamp as the image pair, frame "orb_map".
      ORB-SLAM3 returns Tcw (world->camera): publish its INVERSE Twc.
      Publish /orb_slam3/tracking_ok (true only when state == OK).
      Skip publishing when tracking is lost or the pose is empty.
  1.4 Regenerate third_party/orbslam3_ros2_humble.patch; update the setup script;
      rebuild with ONE compile job at a time.
  1.5 Rewrite localization_node.py -> /localization/slam_pose
      (PoseWithCovarianceStamped):
        - convert camera convention (z fwd, x right, y down) to ROS body frame
          at base_footprint using the TF extrinsics
          (stereo_camera_link -> base_footprint);
        - metric scale from stereo: no scale estimation;
        - pose relative to the first accepted frame, expressed in odom
          (compose with the initial odom->base_footprint at initialisation);
        - covariance start: x,y = 0.05^2, yaw = 0.03^2; inflate when
          tracking_ok is false or the pose jumps more than a threshold;
        - drop messages when tracking is not OK;
        - maths in a ROS-free module with unit tests.
  1.6 tools/eval_localization.py: records /localization/slam_pose and
      /ground_truth/odom, time-aligns, rigid SE(2) alignment, prints ATE RMSE
      and drift per 10 m. Unit-test the maths on synthetic trajectories.
  1.7 full_stack.launch.py: orbslam:=true (default false); run_orbslam3.sh
      checks the pose topic exists.
  1.8 Textured worlds only (offroad_sparse, offroad_forest). Drive R1, R2, R3.
      Record the speed and turn rate at which tracking is lost: this sets the
      Nav2 speed limit in Phase 4.
  VIEWER: ORB-SLAM3 trajectory (cyan), ground truth (green), driven path
    (yellow); tracking OK/LOST indicator; live position error. Optional topics
    /localization/slam_pose, /ground_truth/odom, /orb_slam3/tracking_ok as JSON
    text messages (binary layout unchanged). Update INTERFACES.md.
  EXIT TEST: pose visible in the viewer and following the robot; eval script
    prints an ATE for each of R1, R2, R3.
  GATE 1: ATE well under 5% of distance travelled.


=== PHASE 2 - EKF FUSION AND HONEST TF TREE    (about 1-2 days) ==============
Goal: one fused pose, correct TF tree, graceful fallback.

  2.1 Honest test: odom_noise_node republishes /odom as /odom_noisy with seeded
      velocity-scale error (3-5%, wheel slip), yaw-rate bias and Gaussian noise;
      covariances set to match what you inject. Launch arg odom_noise:=true.
      Clean /odom stays untouched as reference.
  2.2 TF switch (together): URDF publish_odom_tf=false, ekf publish_tf=true;
      world_frame odom, odom_frame odom, base_link_frame base_footprint.
      Update INTERFACES.md section 2.
  2.3 EKF inputs (15-value order x y z roll pitch yaw vx vy vz vroll vpitch vyaw
      ax ay az):
        odom0 = /odom_noisy              vx, vyaw
        pose0 = /localization/slam_pose  x, y, yaw, differential false, with a
                                         rejection threshold (bad jumps
                                         rejected, not trusted)
        imu0  = /camera/imu              angular velocity ONLY, no orientation
        two_d_mode true (ramps are projected to 2D; note it in the docs)
        output /odometry/filtered
      Tune process noise against R1-R3, not by feel.
  2.4 RTAB-Map consumes /odometry/filtered (remap odom); keep frame_id
      base_footprint; confirm RTAB-Map is the only publisher of map->odom.
  2.5 Fallback + health: /localization/status JSON
      {"mode": "fused"|"degraded", "slam_ok": bool, "age_s": ...} with a
      slam_pose_timeout parameter (default 1 s). If ORB-SLAM3 is lost, the EKF
      keeps running on wheel + gyro and status becomes "degraded".
  2.6 Results table for the slides: ATE for (a) noisy wheel odom only,
      (b) ORB-SLAM3 only, (c) fused, on R1-R3 (CSV + markdown). Also: kill
      ORB-SLAM3 mid-route; fused pose keeps going; status flips to "degraded".
  2.7 Save view_frames output to docs/.
  VIEWER: GT green, fused yellow (main), noisy wheel odom red (dim/dashed),
    ORB-SLAM3 cyan; status line fused/degraded; running error vs GT;
    fixed_frame stays odom.
  EXIT TEST: one publisher per transform; fused ATE <= the better single source
    on all three routes; killing ORB-SLAM3 crashes nothing.
  GATE 2: you approve the table and the TF tree.


=== PHASE 3 - TIER 1: DEPTH OBSTACLE DETECTION  (about 2-3 days) ==============
Goal: deterministic, no-ML obstacle layer from stereo depth = safety floor.

  3.1 depth_obstacle_node: read /points2, transform to base_footprint with tf2.
  3.2 Ground plane: known camera height and pitch first; RANSAC later for slopes.
  3.3 Keep range 0.8-6 m. Height > 0.15 m = obstacle; < -0.20 m = drop-off/ditch.
      Voxel downsample. Publish /perception/tier1_obstacles (PointCloud2,
      base_footprint) + a debug marker.
  3.4 Share the maths with ugv_viewer/viewer_core.py in a ROS-free module + tests.
  3.5 Tune on the three worlds: rocks detected; flat ground and gentle ramps
      give no false obstacles (check the 0.06 m ground-clearance limit).
  3.6 Measure latency, rate, detection range, false-positive rate. Target >= 10 Hz.
      NOTE: /points2 was measured at only ~8-9 Hz in Phase 0 (stereo_image_proc on
      CPU). Tier 1 cannot beat that. Either speed stereo up (smaller disparity range,
      speckle filter, lower resolution, RTAB-Map off while profiling) or set the
      target to the real /points2 rate and say so.
  3.7 Optional: widen the baseline (0.12 -> 0.20-0.30 m) if range accuracy
      limits you. If you do, re-run the Phase 1 checks.
  VIEWER: map colouring switches from the height preview to the real Tier-1
    classes; preview-vs-Tier-1 toggle; detection range ring and rate.
  EXIT TEST: drive at a rock, Tier-1 points appear at the right place; open
    ground gives no false obstacles; >= 10 Hz.
  GATE 3: you approve detection range and false-positive rate.


=== PHASE 4 - NAV2, TIER 1 ONLY               (about 3-4 days) ================
Goal: robot drives A -> B on its own. First complete PS-satisfying demo.
PROTECT THIS PHASE; it is the minimum viable demo.

  4.1 Full nav2_params.yaml: bt_navigator, controller_server, planner_server
      (NavFn A*), behavior server, velocity smoother, lifecycle manager.
  4.2 Local costmap: rolling window, ObstacleLayer fed by
      /perception/tier1_obstacles, inflation for the 0.60 x 0.40 m footprint.
      Global costmap: RTAB-Map /map, allow unknown space.
  4.3 Controller per D2 (MPPI or Regulated Pure Pursuit). Velocity limit from
      the Phase 1 tracking-loss speed; slow down when status is "degraded";
      skid-steer turning behaviour.
  4.4 Goals: viewer double-click (/goal_pose) and an action-client script that
      sends A -> B waypoints by coordinates.
  4.5 Online-map problem: the global map starts empty. Verify the robot does not
      plan through unseen obstacles. Backup if RTAB-Map lags: rolling
      odom-frame global costmap.
  4.6 Recoveries (spin, back up, clear costmap) tuned; test stuck situations.
  4.7 full_stack.launch.py nav:=true works: sim + vision + localization + Nav2.
  VIEWER: Nav2 plan and goal already exist; optional local-costmap footprint /
    inflation layer.
  EXIT TEST: in offroad_sparse, goal B reached without touching a rock in 5/5.
  GATE 4: you approve the recorded run.


=== PHASE 5 - WORLDS AND SCENARIOS            (about 2-3 days) ================
Goal: "outdoor scenarios" plural and "sudden obstacles", as the PS requires.

  5.1 Worlds: (a) sparse rocks, (b) dense rocks + trees, (c) ditch/slope
      (heightmap), (d) mixed terrain (grass/dirt/mud-coloured patches).
  5.2 SUDDEN obstacle: model that appears or moves into the path at a trigger
      time or position (plugin or spawn-service script).
  5.3 Lighting variants (sun angle/intensity) for the slide-4 lighting claim.
  5.4 3 start/goal pairs per world (short, medium, long; 10-40 m).
  5.5 tools/run_scenarios.py: launches a world, sends the goal, logs, resets,
      repeats N times, writes a CSV unattended (becomes the Phase 8 harness).
      Collisions counted from ground truth, not from the robot's belief.
  5.6 Make sure worlds are textured enough for ORB-SLAM3 and RTAB-Map.
  VIEWER: optional replay of a logged run.
  EXIT TEST: runner executes all worlds unattended and writes a results CSV.
  GATE 5: you approve the scenario set.


=== PHASE 6 - TIER 2: TERRAIN SEGMENTATION    (about 5-7 days) ================
Goal: lightweight learned model that adds semantics geometry cannot see (mud vs
dirt, grass vs brush). HIGHEST RISK; comes after a working system exists. If it
fails the project still works on Tier 1. Do at least a minimal, honest version:
the PS asks for a lightweight AI model.

  6.1 Datasets: RUGD and RELLIS-3D (check licence and size first). Map classes
      to 0 unknown / 1 traversable / 2 caution / 3 hazard; document the mapping.
  6.2 training/train_segmentation.py for real: loader, Fast-SCNN (D4),
      cross-entropy + class weights, augmentation incl. lighting/colour jitter.
  6.3 Offline eval on a held-out split: mIoU, per-class IoU, confusion matrix,
      latency.
  6.4 SIM-TO-REAL (do not skip), pick one at the start of the phase:
        Plan A: realistic world textures + fine-tune on a small set of sim
                frames labelled from known object/patch locations.
        Plan B: show Tier 2 on real RUGD/RELLIS frames replayed as a topic/bag;
                sim used only for Tier 1 + navigation.
      You may need both.
  6.5 Export ONNX (training/export_onnx.py); verify ONNX output matches PyTorch.
  6.6 segmentation_node with onnxruntime (CUDA if available) + per-pixel
      CONFIDENCE (softmax max) -> /perception/tier2_terrain and
      /perception/tier2_confidence.
  6.7 Project the class mask onto the ground plane using calibration + depth
      (share geometry code with Phase 3).
  VIEWER: terrain classes drawn on the ground.
  EXIT TEST: mIoU >= 0.50 on merged classes (propose; set at Gate 5); node >= 8 Hz;
    mask visibly correct on real frames and reasonable on sim frames.
  GATE 6: you approve mIoU, latency and the sim-to-real approach. If it fails,
    use the cut line.


=== PHASE 7 - FUSION AND FALLBACK LOGIC       (about 2-3 days) ================
Goal: implement the slide-3 core idea literally.

  7.1 costmap_fusion_node: Tier 1 obstacles are never removed by Tier 2. Tier 2
      only ADDS cost (hazard terrain) where confidence is above a threshold.
  7.2 Low confidence -> publish nothing extra (Tier 1 only); log the fallback rate.
  7.3 Publish /perception/obstacle_points (or a costmap input) for Nav2 (D3).
  7.4 Ablation launch switches: tier1_only | tier2_only | fused.
  EXIT TEST (both must pass):
    a) a FLAT hazard-terrain patch (Tier 1 is blind) is avoided in fused mode
       and driven through in tier1_only mode;
    b) an obstacle that Tier 2 misses (or low-confidence Tier 2 output) is still
       avoided in fused mode.
  GATE 7.


=== PHASE 8 - EVALUATION AND BENCHMARKS       (about 2-3 days) ================
Goal: numbers that back every claim in the slides.

  8.1 Phase 5 runner: all worlds x all start/goal pairs x 10 runs x modes
      (tier1_only / tier2_only / fused).
  8.2 Metrics per run: success (within tolerance), collisions (ground truth /
      contact), time, path length vs straight line, minimum clearance, replans,
      recoveries, localization ATE, perception rate and latency.
  8.3 Stress tests: sudden obstacle, lighting extremes, injected ORB-SLAM3
      tracking loss, segmentation off.
  8.4 Tables and plots (success per mode, ATE per route, latency) from ONE command.
  8.5 Honest limitations section (sim only, stereo range, 2D EKF, etc.).
  EXIT TEST: results folder with CSVs + plots reproducible by one command.
  GATE 8: you approve the headline numbers (PS success-criterion evidence).


=== PHASE 9 - PACKAGING AND SUBMISSION        (about 2-3 days) ================
  9.1 README rewrite: quick start, architecture, results, limitations, true status.
  9.2 Demo video: viewer + Gazebo + the sudden-obstacle run.
  9.3 Update the architecture diagram to what actually runs.
  9.4 Slide fixes: stereo-only (not inertial), A* (not D* Lite), MPPI (not TEB),
      the real segmentation model, real compute hardware (no placeholder), drop
      VINS-Fusion from references if unused, replace "implementation is the next
      phase" with real numbers. Keep the SIH template.
  9.5 Fresh-clone test on a clean Ubuntu 22.04 / VM, README only.
  9.6 Licence note (ORB-SLAM3 GPLv3, built outside the repo). Tag a git release.
  EXIT TEST: someone else clones, follows the README, reproduces the demo.
  GATE 9: you approve the submission package.


=== PHASE 10 (OPTIONAL / STRETCH) - EMBEDDED READINESS   (about 3-4 days) ====
  - Export Tier 2 to TensorRT; benchmark on a Jetson-class device if available
    (otherwise report estimates clearly labelled as estimates).
  - Fill the "[model]" placeholder on slide 3 with hardware you can justify.
  - ORB-SLAM3 stereo-inertial mode using the simulated IMU.
  - Custom C++ costmap layer instead of ObstacleLayer.
  - D* Lite planner plugin (only if you keep that slide claim).


--------------------------------------------------------------------------------
F. SCHEDULE AND CUT LINE
--------------------------------------------------------------------------------
  Phase 0  Baseline + cleanup          ~1 d
  Phase 1  ORB-SLAM3 pose              2-3 d
  Phase 2  EKF fusion                  1-2 d
  Phase 3  Tier 1 depth                2-3 d
  Phase 4  Nav2 A->B (Tier 1 only)     3-4 d   <-- first complete demo
  Phase 5  Worlds + scenarios          2-3 d
  Phase 6  Tier 2 segmentation         5-7 d
  Phase 7  Fusion + fallback           2-3 d
  Phase 8  Evaluation                  2-3 d
  Phase 9  Packaging                   2-3 d
  Total (no Phase 10)                  ~22-32 working days (rough, one person,
                                       real Ubuntu machine; your SIH deadline
                                       is not known, so these are estimates)

  IF TIME IS SHORT
    - Minimum credible submission = Phases 0-5 + 8 + 9, Tier 1 only. Meets all
      three PS items and the success criterion; Tier 2 stated as future work.
    - Add next: Phase 6 Plan B (offline demo on RUGD/RELLIS frames), then Phase 7.
    - Cut first: Phase 10, custom C++ layer, D* Lite, inertial SLAM.


--------------------------------------------------------------------------------
G. RISKS
--------------------------------------------------------------------------------
  Risk                                       Phase   Mitigation
  ----------------------------------------   -----   ------------------------------
  ORB-SLAM3 loses tracking (low texture,     1,5     Textured worlds; speed limit from
  fast turns)                                        1.8; EKF wheel+gyro fallback (2.5)
  Stereo depth noisy beyond ~5-6 m           3       Range cap; optional wider baseline;
                                                     inflation tuned for error
  Fusion looks pointless (sim odom perfect)  2       Inject seeded noise/slip (2.1)
  RTAB-Map /map empty or lagging early       4       Local costmap primary for avoidance;
                                                     unknown-space test (4.5); rolling
                                                     global costmap backup
  Segmentation does not transfer to sim      6       Plan A/B (6.4); Tier 1 keeps safety
  Dataset licence / size / training time     6       Check early; subset; cloud GPU
  Compute claim ("embedded GPU") unbacked    6,9,10  Measure ONNX FPS or state "PC sim"
  2D EKF flattens ramps                      2       Document it; test offroad_forest
  TF/frame conflicts                         2       One-owner rule (D5); view_frames
                                                     at every gate
  Controller (TEB) unavailable               4       MPPI / Regulated Pure Pursuit (D2)
  CPU/GPU load of everything together        all     Headless Gazebo (gui:=false); lower
                                                     camera rate; one compile job;
                                                     measure early
  Scope creep / time                         all     Follow the cut line; gates stop
                                                     half-done phases


--------------------------------------------------------------------------------
H. DELIVERABLES CHECKLIST
--------------------------------------------------------------------------------
  [ ] One-command simulation launch; fresh clone builds
  [ ] ORB-SLAM3 pose + fused EKF pose; ATE table (3 sources x 3 routes)
  [ ] Tier-1 detection with measured range and false-positive rate
  [ ] Nav2 A->B in 3+ worlds, sudden obstacle handled
  [ ] Tier-2 model: mIoU, FPS, ONNX file
  [ ] Ablation: tier1_only / tier2_only / fused
  [ ] Benchmark CSV + plots from one command
  [ ] README, architecture diagram, demo video, slides matching the code
  [ ] Fresh-clone test passed

NEXT STEP: approve Phase 0 and answer D7 (strict order, or the reorder).
================================================================================
