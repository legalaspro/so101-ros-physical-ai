#!/usr/bin/env python3
"""
SO-101 MoveIt Cartesian motion test (moveit_py API).

Uses the moveit_py API (PlanningComponent / MoveItPy) — the official
Python equivalent of the C++ MoveGroupInterface.

For visualisation, run the Rerun bridge separately:
  pixi run bridge-3d

Prerequisites:
  ros2 launch so101_bringup moveit_py_test.launch.py
"""
from __future__ import annotations

import os
import signal
import threading
import time

import rclpy
import rclpy.logging
from geometry_msgs.msg import Pose, PoseStamped
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy, MultiPipelinePlanRequestParameters
from tf_transformations import quaternion_from_euler

# ── Constants ────────────────────────────────────────────────────────
EE_FRAME = "gripper_frame_link"
PLANNING_GROUP = "manipulator"
BASE_FRAME = "world"


# ═════════════════════════════════════════════════════════════════════
#  plan_and_execute  –  helper (matches moveit_py tutorial pattern)
# ═════════════════════════════════════════════════════════════════════

def plan_and_execute(robot, planning_component, logger,
                     single_plan_parameters=None,
                     multi_plan_parameters=None):
    """Plan and execute a motion. Returns True on success."""
    logger.info("Planning trajectory")
    if multi_plan_parameters is not None:
        plan_result = planning_component.plan(
            multi_plan_parameters=multi_plan_parameters)
    elif single_plan_parameters is not None:
        plan_result = planning_component.plan(
            single_plan_parameters=single_plan_parameters)
    else:
        plan_result = planning_component.plan()

    if plan_result:
        logger.info("Executing plan")
        robot.execute(plan_result.trajectory, controllers=[])
        return True
    else:
        logger.error("Planning failed")
        return False


# ═════════════════════════════════════════════════════════════════════
#  Two-stage IK → joint-space planning (slobot parity)
# ═════════════════════════════════════════════════════════════════════

def make_pose_stamped(pos_mm, rpy_rad, frame_id=BASE_FRAME):
    """Position in mm + RPY in rad → geometry_msgs/Pose (metres + quat)."""
    q = quaternion_from_euler(*rpy_rad)
    ps = PoseStamped()
    ps.header.frame_id = frame_id
    ps.pose.position.x = pos_mm[0] / 1e3
    ps.pose.position.y = pos_mm[1] / 1e3
    ps.pose.position.z = pos_mm[2] / 1e3
    ps.pose.orientation.x = q[0]
    ps.pose.orientation.y = q[1]
    ps.pose.orientation.z = q[2]
    ps.pose.orientation.w = q[3]
    return ps


def solve_ik_and_plan(robot, arm, logger, pose, plan_params=None, ik_timeout=0.2):
    robot_model = robot.get_robot_model()
    
    # Copy current state (don't hold the scene lock)
    robot_state = RobotState(robot_model)
    with robot.get_planning_scene_monitor().read_only() as scene:
        current = scene.current_state
        robot_state.set_joint_group_positions(
            PLANNING_GROUP,
            current.get_joint_group_positions(PLANNING_GROUP),
        )
    robot_state.update()

    # Stage 1: IK seeded from current joints
    ok = robot_state.set_from_ik(PLANNING_GROUP, pose, EE_FRAME, ik_timeout)
    if not ok:
        logger.error(f"IK failed — pose unreachable (tried {ik_timeout}s)")
        return False

    robot_state.update()

    # Stage 2: Joint-space goal + plan
    robot_state.update()
    arm.set_start_state_to_current_state()
    arm.set_goal_state(robot_state=robot_state)
    return plan_and_execute(robot, arm, logger, multi_plan_parameters=plan_params)

def plan_linear_cartesian(robot, arm, logger, pose_stamped, plan_params):
    arm.set_start_state_to_current_state()
    arm.set_goal_state(
        pose_stamped_msg=pose_stamped,
        pose_link=EE_FRAME,
    )
    return plan_and_execute(robot, arm, logger, multi_plan_parameters=plan_params)


def main() -> None:
    # ── ROS 2 + MoveItPy ──
    rclpy.init()
    logger = rclpy.logging.get_logger("moveit_py.so101")

    logger.info("Creating MoveItPy instance…")
    robot = MoveItPy(
        node_name="moveit_py",
        # MoveItCpp hardcodes joint_state_topic to "joint_states" (const in
        # PlanningSceneMonitorOptions).  Remap so the namespaced topic is used.
        remappings={"joint_states": "/follower/joint_states"},
    )
    arm = robot.get_planning_component(PLANNING_GROUP)
    logger.info("MoveItPy instance created")

    # ── Named plan-request parameter sets (from moveit_py_config.yaml) ──
    ompl_params = MultiPipelinePlanRequestParameters(robot, ["ompl_rrtc"])
    pilz_lin_params = MultiPipelinePlanRequestParameters(robot, ["pilz_lin"])

    # ── Phase 1: Named joint-space goals (pipeline sanity check) ──
    named_goals = ["zero"] #"rest", "extended", "zero"]

    # ── Phase 2: Pose goals — two-stage IK → OMPL (joint-space) ──
    pose_goals = [
        ("FWD_HIGH", [390.0,    0.0, 220.0], [0.0, 1.57, 0.0]),
        ("DOWN",     [136.4,    0.0,  62.0], [0.0, 3.141, 0.0]),
        ("LEFT",     [136.4,  100.0,  62.0], [0.0, 3.141, 0.0]),
        ("RIGHT",    [136.4, -100.0,  62.0], [0.0, 3.141, 0.0]),
        ("LINE_START", [250.0, 0.0, 50.0], [0.0,  3.141, 0.0]),
    ]

    # ── Phase 3: Cartesian linear path — Pilz LIN (like computeCartesianPath) ──
    # Each segment is a straight-line Cartesian move between waypoints.
    # The TCP traces a linear path in XYZ space with trapezoidal velocity.
    cartesian_waypoints = [
        ("LINE_MID",   [250.0,  0.0, 10.0], [0.0, 3.141, 0.0]),
        ("LINE_END",   [250.0,  0.0, 50.0], [0.0, 3.141, 0.0]),
    ]

    # ── SIGINT handler: race to send "rest" before hardware dies ──
    shutdown_event = threading.Event()

    def _sigint_handler(signum, frame):
        if shutdown_event.is_set():
            return  # already handling
        shutdown_event.set()
        logger.info("SIGINT caught — racing to send rest position…")
        try:
            arm.set_start_state_to_current_state()
            arm.set_goal_state(configuration_name="rest")
            plan_result = arm.plan()
            if plan_result:
                logger.info("Rest plan found — executing immediately")
                robot.execute(plan_result.trajectory, controllers=[])
                # Give the trajectory a moment to start executing on hardware
                time.sleep(2.0)
                logger.info("Rest trajectory sent.")
            else:
                logger.error("Failed to plan rest position.")
        except Exception as e:
            logger.error(f"Rest motion failed: {e}")

    signal.signal(signal.SIGINT, _sigint_handler)

    # ── Execute (loop forever, SIGINT to stop) ──
    loop = 0
    while not shutdown_event.is_set():
        loop += 1
        logger.info(f"═══ Loop {loop} ═══")

        # ── A) Joint-space moves via OMPL (two-stage: pick_ik → OMPL) ──
        for name, pos_mm, rpy_rad in pose_goals:
            if shutdown_event.is_set():
                break
            logger.info(f"[{name}] Target xyz={pos_mm} mm  (OMPL)")
            pose_stamped = make_pose_stamped(pos_mm, rpy_rad)
            ok = solve_ik_and_plan(robot, arm, logger, pose_stamped.pose)
            if ok:
                time.sleep(1.0)  # let hardware reach position

        # ── B) Cartesian linear path via Pilz LIN ────────────────────
        #    Like C++ move_group->computeCartesianPath(waypoints, ...)
        #    Each LIN move produces a straight-line TCP path in XYZ.
        if not shutdown_event.is_set():
            logger.info("─── Cartesian linear sweep (Pilz LIN) ───")
        for name, pos_mm, rpy_rad in cartesian_waypoints:
            if shutdown_event.is_set():
                break
            logger.info(f"[{name}] LIN → xyz={pos_mm} mm")
            ps = make_pose_stamped(pos_mm, rpy_rad)
            ok = plan_linear_cartesian(robot, arm, logger, ps, pilz_lin_params)
            if ok:
                time.sleep(1.0)

        if not shutdown_event.is_set():
            logger.info(f"Loop {loop} complete!")

    logger.info("Shutting down.")
    try:
        rclpy.shutdown()
    except Exception:
        pass
    os._exit(0)


if __name__ == "__main__":
    main()
