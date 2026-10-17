#!/usr/bin/env python3
"""Send exactly one FollowJointTrajectory goal and report its real outcome.

This is an operator tool, not a safety device: it reports what the action
server said and nothing more.  A cancellation is never interpreted as evidence
that the robot stopped.

The exit code is the answer an operator or a wrapping shell script needs:

    0  the action reached ``GoalStatus.STATUS_SUCCEEDED`` *and* the
       FollowJointTrajectory result reported ``error_code == SUCCESSFUL``
    1  every other outcome: no action server, a rejected goal, ABORTED,
       CANCELED, UNKNOWN or an unknown status, a non-successful result error
       code, a missing result object, an invalid waypoint file or any
       communication failure

The tool deliberately does not retry and does not send a follow-up
trajectory.  A failure leaves the decision to the operator, because a real
trajectory that ended without a confirmed completion may have left the arm
anywhere, and only a human can clear that state.
"""

import argparse
import math
import sys

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint

SUCCESS_EXIT_CODE = 0
FAILURE_EXIT_CODE = 1

_STATUS_NAMES = {
    GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
    GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
    GoalStatus.STATUS_EXECUTING: "EXECUTING",
    GoalStatus.STATUS_CANCELING: "CANCELING",
    GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
    GoalStatus.STATUS_CANCELED: "CANCELED",
    GoalStatus.STATUS_ABORTED: "ABORTED",
}

_ERROR_CODE_NAMES = {
    FollowJointTrajectory.Result.SUCCESSFUL: "SUCCESSFUL",
    FollowJointTrajectory.Result.INVALID_GOAL: "INVALID_GOAL",
    FollowJointTrajectory.Result.INVALID_JOINTS: "INVALID_JOINTS",
    FollowJointTrajectory.Result.OLD_HEADER_TIMESTAMP: "OLD_HEADER_TIMESTAMP",
    FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED: "PATH_TOLERANCE_VIOLATED",
    FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED: "GOAL_TOLERANCE_VIOLATED",
}


def describe_status(status):
    """Return a readable name for an ``action_msgs/GoalStatus`` value."""
    if status is None:
        return "UNKNOWN"
    return _STATUS_NAMES.get(status, f"UNRECOGNIZED_STATUS({status})")


def describe_error_code(error_code):
    """Return a readable name for a FollowJointTrajectory result error code."""
    if error_code is None:
        return "UNKNOWN"
    return _ERROR_CODE_NAMES.get(error_code, f"UNRECOGNIZED_ERROR_CODE({error_code})")


class TrajectoryOutcomeError(RuntimeError):
    """The trajectory did not reach a confirmed success.

    Carries every field the operator needs to understand the failure: the
    action terminal status, the result error code, the result error string and
    a plain reason.  The message states what is known; it never claims that the
    robot stopped.
    """

    def __init__(self, reason, status=None, error_code=None, error_string=""):
        self.reason = reason
        self.status = status
        self.error_code = error_code
        self.error_string = error_string
        super().__init__(self.describe())

    def describe(self):
        parts = [self.reason]
        if self.status is not None:
            parts.append(f"action_status={describe_status(self.status)} ({self.status})")
        if self.error_code is not None:
            parts.append(
                f"error_code={describe_error_code(self.error_code)} ({self.error_code})")
        if self.error_string:
            parts.append(f"error_string={self.error_string!r}")
        elif self.error_code is not None:
            parts.append("error_string=<empty>")
        return "; ".join(parts)


def check_outcome(response):
    """Return ``(status, result)`` for a real success, otherwise raise.

    A goal counts as successful only when both conditions hold:

    1. the action reached ``GoalStatus.STATUS_SUCCEEDED``;
    2. the FollowJointTrajectory result reported ``SUCCESSFUL``.

    Everything else - including a response without a result object, which is
    exactly the case where nothing is known about the robot - is a failure.
    """
    if response is None:
        raise TrajectoryOutcomeError("the action server sent no result response")

    status = getattr(response, "status", None)
    result = getattr(response, "result", None)

    if result is None:
        raise TrajectoryOutcomeError(
            "the action finished without a FollowJointTrajectory result object, "
            "so the trajectory outcome is unknown",
            status=status)

    if status != GoalStatus.STATUS_SUCCEEDED:
        raise TrajectoryOutcomeError(
            "the trajectory did not succeed",
            status=status,
            error_code=result.error_code,
            error_string=result.error_string)

    if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
        raise TrajectoryOutcomeError(
            "the action reported success but the result error code is not SUCCESSFUL",
            status=status,
            error_code=result.error_code,
            error_string=result.error_string)

    return status, result


class JointWaypointSender(Node):
    def __init__(self, action_name, wait_sec, action_client=None):
        super().__init__("joint_waypoint_sender")
        self._action_name = action_name
        self._wait_sec = wait_sec
        # An injected client keeps the decision logic testable without a running
        # action server.  Production callers never pass one.
        if action_client is not None:
            self._client = action_client
        else:
            self._client = ActionClient(self, FollowJointTrajectory, action_name)

    def send(self, trajectory):
        """Send one goal and return the raw result response.

        No retry and no follow-up trajectory: exactly one goal is sent, and the
        caller decides what the response means.
        """
        if not self._client.wait_for_server(timeout_sec=self._wait_sec):
            raise RuntimeError(
                f"FollowJointTrajectory action server '{self._action_name}' is not "
                "available. Start digital_twin_real.launch.py first and keep it running."
            )

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        goal_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, goal_future)
        goal_handle = goal_future.result()
        if goal_handle is None:
            raise RuntimeError(
                "the action server returned no goal handle, so the goal was not accepted"
            )
        if not goal_handle.accepted:
            raise RuntimeError(
                f"the trajectory goal was rejected by '{self._action_name}'; "
                "no trajectory was commanded"
            )

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result()


def load_trajectory(path):
    with open(path, "r", encoding="utf-8") as file:
        data = yaml.safe_load(file)

    if not isinstance(data, dict):
        raise ValueError("the waypoint file must contain a YAML mapping")
    for key in ("joint_names", "waypoints"):
        if key not in data:
            raise ValueError(f"the waypoint file must define '{key}'")

    names = data["joint_names"]
    points = data["waypoints"]
    if not isinstance(names, list) or not names:
        raise ValueError("joint_names must be a non-empty list")
    if any(not isinstance(name, str) or not name for name in names):
        raise ValueError("joint_names must contain non-empty strings")
    if len(set(names)) != len(names):
        raise ValueError("joint_names must not contain duplicates")
    if not isinstance(points, list) or not points:
        raise ValueError("waypoints must be a non-empty list")
    trajectory = FollowJointTrajectory.Goal().trajectory
    trajectory.joint_names = names

    previous_duration = 0.0
    for index, waypoint in enumerate(points):
        if not isinstance(waypoint, dict):
            raise ValueError(f"Waypoint {index} must be a mapping")
        for key in ("positions", "duration"):
            if key not in waypoint:
                raise ValueError(f"Waypoint {index} must define '{key}'")
        positions = waypoint["positions"]
        if len(positions) != len(names):
            raise ValueError(f"Waypoint {index} has {len(positions)} positions, expected {len(names)}")
        position_values = [float(value) for value in positions]
        if any(not math.isfinite(value) for value in position_values):
            raise ValueError(f"Waypoint {index} positions contain NaN or Inf")

        duration = float(waypoint["duration"])
        if not math.isfinite(duration) or duration <= previous_duration:
            raise ValueError(f"Waypoint {index} duration must be greater than the previous duration")
        previous_duration = duration

        point = JointTrajectoryPoint()
        point.positions = position_values
        for field in ("velocities", "accelerations", "effort"):
            if field in waypoint:
                values = [float(value) for value in waypoint[field]]
                if len(values) != len(names):
                    raise ValueError(
                        f"Waypoint {index} {field} has {len(values)} values, expected {len(names)}")
                if any(not math.isfinite(value) for value in values):
                    raise ValueError(f"Waypoint {index} {field} contains NaN or Inf")
                setattr(point, field, values)
        seconds = int(math.floor(duration))
        nanoseconds = int(round((duration - seconds) * 1_000_000_000))
        if nanoseconds >= 1_000_000_000:
            seconds += 1
            nanoseconds -= 1_000_000_000
        point.time_from_start.sec = seconds
        point.time_from_start.nanosec = nanoseconds
        trajectory.points.append(point)

    return trajectory


def failure_message(exc):
    """Build a log line that always names the reason, never only a number."""
    if isinstance(exc, TrajectoryOutcomeError):
        return f"trajectory failed: {exc.describe()}"
    return f"{type(exc).__name__}: {exc}"


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("waypoints_file")
    parser.add_argument(
        "--action",
        default="/arm_1_controller/follow_joint_trajectory",
        help="FollowJointTrajectory action name",
    )
    parser.add_argument(
        "--wait-sec",
        type=float,
        default=60.0,
        help="Seconds to wait for the FollowJointTrajectory action server",
    )
    args = parser.parse_args(argv)

    node = None
    # rclpy.init() may only be called once per process, so only take ownership
    # of the context when nobody else has initialised it already; only the owner
    # shuts it down again.
    owns_context = not rclpy.ok()
    try:
        if owns_context:
            rclpy.init()
        node = JointWaypointSender(args.action, args.wait_sec)
        trajectory = load_trajectory(args.waypoints_file)
        status, result = check_outcome(node.send(trajectory))
        node.get_logger().info(
            f"Trajectory finished with action_status={describe_status(status)} ({status}) "
            f"and error_code={describe_error_code(result.error_code)} ({result.error_code})"
        )
    except Exception as exc:
        message = failure_message(exc)
        if node is not None:
            node.get_logger().error(message)
        else:
            print(f"[joint_waypoint_sender] ERROR: {message}", file=sys.stderr)
        return FAILURE_EXIT_CODE
    finally:
        if node is not None:
            node.destroy_node()
        if owns_context and rclpy.ok():
            rclpy.shutdown()
    return SUCCESS_EXIT_CODE


if __name__ == "__main__":
    sys.exit(main())
