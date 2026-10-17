#!/usr/bin/env python3
"""Tests for ``scripts/send_joint_waypoints.py``.

The tool sends exactly one FollowJointTrajectory goal, so its exit code has to
report the real outcome: 0 only for ``GoalStatus.STATUS_SUCCEEDED`` together
with a ``SUCCESSFUL`` result error code, non-zero for everything else.

Four groups:

* pure decision logic - the status/result contract and the waypoint file
  validation, no ROS graph involved;
* the client code path against a mock action client - no action server, no
  goal handle, a rejected goal and a communication exception;
* ``main()`` control flow against a fake sender - exit codes, the message an
  operator sees and the rclpy context/node lifecycle, including the paths where
  the node cannot even be constructed;
* ``main()`` end to end against an in-process simulated
  FollowJointTrajectory action server - the real client code over real ROS 2
  action plumbing.

No real robot and no Gazebo take part: every abnormal outcome is produced by
the simulated action server.
"""

import importlib.util
import threading
import time
import uuid
from concurrent.futures import Future
from contextlib import contextmanager
from pathlib import Path

import pytest

SCRIPT_PATH = (
    Path(__file__).resolve().parents[1] / "scripts" / "send_joint_waypoints.py"
)


def _load_script_under_test():
    spec = importlib.util.spec_from_file_location(
        "send_joint_waypoints_under_test", SCRIPT_PATH
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


sjw = _load_script_under_test()

rclpy = sjw.rclpy
from action_msgs.msg import GoalInfo, GoalStatus  # noqa: E402
from action_msgs.srv import CancelGoal  # noqa: E402
from control_msgs.action import FollowJointTrajectory  # noqa: E402
from rclpy.action import (  # noqa: E402
    ActionServer,
    CancelResponse,
    GoalResponse,
)
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402

SUCCESSFUL = FollowJointTrajectory.Result.SUCCESSFUL
GOAL_TOLERANCE_VIOLATED = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED

GOOD_YAML = """\
joint_names: [joint_1, joint_2]
waypoints:
  - positions: [0.0, 0.0]
    duration: 1.0
  - positions: [0.1, -0.1]
    duration: 2.5
"""


def _write_yaml(tmp_path, text, name="waypoints.yaml"):
    path = tmp_path / name
    path.write_text(text, encoding="utf-8")
    return str(path)


class FakeResult:
    def __init__(self, error_code, error_string=""):
        self.error_code = error_code
        self.error_string = error_string


class FakeResponse:
    def __init__(self, status, result):
        self.status = status
        self.result = result


class _CompletedFuture(Future):
    """A future that is already resolved, so no executor has to run."""

    def __init__(self, value):
        super().__init__()
        self.set_result(value)


# ---------------------------------------------------------------------------
# group 1: pure decision logic
# ---------------------------------------------------------------------------
def test_describe_helpers_name_known_and_unknown_values():
    assert sjw.describe_status(GoalStatus.STATUS_SUCCEEDED) == "SUCCEEDED"
    assert sjw.describe_status(GoalStatus.STATUS_ABORTED) == "ABORTED"
    assert sjw.describe_status(7) == "UNRECOGNIZED_STATUS(7)"
    assert sjw.describe_error_code(SUCCESSFUL) == "SUCCESSFUL"
    assert sjw.describe_error_code(GOAL_TOLERANCE_VIOLATED) == "GOAL_TOLERANCE_VIOLATED"
    assert sjw.describe_error_code(-99) == "UNRECOGNIZED_ERROR_CODE(-99)"


def test_successful_outcome_is_accepted():
    response = FakeResponse(
        GoalStatus.STATUS_SUCCEEDED, FakeResult(SUCCESSFUL, "moved as commanded")
    )
    status, result = sjw.check_outcome(response)
    assert status == GoalStatus.STATUS_SUCCEEDED
    assert result.error_code == SUCCESSFUL


@pytest.mark.parametrize(
    "status,error_code,expected_status_text",
    [
        (GoalStatus.STATUS_ABORTED, GOAL_TOLERANCE_VIOLATED, "action_status=ABORTED"),
        (GoalStatus.STATUS_CANCELED, GOAL_TOLERANCE_VIOLATED, "action_status=CANCELED"),
        (GoalStatus.STATUS_UNKNOWN, SUCCESSFUL, "action_status=UNKNOWN"),
        (GoalStatus.STATUS_ACCEPTED, SUCCESSFUL, "action_status=ACCEPTED"),
        (GoalStatus.STATUS_EXECUTING, SUCCESSFUL, "action_status=EXECUTING"),
        (GoalStatus.STATUS_CANCELING, SUCCESSFUL, "action_status=CANCELING"),
    ],
)
def test_any_non_succeeded_status_is_a_failure(
    status, error_code, expected_status_text
):
    response = FakeResponse(status, FakeResult(error_code, "server side reason"))
    with pytest.raises(sjw.TrajectoryOutcomeError) as excinfo:
        sjw.check_outcome(response)
    message = str(excinfo.value)
    assert expected_status_text in message
    assert "error_string='server side reason'" in message


@pytest.mark.parametrize(
    "error_code,expected_error_text",
    [
        (FollowJointTrajectory.Result.INVALID_GOAL, "error_code=INVALID_GOAL"),
        (FollowJointTrajectory.Result.INVALID_JOINTS, "error_code=INVALID_JOINTS"),
        (
            FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED,
            "error_code=PATH_TOLERANCE_VIOLATED",
        ),
        (GOAL_TOLERANCE_VIOLATED, "error_code=GOAL_TOLERANCE_VIOLATED"),
    ],
)
def test_succeeded_status_with_bad_error_code_is_a_failure(
    error_code, expected_error_text
):
    response = FakeResponse(
        GoalStatus.STATUS_SUCCEEDED, FakeResult(error_code, "tolerance exceeded")
    )
    with pytest.raises(sjw.TrajectoryOutcomeError) as excinfo:
        sjw.check_outcome(response)
    message = str(excinfo.value)
    assert "action_status=SUCCEEDED" in message
    assert expected_error_text in message
    assert "error_string='tolerance exceeded'" in message


def test_missing_result_object_is_a_failure():
    with pytest.raises(sjw.TrajectoryOutcomeError) as excinfo:
        sjw.check_outcome(FakeResponse(GoalStatus.STATUS_SUCCEEDED, None))
    message = str(excinfo.value)
    assert "result object" in message
    assert "action_status=SUCCEEDED" in message


def test_missing_response_is_a_failure():
    with pytest.raises(sjw.TrajectoryOutcomeError) as excinfo:
        sjw.check_outcome(None)
    assert "no result response" in str(excinfo.value)


def test_empty_error_string_is_reported_as_empty():
    response = FakeResponse(
        GoalStatus.STATUS_ABORTED, FakeResult(GOAL_TOLERANCE_VIOLATED, "")
    )
    with pytest.raises(sjw.TrajectoryOutcomeError) as excinfo:
        sjw.check_outcome(response)
    assert "error_string=<empty>" in str(excinfo.value)


def test_failure_message_never_reduces_to_a_bare_number():
    assert sjw.failure_message(RuntimeError("action server is gone")) == (
        "RuntimeError: action server is gone"
    )
    message = sjw.failure_message(
        sjw.TrajectoryOutcomeError(
            "the trajectory did not succeed",
            status=GoalStatus.STATUS_ABORTED,
            error_code=GOAL_TOLERANCE_VIOLATED,
            error_string="cancel timeout",
        )
    )
    assert message.startswith("trajectory failed: the trajectory did not succeed")
    assert "action_status=ABORTED (6)" in message
    assert "error_code=GOAL_TOLERANCE_VIOLATED (-5)" in message
    assert "error_string='cancel timeout'" in message


def test_load_trajectory_accepts_a_valid_file(tmp_path):
    trajectory = sjw.load_trajectory(_write_yaml(tmp_path, GOOD_YAML))
    assert list(trajectory.joint_names) == ["joint_1", "joint_2"]
    assert len(trajectory.points) == 2
    assert list(trajectory.points[0].positions) == [0.0, 0.0]
    assert trajectory.points[1].time_from_start.sec == 2
    assert trajectory.points[1].time_from_start.nanosec == 500000000


def test_load_trajectory_keeps_optional_velocity_fields(tmp_path):
    text = GOOD_YAML.replace(
        "    duration: 1.0", "    duration: 1.0\n    velocities: [0.0, 0.2]"
    )
    trajectory = sjw.load_trajectory(_write_yaml(tmp_path, text))
    assert list(trajectory.points[0].velocities) == [0.0, 0.2]


@pytest.mark.parametrize(
    "text,expected_fragment",
    [
        ("just a scalar\n", "must contain a YAML mapping"),
        ("joint_names: [joint_1]\n", "must define 'waypoints'"),
        ("waypoints:\n  - positions: [0.0]\n    duration: 1.0\n", "must define 'joint_names'"),
        ("joint_names: []\nwaypoints: []\n", "joint_names must be a non-empty list"),
        (
            "joint_names: [joint_1, joint_1]\nwaypoints:\n"
            "  - positions: [0.0, 0.0]\n    duration: 1.0\n",
            "must not contain duplicates",
        ),
        (
            "joint_names: [joint_1, joint_2]\nwaypoints: []\n",
            "waypoints must be a non-empty list",
        ),
        (
            "joint_names: [joint_1, joint_2]\nwaypoints:\n  - duration: 1.0\n",
            "must define 'positions'",
        ),
        (
            "joint_names: [joint_1, joint_2]\nwaypoints:\n  - positions: [0.0, 0.0]\n",
            "must define 'duration'",
        ),
        (
            "joint_names: [joint_1, joint_2]\nwaypoints:\n"
            "  - positions: [0.0]\n    duration: 1.0\n",
            "has 1 positions, expected 2",
        ),
        (
            "joint_names: [joint_1, joint_2]\nwaypoints:\n"
            "  - positions: [.nan, 0.0]\n    duration: 1.0\n",
            "contain NaN or Inf",
        ),
        (
            "joint_names: [joint_1, joint_2]\nwaypoints:\n"
            "  - positions: [0.0, 0.0]\n    duration: -1.0\n",
            "duration must be greater than the previous duration",
        ),
        (
            "joint_names: [joint_1, joint_2]\nwaypoints:\n"
            "  - positions: [0.0, 0.0]\n    duration: 2.0\n"
            "  - positions: [0.1, 0.1]\n    duration: 1.0\n",
            "duration must be greater than the previous duration",
        ),
    ],
)
def test_load_trajectory_rejects_invalid_waypoint_files(
    tmp_path, text, expected_fragment
):
    with pytest.raises(ValueError) as excinfo:
        sjw.load_trajectory(_write_yaml(tmp_path, text))
    assert expected_fragment in str(excinfo.value)


def test_load_trajectory_rejects_malformed_yaml(tmp_path):
    path = _write_yaml(tmp_path, "joint_names: [joint_1\n  waypoints: {")
    with pytest.raises(Exception) as excinfo:
        sjw.load_trajectory(path)
    assert "yaml" in type(excinfo.value).__module__.lower()


def test_load_trajectory_rejects_a_missing_file(tmp_path):
    with pytest.raises(OSError):
        sjw.load_trajectory(str(tmp_path / "does_not_exist.yaml"))


# ---------------------------------------------------------------------------
# group 2: the client against a mock action client
# ---------------------------------------------------------------------------
class FakeGoalHandle:
    def __init__(self, accepted, result_future=None):
        self.accepted = accepted
        self._result_future = result_future

    def get_result_async(self):
        return self._result_future


class FakeActionClient:
    def __init__(self, wait_ok=True, goal_handle=None, send_goal_error=None):
        self._wait_ok = wait_ok
        self._goal_handle = goal_handle
        self._send_goal_error = send_goal_error
        self.send_goal_calls = 0

    def wait_for_server(self, timeout_sec=None):
        return self._wait_ok

    def send_goal_async(self, goal):
        self.send_goal_calls += 1
        if self._send_goal_error is not None:
            future = Future()
            future.set_exception(self._send_goal_error)
            return future
        return _CompletedFuture(self._goal_handle)


@pytest.fixture(scope="session")
def ros_context():
    """The script is a process entry point; here the test owns the context."""
    rclpy.init()
    try:
        yield
    finally:
        if rclpy.ok():
            rclpy.shutdown()


@pytest.fixture
def no_spin(monkeypatch):
    """Replace rclpy's executor with the mock futures used in group 2."""
    monkeypatch.setattr(
        sjw.rclpy, "spin_until_future_complete", lambda *args, **kwargs: None
    )


def _sender(client):
    return sjw.JointWaypointSender("/test/action", 0.1, action_client=client)


@pytest.fixture
def trajectory(tmp_path):
    return sjw.load_trajectory(_write_yaml(tmp_path, GOOD_YAML))


def test_send_without_server_never_sends_a_goal(ros_context, no_spin, trajectory):
    client = FakeActionClient(wait_ok=False)
    sender = _sender(client)
    try:
        with pytest.raises(RuntimeError) as excinfo:
            sender.send(trajectory)
        assert "is not available" in str(excinfo.value)
        assert client.send_goal_calls == 0
    finally:
        sender.destroy_node()


def test_send_without_a_goal_handle_is_reported(ros_context, no_spin, trajectory):
    client = FakeActionClient(goal_handle=None)
    sender = _sender(client)
    try:
        with pytest.raises(RuntimeError) as excinfo:
            sender.send(trajectory)
        assert "no goal handle" in str(excinfo.value)
        assert client.send_goal_calls == 1
    finally:
        sender.destroy_node()


def test_send_with_a_rejected_goal_is_reported(ros_context, no_spin, trajectory):
    client = FakeActionClient(goal_handle=FakeGoalHandle(accepted=False))
    sender = _sender(client)
    try:
        with pytest.raises(RuntimeError) as excinfo:
            sender.send(trajectory)
        assert "rejected" in str(excinfo.value)
        assert "no trajectory was commanded" in str(excinfo.value)
    finally:
        sender.destroy_node()


def test_send_propagates_a_communication_failure(ros_context, no_spin, trajectory):
    client = FakeActionClient(send_goal_error=RuntimeError("goal request lost"))
    sender = _sender(client)
    try:
        with pytest.raises(RuntimeError) as excinfo:
            sender.send(trajectory)
        assert "goal request lost" in str(excinfo.value)
    finally:
        sender.destroy_node()


def test_send_returns_the_raw_response_for_a_success(ros_context, no_spin, trajectory):
    response = FakeResponse(GoalStatus.STATUS_SUCCEEDED, FakeResult(SUCCESSFUL, "ok"))
    client = FakeActionClient(goal_handle=FakeGoalHandle(True, _CompletedFuture(response)))
    sender = _sender(client)
    try:
        status, result = sjw.check_outcome(sender.send(trajectory))
        assert status == GoalStatus.STATUS_SUCCEEDED
        assert result.error_string == "ok"
        assert client.send_goal_calls == 1
    finally:
        sender.destroy_node()


# ---------------------------------------------------------------------------
# group 3: main() control flow with a fake sender
# ---------------------------------------------------------------------------
class FakeLogger:
    def __init__(self, sender):
        self._sender = sender

    def info(self, message):
        self._sender.infos.append(message)

    def error(self, message):
        self._sender.errors.append(message)

    def warn(self, message):  # pragma: no cover - kept for completeness
        self._sender.errors.append(message)


class FakeSender:
    """Stands in for JointWaypointSender so main() can be driven exactly."""

    scripted_response = None
    scripted_exception = None
    construction_error = None
    created = []

    def __init__(self, action_name, wait_sec, action_client=None):
        if FakeSender.construction_error is not None:
            raise FakeSender.construction_error
        self.action_name = action_name
        self.wait_sec = wait_sec
        self.send_calls = 0
        self.destroyed = False
        self.infos = []
        self.errors = []
        FakeSender.created.append(self)

    def get_logger(self):
        return FakeLogger(self)

    def send(self, trajectory):
        self.send_calls += 1
        if FakeSender.scripted_exception is not None:
            raise FakeSender.scripted_exception
        return FakeSender.scripted_response

    def destroy_node(self):
        self.destroyed = True


@pytest.fixture(autouse=True)
def _reset_fake_sender():
    FakeSender.scripted_response = None
    FakeSender.scripted_exception = None
    FakeSender.construction_error = None
    FakeSender.created = []
    yield
    FakeSender.scripted_response = None
    FakeSender.scripted_exception = None
    FakeSender.construction_error = None
    FakeSender.created = []


@pytest.fixture
def fake_sender(monkeypatch):
    monkeypatch.setattr(sjw, "JointWaypointSender", FakeSender)
    return FakeSender


@pytest.fixture
def frozen_rclpy(monkeypatch):
    """Take the real rclpy context out of the loop and record its lifecycle."""
    calls = {"init": 0, "shutdown": 0}
    state = {"ok": False}

    def fake_init():
        calls["init"] += 1
        state["ok"] = True

    def fake_shutdown():
        calls["shutdown"] += 1
        state["ok"] = False

    monkeypatch.setattr(sjw.rclpy, "init", fake_init)
    monkeypatch.setattr(sjw.rclpy, "shutdown", fake_shutdown)
    monkeypatch.setattr(sjw.rclpy, "ok", lambda: state["ok"])
    return calls, state


def test_main_returns_zero_only_for_a_confirmed_success(
    tmp_path, fake_sender, frozen_rclpy
):
    calls, state = frozen_rclpy
    state["ok"] = False
    FakeSender.scripted_response = FakeResponse(
        GoalStatus.STATUS_SUCCEEDED, FakeResult(SUCCESSFUL, "reached")
    )
    exit_code = sjw.main([_write_yaml(tmp_path, GOOD_YAML)])

    assert exit_code == 0
    sender = FakeSender.created[0]
    assert sender.send_calls == 1
    assert sender.destroyed is True
    assert calls == {"init": 1, "shutdown": 1}
    assert "SUCCEEDED" in sender.infos[0]
    assert "SUCCESSFUL" in sender.infos[0]
    assert sender.errors == []


@pytest.mark.parametrize(
    "status,error_code",
    [
        (GoalStatus.STATUS_ABORTED, GOAL_TOLERANCE_VIOLATED),
        (GoalStatus.STATUS_CANCELED, GOAL_TOLERANCE_VIOLATED),
        (GoalStatus.STATUS_UNKNOWN, SUCCESSFUL),
        (GoalStatus.STATUS_CANCELING, SUCCESSFUL),
        (GoalStatus.STATUS_SUCCEEDED, FollowJointTrajectory.Result.INVALID_GOAL),
        (GoalStatus.STATUS_SUCCEEDED, GOAL_TOLERANCE_VIOLATED),
    ],
)
def test_main_returns_non_zero_for_unsuccessful_outcomes(
    tmp_path, fake_sender, frozen_rclpy, status, error_code
):
    FakeSender.scripted_response = FakeResponse(
        status, FakeResult(error_code, "controller side reason")
    )
    exit_code = sjw.main([_write_yaml(tmp_path, GOOD_YAML)])

    assert exit_code != 0
    sender = FakeSender.created[0]
    assert sender.destroyed is True
    log_line = sender.errors[0]
    assert "trajectory failed" in log_line
    assert f"action_status={sjw.describe_status(status)} ({status})" in log_line
    assert (
        f"error_code={sjw.describe_error_code(error_code)} ({error_code})" in log_line
    )
    assert "error_string='controller side reason'" in log_line


def test_main_reports_a_missing_result_object(
    tmp_path, fake_sender, frozen_rclpy
):
    FakeSender.scripted_response = FakeResponse(GoalStatus.STATUS_SUCCEEDED, None)
    assert sjw.main([_write_yaml(tmp_path, GOOD_YAML)]) != 0
    assert "result object" in FakeSender.created[0].errors[0]


def test_main_returns_non_zero_when_the_goal_is_rejected(
    tmp_path, fake_sender, frozen_rclpy
):
    FakeSender.scripted_exception = RuntimeError(
        "the trajectory goal was rejected by '/test/action'; "
        "no trajectory was commanded"
    )
    assert sjw.main([_write_yaml(tmp_path, GOOD_YAML)]) != 0
    sender = FakeSender.created[0]
    assert "rejected" in sender.errors[0]
    assert sender.destroyed is True


def test_main_returns_non_zero_when_the_server_is_unavailable(
    tmp_path, fake_sender, frozen_rclpy
):
    FakeSender.scripted_exception = RuntimeError(
        "FollowJointTrajectory action server '/test/action' is not available. "
        "Start digital_twin_real.launch.py first and keep it running."
    )
    assert sjw.main([_write_yaml(tmp_path, GOOD_YAML)]) != 0
    assert "is not available" in FakeSender.created[0].errors[0]


def test_main_returns_non_zero_on_a_communication_failure(
    tmp_path, fake_sender, frozen_rclpy
):
    FakeSender.scripted_exception = RuntimeError("goal request lost")
    assert sjw.main([_write_yaml(tmp_path, GOOD_YAML)]) != 0
    assert "goal request lost" in FakeSender.created[0].errors[0]


def test_main_returns_non_zero_for_a_malformed_waypoint_file(
    tmp_path, fake_sender, frozen_rclpy
):
    path = _write_yaml(tmp_path, "joint_names: [joint_1\n  waypoints: {")
    assert sjw.main([path]) != 0
    sender = FakeSender.created[0]
    assert sender.send_calls == 0
    assert sender.errors, "the parse failure must be logged"
    assert sender.destroyed is True


def test_main_returns_non_zero_for_invalid_waypoint_data(
    tmp_path, fake_sender, frozen_rclpy
):
    text = GOOD_YAML.replace("duration: 1.0", "duration: .nan")
    assert sjw.main([_write_yaml(tmp_path, text)]) != 0
    sender = FakeSender.created[0]
    assert sender.send_calls == 0
    assert "duration must be greater than the previous duration" in sender.errors[0]


def test_main_returns_non_zero_when_the_waypoint_file_is_missing(
    tmp_path, fake_sender, frozen_rclpy
):
    assert sjw.main([str(tmp_path / "nope.yaml")]) != 0
    assert "No such file" in FakeSender.created[0].errors[0]


def test_main_releases_ros_resources_when_node_construction_fails(
    tmp_path, fake_sender, frozen_rclpy, capsys
):
    calls, _ = frozen_rclpy
    FakeSender.construction_error = RuntimeError("no ros middleware available")

    assert sjw.main([_write_yaml(tmp_path, GOOD_YAML)]) != 0

    assert FakeSender.created == []
    assert calls == {"init": 1, "shutdown": 1}
    captured = capsys.readouterr()
    assert "no ros middleware available" in captured.err


def test_main_leaves_an_existing_context_alone(tmp_path, fake_sender, frozen_rclpy):
    calls, state = frozen_rclpy
    state["ok"] = True
    FakeSender.scripted_response = FakeResponse(
        GoalStatus.STATUS_SUCCEEDED, FakeResult(SUCCESSFUL)
    )

    assert sjw.main([_write_yaml(tmp_path, GOOD_YAML)]) == 0

    assert calls == {"init": 0, "shutdown": 0}
    assert FakeSender.created[0].destroyed is True


def test_main_does_not_retry_and_does_not_send_a_second_goal(
    tmp_path, fake_sender, frozen_rclpy
):
    FakeSender.scripted_exception = RuntimeError("first goal failed")
    assert sjw.main([_write_yaml(tmp_path, GOOD_YAML)]) != 0
    assert FakeSender.created[0].send_calls == 1
    assert len(FakeSender.created) == 1


def test_main_keeps_the_cli_contract(tmp_path, fake_sender, frozen_rclpy):
    FakeSender.scripted_response = FakeResponse(
        GoalStatus.STATUS_SUCCEEDED, FakeResult(SUCCESSFUL)
    )
    sjw.main(
        [
            _write_yaml(tmp_path, GOOD_YAML),
            "--action",
            "/arm_1_controller/follow_joint_trajectory",
            "--wait-sec",
            "12.5",
        ]
    )
    sender = FakeSender.created[0]
    assert sender.action_name == "/arm_1_controller/follow_joint_trajectory"
    assert sender.wait_sec == 12.5


def test_default_action_name_is_unchanged(tmp_path, fake_sender, frozen_rclpy):
    FakeSender.scripted_response = FakeResponse(
        GoalStatus.STATUS_SUCCEEDED, FakeResult(SUCCESSFUL)
    )
    sjw.main([_write_yaml(tmp_path, GOOD_YAML)])
    assert FakeSender.created[0].action_name == "/arm_1_controller/follow_joint_trajectory"
    assert FakeSender.created[0].wait_sec == 60.0


# ---------------------------------------------------------------------------
# group 4: main() end to end against a simulated action server
# ---------------------------------------------------------------------------
class SimulatedOutcome:
    """What the simulated server does with the next goal."""

    def __init__(self, *, reject=False, terminal="succeed", error_code=0,
                 error_string=""):
        self.reject = reject
        self.terminal = terminal
        self.error_code = error_code
        self.error_string = error_string


class SimulatedTrajectoryServer(Node):
    """A FollowJointTrajectory action server with one scripted outcome.

    For the ``canceled`` outcome the server behaves like a real one: it keeps
    executing until a CancelGoal request arrives, moves to CANCELING and only
    then reports CANCELED.  ``rcl_action`` rejects a direct
    ``EXECUTING -> CANCELED`` transition, which is exactly why a cancel request
    is part of the scenario.
    """

    cancel_poll_sec = 20.0

    def __init__(self, action_name, outcome):
        super().__init__(f"simulated_trajectory_server_{uuid.uuid4().hex[:8]}")
        self._outcome = outcome
        self.goals_received = 0
        self.last_goal = None
        self._server = ActionServer(
            self,
            FollowJointTrajectory,
            action_name,
            execute_callback=self._execute,
            goal_callback=self._goal,
            cancel_callback=self._cancel,
        )
        # Plays the part of an operator or upper computer that cancels the
        # trajectory, using the action's public CancelGoal service.
        self._cancel_client = self.create_client(
            CancelGoal, action_name + "/_action/cancel_goal"
        )
        self.cancel_request = None

    def _goal(self, goal_request):
        self.goals_received += 1
        self.last_goal = goal_request
        if self._outcome.reject:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel(self, goal_handle):
        return CancelResponse.ACCEPT

    def request_cancel(self, timeout_sec=15.0):
        """Send a real CancelGoal request; an empty goal_info means 'all goals'."""
        deadline = time.monotonic() + timeout_sec
        while not self._cancel_client.wait_for_service(timeout_sec=1.0):
            if time.monotonic() >= deadline:
                return False
        request = CancelGoal.Request()
        request.goal_info = GoalInfo()
        self.cancel_request = self._cancel_client.call_async(request)
        return True

    def cancel_return_code(self, timeout_sec=15.0):
        deadline = time.monotonic() + timeout_sec
        while not self.cancel_request.done():
            if time.monotonic() >= deadline:
                return None
            time.sleep(0.02)
        return self.cancel_request.result().return_code

    def _result(self):
        result = FollowJointTrajectory.Result()
        result.error_code = self._outcome.error_code
        result.error_string = self._outcome.error_string
        return result

    def _execute(self, goal_handle):
        if self._outcome.terminal == "canceled":
            return self._execute_until_canceled(goal_handle)
        if self._outcome.terminal == "abort":
            goal_handle.abort()
        else:
            goal_handle.succeed()
        return self._result()

    def _execute_until_canceled(self, goal_handle):
        deadline = time.monotonic() + self.cancel_poll_sec
        while time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                time.sleep(0.05)
                goal_handle.canceled()
                return self._result()
            time.sleep(0.02)
        goal_handle.abort()
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
        result.error_string = "no cancel request arrived before the deadline"
        return result


@contextmanager
def running_server(action_name, outcome):
    server = SimulatedTrajectoryServer(action_name, outcome)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(server)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        yield server
    finally:
        executor.shutdown()
        thread.join(timeout=15.0)
        server.destroy_node()


@pytest.mark.parametrize(
    "outcome,expected_exit,expected_fragments",
    [
        (
            SimulatedOutcome(),
            0,
            ["action_status=SUCCEEDED (4)", "error_code=SUCCESSFUL (0)"],
        ),
        (
            SimulatedOutcome(
                error_code=GOAL_TOLERANCE_VIOLATED,
                error_string="goal tolerance violated",
            ),
            1,
            [
                "action_status=SUCCEEDED (4)",
                "error_code=GOAL_TOLERANCE_VIOLATED (-5)",
                "error_string='goal tolerance violated'",
            ],
        ),
        (
            SimulatedOutcome(
                terminal="abort",
                error_code=GOAL_TOLERANCE_VIOLATED,
                error_string="cancel timed out; the result is unknown",
            ),
            1,
            [
                "action_status=ABORTED (6)",
                "error_code=GOAL_TOLERANCE_VIOLATED (-5)",
                "error_string='cancel timed out; the result is unknown'",
            ],
        ),
        (
            SimulatedOutcome(
                terminal="canceled",
                error_code=GOAL_TOLERANCE_VIOLATED,
                error_string="canceled; the real trajectory result is unknown",
            ),
            1,
            [
                "action_status=CANCELED (5)",
                "error_code=GOAL_TOLERANCE_VIOLATED (-5)",
                "error_string='canceled; the real trajectory result is unknown'",
            ],
        ),
        (
            SimulatedOutcome(reject=True),
            1,
            ["rejected"],
        ),
    ],
)
def test_main_against_a_simulated_action_server(
    ros_context, tmp_path, capfd, outcome, expected_exit, expected_fragments
):
    action_name = f"/test_sjw_{uuid.uuid4().hex[:8]}/follow_joint_trajectory"
    waypoint_file = _write_yaml(tmp_path, GOOD_YAML)
    canceller = None

    with running_server(action_name, outcome) as server:
        if outcome.terminal == "canceled":
            canceller = threading.Thread(
                target=_cancel_after_delay, args=(server, 0.5), daemon=True
            )
            canceller.start()
        exit_code = sjw.main(
            [waypoint_file, "--action", action_name, "--wait-sec", "30"]
        )

    if canceller is not None:
        canceller.join(timeout=15.0)
        assert server.cancel_return_code() == CancelGoal.Response.ERROR_NONE
        assert len(server.cancel_request.result().goals_canceling) == 1

    assert server.goals_received == 1
    assert exit_code == expected_exit

    _, stderr = capfd.readouterr()
    for fragment in expected_fragments:
        assert fragment in stderr, f"{fragment!r} missing from the log:\n{stderr}"


def _cancel_after_delay(server, delay_sec):
    time.sleep(delay_sec)
    server.request_cancel()


def test_simulated_server_receives_the_parsed_trajectory(ros_context, tmp_path):
    action_name = f"/test_sjw_{uuid.uuid4().hex[:8]}/follow_joint_trajectory"
    waypoint_file = _write_yaml(tmp_path, GOOD_YAML)

    with running_server(action_name, SimulatedOutcome()) as server:
        assert sjw.main([waypoint_file, "--action", action_name, "--wait-sec", "30"]) == 0

    assert list(server.last_goal.trajectory.joint_names) == ["joint_1", "joint_2"]
    assert len(server.last_goal.trajectory.points) == 2


def test_main_against_no_server_at_all(ros_context, tmp_path, capfd):
    action_name = f"/test_sjw_missing_{uuid.uuid4().hex[:8]}/follow_joint_trajectory"
    waypoint_file = _write_yaml(tmp_path, GOOD_YAML)

    exit_code = sjw.main([waypoint_file, "--action", action_name, "--wait-sec", "1"])

    assert exit_code != 0
    _, stderr = capfd.readouterr()
    assert "is not available" in stderr
