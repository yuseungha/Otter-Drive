from kmu_track.drive_mode_core import (
    ConsecutiveThreshold,
    DriveEvent,
    DriveMode,
    DriveModeMachine,
    RollingConfirmation,
    SubscriptionInterlock,
    desired_subscriptions,
    largest_bbox_area_ratio,
    selected_command_source,
    stamp_is_fresh,
)


def test_three_camera_frames_enter_cone_init():
    ratio = largest_bbox_area_ratio([(0, 0, 80, 80)], 640, 480)
    assert ratio > 0.015
    detector = ConsecutiveThreshold(3)
    fsm = DriveModeMachine()
    assert not detector.update(True)
    assert not detector.update(True)
    assert detector.update(True)
    assert fsm.apply(DriveEvent.CONE_CONFIRMED) == DriveMode.CONE_INIT


def test_camera_must_report_off_before_lidar_can_start():
    lock = SubscriptionInterlock()
    lock.camera_active = True
    lock.set_mode(DriveMode.CONE_INIT)
    assert not lock.lidar_allowed()
    lock.camera_active = False
    assert lock.lidar_allowed()


def test_lidar_must_report_off_before_camera_can_start():
    lock = SubscriptionInterlock()
    lock.set_mode(DriveMode.CONE_SLALOM)
    lock.lidar_active = True
    lock.set_mode(DriveMode.LANE_REACQUIRE)
    assert not lock.camera_allowed()
    lock.lidar_active = False
    assert lock.camera_allowed()


def test_modes_never_request_both_perception_inputs():
    for mode in DriveMode:
        decision = desired_subscriptions(mode)
        assert not (decision.camera and decision.lidar)
    assert desired_subscriptions(DriveMode.CONE_SLALOM).lidar
    assert not desired_subscriptions(DriveMode.LANE_FOLLOW).lidar


def test_lane_reacquire_requires_four_of_five():
    stable = RollingConfirmation(5, 4)
    for value in [True, True, False, True]:
        assert not stable.update(value)
    assert stable.update(True)
    assert not RollingConfirmation(5, 4).update(True)


def test_timeout_events_fail_closed_from_any_driving_mode():
    for mode in list(DriveMode)[:-1]:
        fsm = DriveModeMachine()
        fsm.mode = mode
        assert fsm.apply(DriveEvent.SENSOR_TIMEOUT) == DriveMode.SAFE_STOP
    fsm = DriveModeMachine()
    assert fsm.apply(DriveEvent.PATH_TIMEOUT) == DriveMode.SAFE_STOP
    assert not stamp_is_fresh(1_000_000_000, 2_000_000_000, 0.2)


def test_complete_nominal_transition_table():
    fsm = DriveModeMachine()
    assert fsm.apply(DriveEvent.CONE_CONFIRMED) == DriveMode.CONE_INIT
    assert fsm.apply(DriveEvent.VALID_CONE_PATH) == DriveMode.CONE_SLALOM
    assert fsm.apply(DriveEvent.CONE_FINISHED) == DriveMode.LANE_REACQUIRE
    assert fsm.apply(DriveEvent.LANE_STABLE) == DriveMode.LANE_FOLLOW


def test_transition_timeouts_fail_closed():
    fsm = DriveModeMachine()
    fsm.mode = DriveMode.CONE_INIT
    assert fsm.apply(DriveEvent.TIMEOUT) == DriveMode.SAFE_STOP
    fsm.mode = DriveMode.LANE_REACQUIRE
    assert fsm.apply(DriveEvent.TIMEOUT) == DriveMode.SAFE_STOP


def test_command_mux_blocks_inactive_and_transition_modes():
    assert selected_command_source(DriveMode.LANE_FOLLOW, False) == "lane"
    assert selected_command_source(DriveMode.CONE_SLALOM, False) == "cone"
    assert selected_command_source(DriveMode.CONE_INIT, False) == "neutral"
    assert selected_command_source(
        DriveMode.LANE_REACQUIRE, False) == "neutral"
    assert selected_command_source(DriveMode.LANE_FOLLOW, True) == "neutral"
