import xml.dom.minidom

from robogame_sim import firmware_model as fw
from robogame_sim.robot_sdf import robot_sdf


def test_bridge_percentages_match_robot_commands():
    assert fw.twist_to_percent(0.5, 0.0) == (100, 0)
    assert fw.twist_to_percent(0.25, -0.75) == (50, -50)
    assert fw.twist_to_percent(2.0, 9.0) == (100, 100)


def test_mix_matches_drive_with_speed():
    assert fw.mix(100, 0) == (100, 100)
    assert fw.mix(0, 100) == (-100, 100)
    assert fw.mix(100, 50) == (33, 100)
    assert fw.mix(-60, 20) == (-80, -40)


def test_duty_has_stall_floor():
    assert fw.duty_percent(0) == 0
    assert fw.duty_percent(1) == 35
    assert fw.duty_percent(-50) == -67
    assert fw.duty_percent(100) == 100


def test_side_speeds_to_twist():
    linear, angular = fw.side_speeds_to_twist(100, 100, 0.5, 0.06)
    assert linear == 0.5 and angular == 0.0

    linear, angular = fw.side_speeds_to_twist(-100, 100, 0.5, 0.06)
    assert linear == 0.0 and abs(angular - 1.0 / 0.06) < 1e-9


def test_arm_steps_and_clamps():
    arm = fw.ArmModel()
    assert arm.apply("+") == ""
    assert arm.shoulder == 95

    arm.apply("+" * 40)
    assert arm.shoulder == 180

    arm.apply("Hc")
    assert arm.elbow == 85
    assert arm.gripper == fw.GRIPPER_CLOSED_ANGLE
    assert arm.joint_positions()["gripper"] == 0.0

    assert arm.apply("Fz") == "Fz"


def test_robot_sdf_is_valid_xml_and_namespaced():
    sdf = robot_sdf(3)
    xml.dom.minidom.parseString(sdf)
    assert '<model name="robot_3">' in sdf
    assert '/robot_3/sim/cmd_vel' in sdf
    assert '/model/robot_3/odometry' in sdf
