"""Exercise the real runtime executable on random, localhost-only mock topics."""
import os
import signal
import subprocess
import time
import uuid
from pathlib import Path

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import yaml


def test_source_feedback_age_and_latest_commands(tmp_path):
    executable = os.environ['DRIVER_TEST_EXECUTABLE']
    plugin_xml = os.environ['DRIVER_TEST_PLUGIN_XML']
    domain = 180 + os.getpid() % 40
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY='1',
               ROS_LOG_DIR=str(tmp_path / 'ros-log'))
    prefix = '/boundary_' + uuid.uuid4().hex[:12]
    names = ['left_a', 'left_b', 'right_a', 'right_b']
    params = {
        'plugin_class': 'humanoid_driver_runtime/RosTopicRobotDriver',
        'plugin_xml_paths': [plugin_xml], 'joint_names': names,
        'vendor_joint_names': list(names), 'vendor_joint_groups': ['left', 'left', 'right', 'right'],
        'platform_joint_state_topic': prefix + '/state',
        'platform_joint_command_topic': prefix + '/command',
        'diagnostics_topic': prefix + '/diagnostics',
        'plugin_parameters': ['state_topic=' + prefix + '/vendor',
            'command_topic.left=' + prefix + '/left', 'command_topic.right=' + prefix + '/right',
            'state_timeout_s=0.25', 'startup_grace_s=3.0']}
    config = tmp_path / 'params.yaml'
    config.write_text(yaml.safe_dump({'/**': {'ros__parameters': params}}))
    context = Context()
    context.init(args=[], domain_id=domain)
    node = Node('boundary_observer_' + uuid.uuid4().hex[:8], context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    vendor = node.create_publisher(JointState, prefix + '/vendor', qos_profile_sensor_data)
    command = node.create_publisher(JointState, prefix + '/command', 10)
    states, left, right = [], [], []
    subscriptions = [
        node.create_subscription(JointState, prefix + '/state', states.append, qos_profile_sensor_data),
        node.create_subscription(Float64MultiArray, prefix + '/left', lambda m: left.append(list(m.data)), 10),
        node.create_subscription(Float64MultiArray, prefix + '/right', lambda m: right.append(list(m.data)), 10)]
    process = None
    log = tmp_path / 'driver.log'

    def pump(duration, stamp=None, feedback=True):
        end = time.monotonic() + duration
        next_send = 0.
        while time.monotonic() < end:
            assert process.poll() is None, log.read_text()
            current = time.monotonic()
            if feedback and current >= next_send:
                next_send = current + .01
                msg = JointState()
                msg.header.stamp = stamp or node.get_clock().now().to_msg()
                msg.name, msg.position = list(names), [0.] * 4
                vendor.publish(msg)
            executor.spin_once(timeout_sec=.001)

    def send(joints, values, stamp=None):
        msg = JointState()
        msg.header.stamp = stamp or node.get_clock().now().to_msg()
        msg.name, msg.position = joints, values
        command.publish(msg)
        return msg.header.stamp

    try:
        with log.open('w') as stream:
            process = subprocess.Popen([executable, '--ros-args', '--params-file', str(config)],
                                       env=env, stdout=stream, stderr=stream)
            old = node.get_clock().now().to_msg()
            old.sec -= 10
            pump(.7, old)
            assert states == [], 'stale vendor feedback was restamped as fresh'
            pump(.25)
            assert len(states) >= 5
            # One delayed sample may be forwarded once, preserving its age.
            pump(.02, feedback=False)
            frozen = node.get_clock().now().to_msg()
            before = len(states)
            pump(.18, frozen)
            stamps = [s.header.stamp.sec * 10**9 + s.header.stamp.nanosec for s in states[before:]]
            assert len(stamps) <= 2, 'cached or replayed feedback kept being republished'
            before = len(states)
            pump(.08, frozen)
            assert len(states) == before
            pump(.15)  # Valid feedback resumes automatically.
            assert len(states) > before
            left.clear()
            send([names[0]], [.91], old)
            pump(.035)
            assert not any(abs(v[0] - .91) < 1e-8 for v in left)
            good = send([names[0]], [.25])
            pump(.035)
            assert any(v == [.25, 0.] for v in left)
            before = len(left)
            send([names[0]], [.88], good)
            send([names[0]], [.89], old)
            pump(.035)
            assert not any(v[0] in (.88, .89) for v in left[before:])
            # Freeze only this fake process while both groups queue commands.
            # Resume with sub-100 ms samples: retain last values for each arm.
            process.send_signal(signal.SIGSTOP)
            try:
                for i in range(3):
                    send([names[0]], [.30 + i * .01])
                    send([names[2]], [-.30 - i * .01])
                    time.sleep(.001)
            finally:
                process.send_signal(signal.SIGCONT)
            before_l, before_r = len(left), len(right)
            pump(.04)
            assert left[before_l:] == [[.32, 0.]], left[before_l:]
            assert right[before_r:] == [[-.32, 0.]], right[before_r:]
            # Expired packets cannot keep the command watchdog alive.
            for _ in range(5):
                send([names[0]], [.99], old)
                pump(.035)
            assert left[-1] == [0., 0.] and right[-1] == [0., 0.]
    finally:
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        executor.shutdown()
        node.destroy_node()
        context.shutdown()
