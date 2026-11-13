#!/usr/bin/env python3
# Copyright (c) 2018-2026, Martin Günther (DFKI GmbH) and contributors
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Rostest comparing scan_to_scan_filter_chain output to rep117_filter.py."""

import math
import queue
import unittest

import rosbag
import rospy
import rostest
from sensor_msgs.msg import LaserScan

CONNECTION_TIMEOUT = 30.0
CONNECTION_SETTLE_TIME = 1.0
MESSAGE_TIMEOUT = 10.0


def rep117_transform(msg):
    """Return the REP 117 ranges produced by the deleted rep117_filter.py."""
    ranges = []
    for dist in msg.ranges:
        if dist < msg.range_min:
            ranges.append(float("-inf"))
        elif dist > msg.range_max:
            ranges.append(float("inf"))
        else:
            ranges.append(dist)
    return ranges


def describe_array_mismatch(name, expected, actual):
    """Describe the first difference between two float arrays, or return None."""
    if len(expected) != len(actual):
        return f"{name} length differs: expected {len(expected)}, got {len(actual)}"
    for i, (exp, act) in enumerate(zip(expected, actual)):
        if math.isnan(exp) and math.isnan(act):
            continue
        if exp != act:
            return f"{name}[{i}] differs: expected {exp!r}, got {act!r}"
    return None


def check_message(index, input_msg, output_msg):
    """Compare one filter chain output message against the REP 117 reference."""
    prefix = f"message {index} (seq {input_msg.header.seq})"
    diagnostic = describe_array_mismatch("ranges", rep117_transform(input_msg), output_msg.ranges)
    if diagnostic is not None:
        raise AssertionError(f"{prefix}: {diagnostic}")

    scalar_fields = [
        "angle_min",
        "angle_max",
        "angle_increment",
        "time_increment",
        "scan_time",
        "range_min",
        "range_max",
    ]
    for field in scalar_fields:
        expected = getattr(input_msg, field)
        actual = getattr(output_msg, field)
        if expected != actual:
            raise AssertionError(f"{prefix}: {field} differs: expected {expected!r}, got {actual!r}")

    # header.seq is not compared: rospy (test publisher) and roscpp (filter
    # chain publisher) each stamp the header with their own publisher sequence
    # number while serializing (rospy.msg.serialize_message,
    # ros/advertise_options.h), so the sequence number never survives the
    # round trip. The remaining header fields are copied verbatim.
    header_fields = [
        ("header.stamp", input_msg.header.stamp, output_msg.header.stamp),
        ("header.frame_id", input_msg.header.frame_id, output_msg.header.frame_id),
    ]
    for name, expected, actual in header_fields:
        if expected != actual:
            raise AssertionError(f"{prefix}: {name} differs: expected {expected!r}, got {actual!r}")

    diagnostic = describe_array_mismatch("intensities", input_msg.intensities, output_msg.intensities)
    if diagnostic is not None:
        raise AssertionError(f"{prefix}: {diagnostic}")


class FilterChainComparer:
    """Publish bag scans on raw_scan and compare the filtered output."""

    def __init__(self, bag_path):
        """Store the bag path and set up the raw_scan / filtered_scan_chain topics."""
        self.bag_path = bag_path
        self.received = queue.Queue()
        self.publisher = rospy.Publisher("raw_scan", LaserScan, queue_size=10)
        self.subscriber = rospy.Subscriber("filtered_scan_chain", LaserScan, self.received.put)

    def wait_for_connection(self):
        """Wait until the filter chain node is subscribed, then let it settle."""
        deadline = rospy.get_time() + CONNECTION_TIMEOUT
        rate = rospy.Rate(10)
        while self.publisher.get_num_connections() < 1:
            if rospy.is_shutdown():
                raise RuntimeError("ROS shut down while waiting for the filter chain connection")
            if rospy.get_time() > deadline:
                raise RuntimeError(
                    f"timed out after {CONNECTION_TIMEOUT:.0f} s waiting for the filter chain connection"
                )
            rate.sleep()
        rospy.sleep(CONNECTION_SETTLE_TIME)

    def run(self):
        """Replay the bag scans and compare every filtered message."""
        bag = rosbag.Bag(self.bag_path)
        count = 0
        try:
            for _, msg, _ in bag.read_messages(topics=["/mobipick/b_scan"]):
                self.publisher.publish(msg)
                try:
                    output_msg = self.received.get(timeout=MESSAGE_TIMEOUT)
                except queue.Empty:
                    raise AssertionError(
                        f"timed out after {MESSAGE_TIMEOUT:.0f} s waiting for filtered message {count}"
                    )
                check_message(count, msg, output_msg)
                count += 1
        finally:
            bag.close()
        print(f"PASS: compared {count} messages, all bit-identical")
        return count


class TestRep117FilterChain(unittest.TestCase):
    """Compare scan_to_scan_filter_chain output to the REP 117 reference."""

    def test_filter_chain(self):
        """Replay the bag scans and compare every filtered message."""
        rospy.init_node("test_rep117_filter_chain")
        bag_path = rospy.get_param("~bag")
        comparer = FilterChainComparer(bag_path)
        comparer.wait_for_connection()
        comparer.run()


if __name__ == "__main__":
    try:
        rostest.rosrun("mir_driver", "test_rep117_filter_chain", TestRep117FilterChain)
    except rospy.ROSInterruptException:
        pass
