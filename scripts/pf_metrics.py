#!/usr/bin/env python3

""" Measures how the particle filter performs from its /particle_cloud output.

    Counts filter updates, reports when the cloud converges (gets tight) and when it
    gets lost / recovers, and saves every update to a CSV file for the writeup.

    Spread is measured on the main cluster only (the closest --core-fraction of particles),
    so the random particles injected every resample don't hide convergence.

    A tight cloud is not necessarily in the right place. With --ref, the estimate is compared to
    a known-good run of the same input bag, and only counts as converged if it is also close to it.

    Usage:
        live, in its own terminal while the filter runs:
            python3 scripts/pf_metrics.py --out results/take1.csv
        offline, from a bag that recorded /particle_cloud and /initialpose:
            python3 scripts/pf_metrics.py --bag bags/pf_result_take1 --out results/take1.csv
        compared to a known-good run of the same input bag:
            python3 scripts/pf_metrics.py --bag bags/pf_result_take1_kidnap --out results/take1_kidnap.csv \
                --ref results/take1_run1.csv
"""

import argparse
import bisect
import csv
import math
import os

from nav2_msgs.msg import ParticleCloud
from geometry_msgs.msg import PoseWithCovarianceStamped


def yaw_from_quaternion(q):
    """ Yaw (rotation about z) of a quaternion, in radians """
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def median(values):
    s = sorted(values)
    n = len(s)
    return s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2])


def cloud_stats(msg, core_fraction):
    """ Pose and spread of the main cluster of the particle cloud, plus the spread of the whole cloud """
    xs = [p.pose.position.x for p in msg.particles]
    ys = [p.pose.position.y for p in msg.particles]
    thetas = [yaw_from_quaternion(p.pose.orientation) for p in msg.particles]
    n = len(xs)

    # whole-cloud spread (includes the random particles scattered over the map)
    mx, my = sum(xs) / n, sum(ys) / n
    full_spread = math.sqrt(sum((x - mx) ** 2 + (y - my) ** 2 for x, y in zip(xs, ys)) / n)

    # main cluster: the particles closest to the median position (the median ignores far-away outliers)
    cx, cy = median(xs), median(ys)
    by_distance = sorted(range(n), key=lambda i: (xs[i] - cx) ** 2 + (ys[i] - cy) ** 2)
    core = by_distance[:max(1, int(core_fraction * n))]

    mean_x = sum(xs[i] for i in core) / len(core)
    mean_y = sum(ys[i] for i in core) / len(core)
    spread_xy = math.sqrt(sum((xs[i] - mean_x) ** 2 + (ys[i] - mean_y) ** 2 for i in core) / len(core))
    s = sum(math.sin(thetas[i]) for i in core) / len(core)
    c = sum(math.cos(thetas[i]) for i in core) / len(core)
    # circular standard deviation: 0 when all headings agree, grows as they spread out
    resultant = min(max(math.hypot(s, c), 1e-12), 1.0)
    return {'n': n, 'mean_x': mean_x, 'mean_y': mean_y, 'mean_theta': math.atan2(s, c),
            'spread_xy': spread_xy, 'spread_theta': math.sqrt(-2.0 * math.log(resultant)),
            'full_spread_xy': full_spread}


class Reference(object):
    """ Known-good estimate over time, loaded from a pf_metrics CSV of the same input bag """

    def __init__(self, path):
        rows = [r for r in csv.DictReader(open(path)) if r['state'] == 'converged']
        self.times = [float(r['sim_time_s']) for r in rows]
        self.xy = [(float(r['mean_x']), float(r['mean_y'])) for r in rows]

    def pose_at(self, t):
        """ Reference x, y at sim time t (interpolated), or None outside the reference """
        i = bisect.bisect_left(self.times, t)
        if i == 0 or i >= len(self.times):
            return None
        t0, t1 = self.times[i - 1], self.times[i]
        f = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
        (x0, y0), (x1, y1) = self.xy[i - 1], self.xy[i]
        return (x0 + f * (x1 - x0), y0 + f * (y1 - y0))


class Tracker(object):
    """ Turns a stream of particle clouds into update counts and converged / lost / recovered events """

    def __init__(self, args, log):
        self.args = args
        self.log = log
        self.update_count = 0 # filter updates since the last 2D Pose Estimate
        self.start_time = None # sim time of the last pose estimate
        self.state = 'waiting'
        self.lost_at_update = None
        self.last_first_pose = None # used to tell when the cloud actually changed
        self.events = []
        self.ref = Reference(args.ref) if args.ref else None
        self.error = None #distance to the reference estimate, if known

        os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
        self.csv_file = open(args.out, 'w', newline='')
        self.csv = csv.writer(self.csv_file)
        self.csv.writerow(['sim_time_s', 'update', 'state', 'n_particles', 'mean_x', 'mean_y', 'mean_theta',
                           'spread_xy_m', 'spread_theta_rad', 'full_spread_xy_m', 'error_m'])

    def initial_pose(self, msg):
        """ A new 2D Pose Estimate restarts the count """
        self.update_count = 0
        # rviz may stamp this with wall time, so start the clock at the next cloud instead
        self.start_time = None
        self.state = 'converging'
        self.last_first_pose = None
        self.log("New initial pose: counting updates from 0")

    def cloud(self, msg):
        if not msg.particles:
            return
        # pf.py republishes the same cloud every loop; only count it when the particles moved
        first = msg.particles[0].pose.position
        first_pose = (first.x, first.y)
        if first_pose == self.last_first_pose:
            return
        is_init = self.last_first_pose is None
        self.last_first_pose = first_pose
        if not is_init:
            self.update_count += 1

        stats = cloud_stats(msg, self.args.core_fraction)
        t = stamp_to_sec(msg.header.stamp)
        if self.start_time is None:
            self.start_time = t
        elapsed = t - self.start_time
        self.error = None
        if self.ref:
            ref_xy = self.ref.pose_at(t)
            if ref_xy:
                self.error = math.hypot(stats['mean_x'] - ref_xy[0], stats['mean_y'] - ref_xy[1])
        self.check_state(stats, elapsed)

        self.csv.writerow([f"{t:.2f}", self.update_count, self.state, stats['n'],
                           f"{stats['mean_x']:.3f}", f"{stats['mean_y']:.3f}", f"{stats['mean_theta']:.3f}",
                           f"{stats['spread_xy']:.3f}", f"{stats['spread_theta']:.3f}",
                           f"{stats['full_spread_xy']:.3f}",
                           f"{self.error:.3f}" if self.error is not None else ''])
        self.csv_file.flush()
        if not self.args.quiet:
            err = f" | error {self.error:.2f} m" if self.error is not None else ''
            self.log(f"update {self.update_count:3d} | {self.state:11s} | "
                     f"spread {stats['spread_xy']:.2f} m, {math.degrees(stats['spread_theta']):5.1f} deg{err}")

    def check_state(self, stats, elapsed):
        """ Report convergence, getting lost, and recovery """
        tight = stats['spread_xy'] < self.args.xy_thresh and stats['spread_theta'] < self.args.theta_thresh
        loose = stats['spread_xy'] > self.args.xy_thresh * self.args.lost_factor
        if self.error is not None:
            # with a reference, a tight cloud in the wrong place is not converged
            if tight and self.error > self.args.error_thresh:
                if self.state != 'wrong_place':
                    self.state = 'wrong_place'
                    self.event(f"TIGHT BUT WRONG PLACE at update {self.update_count} ({elapsed:.1f} s): "
                               f"{self.error:.2f} m from reference")
                return
            if self.state == 'wrong_place':
                self.state = 'converging'
            if self.state == 'converged' and self.error > self.args.error_thresh:
                loose = True
        if self.state == 'converging' and tight:
            self.state = 'converged'
            self.event(f"CONVERGED after {self.update_count} updates ({elapsed:.1f} s)")
        elif self.state == 'converged' and loose:
            self.state = 'lost'
            self.lost_at_update = self.update_count
            self.event(f"LOST at update {self.update_count} ({elapsed:.1f} s)")
        elif self.state == 'lost' and tight:
            self.state = 'converged'
            n = self.update_count - self.lost_at_update
            self.event(f"RECOVERED after {n} updates (at update {self.update_count})")

    def event(self, text):
        self.events.append(text)
        self.log(f"*** {text} ***")

    def close(self):
        self.csv_file.close()
        self.log("Summary: " + ("; ".join(self.events) if self.events else
                                f"never converged (state: {self.state}, {self.update_count} updates)"))
        self.log(f"Results saved to {self.args.out}")


def run_live(args):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data

    rclpy.init()
    node = Node('pf_metrics')
    # follow the bag / Gazebo clock so times match the recording
    node.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    tracker = Tracker(args, node.get_logger().info)
    node.create_subscription(ParticleCloud, 'particle_cloud', tracker.cloud, qos_profile_sensor_data)
    node.create_subscription(PoseWithCovarianceStamped, 'initialpose', tracker.initial_pose, 10)
    node.get_logger().info("Waiting for a 2D Pose Estimate click...")
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        tracker.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def run_bag(args):
    import rosbag2_py
    from rclpy.serialization import deserialize_message

    # bags recorded with --compression-mode file need the compression-aware reader
    metadata = rosbag2_py.Info().read_metadata(args.bag, '')
    if metadata.compression_format:
        reader = rosbag2_py.SequentialCompressionReader()
    else:
        reader = rosbag2_py.SequentialReader()
    # the compression reader writes a decompressed copy next to the bag; remember what was there to clean up after
    files_before = set(os.listdir(args.bag))
    reader.open_uri(args.bag)
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/particle_cloud', '/initialpose']))
    tracker = Tracker(args, print)
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == '/initialpose':
            tracker.initial_pose(deserialize_message(data, PoseWithCovarianceStamped))
        else:
            tracker.cloud(deserialize_message(data, ParticleCloud))
    del reader
    for name in set(os.listdir(args.bag)) - files_before:
        os.remove(os.path.join(args.bag, name))
    tracker.close()


def main():
    parser = argparse.ArgumentParser(description="Measure particle filter convergence from /particle_cloud")
    parser.add_argument('--out', default='results/pf_metrics.csv', help="CSV file to write")
    parser.add_argument('--bag', help="read a recorded bag instead of listening live")
    parser.add_argument('--xy-thresh', type=float, default=0.25, help="converged if position spread below this (m)")
    parser.add_argument('--theta-thresh', type=float, default=0.25, help="converged if heading spread below this (rad)")
    parser.add_argument('--lost-factor', type=float, default=3.0, help="lost if spread exceeds thresh times this")
    parser.add_argument('--core-fraction', type=float, default=0.9,
                        help="fraction of particles closest to the center used for spread (ignores random ones)")
    parser.add_argument('--ref', help="pf_metrics CSV of a known-good run of the same input bag, to measure error")
    parser.add_argument('--error-thresh', type=float, default=0.5,
                        help="with --ref, converged only if within this distance of the reference (m)")
    parser.add_argument('--quiet', action='store_true', help="only print converged / lost / recovered events")
    args = parser.parse_args()

    if args.bag:
        run_bag(args)
    else:
        run_live(args)


if __name__ == '__main__':
    main()
