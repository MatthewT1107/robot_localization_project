"""
Unit tests for pf.py

Took inspiration from a discord thread between River and Victoria about interfacing with pytest.
Implemented unit tests so they don't depend on ROS using 'toy test classes.' This just tests logical functionalities.
 
Run using 'python3 -m pytest unit_tests.py -v' in robot_localization folder
"""

import math
import numpy as np
import pytest

from helper_functions import TFHelper
from pf import Particle, ParticleFilter

# Stand-ins for the parts of ParticleFilter that need ROS
class InertTFHelper(TFHelper):
    """
    Uses TFHelper math without the tf2 buffer/listener that the real __init__ sets up.
    """

    def __init__(self):  
        pass # skip TFHelper.__init__  because it needs a ROS node

    def fix_map_to_odom_transform(self, *args, **kwargs):
        pass  # the real one publishes a transform that is not needed here


class InertOccupancyField:
    """
    Fake map: free space is x, y in [0, 5], 
    x<1 means inside object,
    Outside the map the distance is NaN.
    """

    def get_closest_obstacle_distance(self, x, y):
        if not (0.0 <= x <= 5.0 and 0.0 <= y <= 5.0):
            return float("nan")
        return 0.0 if x < 1.0 else 1.0


class InertNode:
    """
    Stand-in for ParticleFilter. Methods under test use `self`, so we hand
    it the attributes they read, and borrow the real helper methods they call."""

    # Real methods
    normalize_particles = ParticleFilter.normalize_particles
    update_robot_pose = ParticleFilter.update_robot_pose
    random_particle = ParticleFilter.random_particle

    # x and y mins and maxes of fake map
    x_min, x_max, y_min, y_max = 0.0, 5.0, 0.0, 5.0

    def __init__(self):
        # Set per instance (not on the class) so tests can't leak state into each other
        self.particle_cloud = []
        self.odom_pose = Particle(x=0, y=0, theta=0).as_pose()
        self.current_odom_xy_theta = (0, 0, 0)
        self.transform_helper = InertTFHelper()
        self.occupancy_field = InertOccupancyField()
        self.w_fast = None  # kidnap-detection averages (None = no scan seen yet)
        self.w_slow = None

# builds inert node for each test
@pytest.fixture
def node():
    return InertNode()

# turn off noise and randomness in motion and resampling code
@pytest.fixture
def no_noise(monkeypatch):
    """
    Make np.random.normal always return 0 so motion/resampling noise is off.
    """
    monkeypatch.setattr(np.random, "normal", lambda *args, **kwargs: 0.0)


# ----------------------------Tests------------------------------------------------------
def test_normalize_particles_sums_to_one_and_handles_zero_weights(node):
    """
    Test all weights sum to one and weight ratio is kept.
    Test if all weights are zero go back to a uniform distribution instead of dividing by 0.
    """

    # 2 particles
    node.particle_cloud = [Particle(w=1.0), Particle(w=3.0)]
    ParticleFilter.normalize_particles(node)
    # particles sum to 1
    assert sum(p.w for p in node.particle_cloud) == pytest.approx(1.0)
    # check ratio
    assert node.particle_cloud[1].w / node.particle_cloud[0].w == pytest.approx(3.0)

    # 4 particle with weight 0
    node.particle_cloud = [Particle(w=0.0) for _ in range(4)]
    ParticleFilter.normalize_particles(node)
    # Particles have four shares of 1.0
    assert [p.w for p in node.particle_cloud] == pytest.approx([0.25] * 4)

def test_update_particles_with_odom_wraps_heading_past_pi(node, no_noise):
    """ 
    Test that a particle facing 3.0 rad while the robot turns +0.5 rad would be at 3.5 rad,
    which is past pi. It must wrap around to 3.5 - 2*pi (about -2.78), and the
    stored heading must stay inside [-pi, pi]
    """

    # particle with theta 3
    node.particle_cloud = [Particle(x=0, y=0, theta=3.0)]
    # set base odom
    node.current_odom_xy_theta = (0, 0, 0)
    # robot turns +0.5 rad in place
    node.odom_pose = Particle(x=0, y=0, theta=0.5).as_pose()  
    ParticleFilter.update_particles_with_odom(node)
    theta = node.particle_cloud[0].theta
    
    assert -math.pi <= theta <= math.pi
    # 3.5 - 2pi gives equivalent angle 
    assert theta == pytest.approx(3.5 - 2 * math.pi, abs=1e-9)


def test_update_robot_pose_averages_headings_across_the_pi_wrap(node):
    """ 
    Test that filter averages headings when they sit on opposite sides of -pi and +pi boundary.
    Headings of +3.1 and -3.1 rad both point almost backwards, so the estimate should face ~pi.
    """

    # two particles diff theta
    node.particle_cloud = [
        Particle(x=0, y=0, theta=3.1, w=0.5),
        Particle(x=0, y=0, theta=-3.1, w=0.5),
    ]
    # update pose takes weighted mean of both particles
    ParticleFilter.update_robot_pose(node)

    # pose stores heading as quaternion
    q = node.robot_pose.orientation
    # quaternion to angle
    yaw = 2.0 * math.atan2(q.z, q.w)
    # cos(yaw) near -1 means "pointing backwards" (a wrong mean of 0 would give +1)
    assert math.cos(yaw) < -0.99

def test_initialize_particle_cloud_always_creates_full_cloud_of_valid_particles(node):
    """
    Make sure initial particle cloud always creates 1050 valid particles. The count is checked repeatedly because the number
    of rejected random draws changes every run. Every particle must be
    on a free cell and the weights must form a distribution.
    """
    for _ in range(10):
        ParticleFilter.initialize_particle_cloud(node, 0, (3.0, 3.0, 0.0))
        # particle cloud length is accurate
        assert len(node.particle_cloud) == 1050
        # every particle is valid
        for p in node.particle_cloud:
            d = node.occupancy_field.get_closest_obstacle_distance(p.x, p.y)
            assert math.isfinite(d) and d > 0.01
        # weights sum to 1
        assert sum(p.w for p in node.particle_cloud) == pytest.approx(1.0)