"""Small local adapter over the shared Crazyflie Python client."""

from crazyflie_py import Crazyswarm
from crazyflie_py.crazyflie import CrazyflieServer


def root_services_only(names_and_types):
    """Keep '/cf1/start_trajectory', drop '/real/cf1/start_trajectory'.

    crazyflie_py discovers robots from every StartTrajectory service in the graph. In mirror
    mode a second (real) server runs under a namespace and must not be taken for simulated robots.
    """
    return [(name, types) for name, types in names_and_types if name.count("/") == 2]


class DroneStack:
    def __init__(self):
        discover = CrazyflieServer.get_service_names_and_types
        CrazyflieServer.get_service_names_and_types = lambda node: root_services_only(discover(node))
        try:
            self.swarm = Crazyswarm()
        finally:
            CrazyflieServer.get_service_names_and_types = discover
        self.allcfs = self.swarm.allcfs

    def names(self):
        return [cf.prefix.strip('/') for cf in self.allcfs.crazyflies]

    def get(self, name):
        for cf in self.allcfs.crazyflies:
            if cf.prefix.strip('/') == name:
                return cf
        raise KeyError(name)

    def takeoff(self, height, duration, wait=False):
        self.allcfs.takeoff(targetHeight=height, duration=duration)
        if wait:
            self.swarm.timeHelper.sleep(duration)

    def land(self, height, duration, wait=False):
        self.allcfs.land(targetHeight=height, duration=duration)
        if wait:
            self.swarm.timeHelper.sleep(duration)
