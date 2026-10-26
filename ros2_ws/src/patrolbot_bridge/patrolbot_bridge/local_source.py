"""Bounded local ROS graph provenance, NOT DDS authentication/authorization.

Jazzy rclpy exposes dictionary timestamps without publisher_gid on some builds.
Require the expected sole endpoint to remain stable longer than the input age
budget, and discard samples generated before that observation. When per-sample
GIDs exist, additionally bind the sample to the endpoint. SROS/DDS access policy
is still required to exclude a malicious participant impersonating a local node.
"""


class LocalSource:
    def __init__(self, topic, node_name, type_name, maximum_age):
        self.topic, self.node_name, self.type_name = topic, node_name, type_name
        self.maximum_age_ns = int(maximum_age * 1e9)
        self.endpoint = None
        self.since_ns = None
        self.last_stamp = None

    def reset(self):
        self.endpoint = self.since_ns = self.last_stamp = None

    def accepts(self, node, info):
        """Fail closed on duplicates, missing timestamps or changed endpoints."""
        try:
            endpoints = node.get_publishers_info_by_topic(self.topic)
            if (len(endpoints) != 1 or endpoints[0].node_name != self.node_name
                    or endpoints[0].node_namespace != '/'
                    or endpoints[0].topic_type != self.type_name):
                self.reset()
                return False
            gid = bytes(endpoints[0].endpoint_gid)
            if not gid or not any(gid):
                self.reset()
                return False
            now = node.get_clock().now().nanoseconds
            get = info.get if isinstance(info, dict) else lambda key: getattr(info, key, None)
            stamp = get('source_timestamp')
            if type(stamp) is not int or not 0 < stamp <= now or now - stamp > self.maximum_age_ns:
                self.reset()
                return False
            if self.endpoint != gid or self.since_ns is None or now < self.since_ns:
                self.endpoint, self.since_ns, self.last_stamp = gid, now, None
                return False
            sample_gid = get('publisher_gid')
            if sample_gid is not None and bytes(sample_gid) != gid:
                self.reset()
                return False
            if (now - self.since_ns < self.maximum_age_ns or stamp <= self.since_ns
                    or (self.last_stamp is not None and stamp <= self.last_stamp)):
                return False
            self.last_stamp = stamp
            return True
        except (AttributeError, TypeError, ValueError, OverflowError):
            self.reset()
            return False
