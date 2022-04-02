import time
import rospy
from threading import Lock, Condition
from std_msgs.msg import Header
from sensor_msgs.msg import CameraInfo
from message_filters import Cache, Subscriber


class ServiceProxyFailed(Exception):
    pass


class RobustServiceProxy(rospy.ServiceProxy):
    def __init__(self, name, service_class, persistent=False, headers=None, retry_count=2, backoff=0.01, block_max=0.05):
        super().__init__(name, service_class, persistent, headers)
        self.retry_count = retry_count
        self.backoff = backoff
        self.block_max = block_max

    def call_with_retry(self, *args, **kwargs):
        start = time.time()
        for i in range(self.retry_count):
            try:
                return self.call(*args, **kwargs)
            except Exception as e:
                cur_dur = time.time() - start
                if self.persistent and (isinstance(e, rospy.TransportTerminated) \
                    or (isinstance(e, rospy.ServiceException) and 'transport' in e.args[0])):
                    # restart connection, presumably transport failed
                    self.transport = None
                if i == self.retry_count - 1 or cur_dur >= self.block_max:
                    raise ServiceProxyFailed(e) from e
                time.sleep(min(self.backoff*(i + 1), self.block_max - cur_dur))

    def __call__(self, *args, **kwds):
        return self.call_with_retry(*args, **kwds)


class CameraInfoHelper:
    def __init__(self, camera_info_topic: str) -> None:
        self.camera_info_topic = camera_info_topic
        self._cache_lock = Lock()
        self._cache_add_condition = Condition(self._cache_lock)
        self._camera_info_sub = Subscriber(self.camera_info_topic, CameraInfo)
        self._camera_info_cache = Cache(self._camera_info_sub, cache_size=1)
        self._camera_info_cache.registerCallback(self._cache_add_callback)

    def wait_for_camera_info(self):
        with self._cache_add_condition:
            while not len(self._camera_info_cache.cache_msgs):
                self._cache_add_condition.wait()

    def get_last_camera_info(self) -> CameraInfo:
        with self._cache_lock:
            ret = self._camera_info_cache.getLast()
        if ret is None:
            raise RuntimeError('Camera info not recieved yet')
        return ret

    def _cache_add_callback(self, *args):
        with self._cache_add_condition:
            self._cache_add_condition.notify_all()


class HeaderCalc:
    def __init__(self, frame_id: str):
        self.frame_id = frame_id
        self._seq = 0

    def get_header(self):
        now = rospy.get_rostime()
        fake_header = Header(seq=self._seq, stamp=now, frame_id=self.frame_id)
        if self._seq >= 4294967295:  # uint32 max
            self._seq = 0
        else:
            self._seq += 1

        return fake_header
