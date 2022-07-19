import time
import rospy
from threading import Lock, Condition
from std_msgs.msg import Header
from sensor_msgs.msg import CameraInfo
from message_filters import Cache, Subscriber, ApproximateTimeSynchronizer


class ServiceProxyFailed(Exception):
    pass


class RobustServiceProxy(rospy.ServiceProxy):
    def __init__(self, name, service_class, persistent=False, headers=None, retry_count=2, backoff=0.01, block_max=0.05):
        super(RobustServiceProxy, self).__init__(name, service_class, persistent, headers)
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
                    raise ServiceProxyFailed(e) #from e
                time.sleep(min(self.backoff*(i + 1), self.block_max - cur_dur))

    def __call__(self, *args, **kwds):
        return self.call_with_retry(*args, **kwds)


class CameraInfoHelper:
    def __init__(self, camera_info_topic): #: str) -> None:
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

    def get_last_camera_info(self): # -> CameraInfo:
        with self._cache_lock:
            ret = self._camera_info_cache.getLast()
        if ret is None:
            raise RuntimeError('Camera info not recieved yet')
        return ret

    def _cache_add_callback(self, *args):
        with self._cache_add_condition:
            self._cache_add_condition.notify_all()


class HeaderCalc:
    def __init__(self, frame_id): #: str):
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


class SynchronizerMinTick(ApproximateTimeSynchronizer):
    """
    Calls a callback if one of two conditions occur:
      1. A group of messages have timestamps within self.slop
      2. An above group has not occured within self.min_tick
    In other words, this class seeks to synchronize incoming messages with the constraint that
    we would also like the callback to be invoked at a minimum frequency. This behavior allows
    for graceful failure when a sensor stops publishing unexpectedly.
    If the min_tick timer elapses and not all messages are ready, the callback will be invoked
    with the earliest message from the highest priority subscriber (in order passed to the constructor),
    all topics that do not have a message recent enough will be filled with None.
    """

    def __init__(self, fs, queue_size, slop, min_tick): #int, slop: float, min_tick: float):
        super(SynchronizerMinTick, self).__init__(fs, queue_size, slop)
        self.min_tick = rospy.Duration.from_sec(min_tick)
        self.last_tick = rospy.Time()
        self.tick_timer = rospy.Timer(self.min_tick, self._timerCallback, reset=True, oneshot=True)

    def signalMessage(self, *msg):
        if self.tick_timer:
            self.tick_timer.shutdown()

        self.last_tick = rospy.Time()
        self.tick_timer = rospy.Timer(self.min_tick, self._timerCallback, reset=True, oneshot=True)
        return super(SynchronizerMinTick, self).signalMessage(*msg)

    def _timerCallback(self, _):
        # signalMessage with whatever the latest data is
        with self.lock:
            found_msg = None
            for q in self.queues:
                if not q:
                    continue
                highest_stamp = max(q)
                if rospy.Time.now() - highest_stamp <= self.min_tick:
                    found_msg = q[highest_stamp]

            if not found_msg:
                # nothing has pinged within the tick rate
                rospy.logwarn('SynchronizerMinTick: no packets within min tick rate')
                ret = [None for _ in range(len(self.queues))]
            else:
                target_stamp = found_msg.header.stamp
                # collect all things that have pinged within the slop
                ret = []
                for q in self.queues:
                    if not q:
                        ret.append(None)
                        continue
                    best_stamp_diff, best_stamp = min((abs(target_stamp - s), s) for s in q)
                    if best_stamp_diff <= self.slop:
                        ret.append(q[best_stamp])
                    else:
                        ret.append(None)

            # empty queue and return
            self.signalMessage(*ret)
            for q in self.queues:
                q.clear()
