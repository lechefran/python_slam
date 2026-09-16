"""Small, bounded archive of accepted views for map-based tracking recovery."""


class RecoveryKeyframes:
    """Keep live frame references, so BA and landmark culling remain authoritative.

    These are recovery representatives, not a new mapping/BA keyframe system.
    Half-second spacing avoids storing every near-identical video frame. When
    full, thin the most crowded interior interval, preserving both endpoints.
    """

    def __init__(self, capacity=64, interval_seconds=.5):
        if capacity < 3 or interval_seconds <= 0:
            raise ValueError('Recovery archive requires capacity >= 3 and positive spacing')
        self.capacity = capacity
        self.interval_seconds = interval_seconds
        self.frames = []

    def add(self, frame):
        """Call only after a camera has been committed to the existing map."""
        if self.frames and frame.timestamp - self.frames[-1].timestamp < self.interval_seconds:
            return
        self.frames.append(frame)
        if len(self.frames) > self.capacity:
            remove = min(range(1, len(self.frames) - 1),
                         key=lambda i: (self.frames[i + 1].timestamp - self.frames[i - 1].timestamp, i))
            self.frames.pop(remove)

    def candidates(self, current, reference):
        """Exclude the normal reference and views too recent to be alternatives."""
        return [frame for frame in self.frames if frame is not reference
                and current.timestamp - frame.timestamp >= self.interval_seconds]
