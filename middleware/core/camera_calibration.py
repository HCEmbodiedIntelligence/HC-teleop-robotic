"""Build camera-adjacent MCAP calibration documents from recorded CameraInfo.

No mounting transform is inferred from CameraInfo.R or CameraInfo.P.
"""
import copy
import math


class CameraCalibrations:
    def __init__(self):
        self.streams = {}
        self.written = {}

    def observe(self, topic, info, received_ns):
        if not topic.endswith('/camera_info'):
            return None
        root = topic.removesuffix('/camera_info')
        stream = root.rsplit('/', 1)[-1]
        if stream in {'color', 'rgb', 'depth', 'aligned_depth_to_color', 'infra1', 'infra2'}:
            root = root.rsplit('/', 1)[0]
            stream = 'rgb' if stream == 'color' else stream
        else:
            stream = 'rgb'
        k, d = list(info.k), list(info.d)
        if (len(k) != 9 or not all(math.isfinite(v) for v in k + d)
                or k[0] <= 0 or k[4] <= 0 or info.width <= 0 or info.height <= 0):
            # Do not carry a previously valid stream into later documents after
            # the driver reports that its calibration is unavailable.
            self.streams.get(root, {}).pop(stream, None)
            self.written.pop(root, None)
            return None
        value = {'fx': k[0], 'fy': k[4], 'cx': k[2], 'cy': k[5],
                 'distortion_model': info.distortion_model,
                 'distortion_coefficients': d, 'frame_id': info.header.frame_id,
                 'camera_info_topic': topic}
        if info.distortion_model == 'plumb_bob' and len(d) == 5:
            value['distortion_order'] = ['k1', 'k2', 'p1', 'p2', 'k3']
        self.streams.setdefault(root, {})[stream] = (value, {'width': info.width, 'height': info.height})
        snapshot = copy.deepcopy(self.streams[root])
        if snapshot == self.written.get(root):
            return None
        document = {'schema': 'hc_camera_calibration/v1', 'camera': root.lstrip('/'),
                    'source': 'recorded_sensor_msgs/CameraInfo',
                    'received_at_ros_time_ns': received_ns, 'resolution': {}}
        for name, (parameters, resolution) in snapshot.items():
            document[name] = parameters
            document['resolution']['color' if name == 'rgb' else name] = resolution
        return root + '/calibration', document, (root, snapshot)

    def committed(self, token):
        # Only suppress duplicates after the writer queue accepts the document.
        root, snapshot = token
        self.written[root] = snapshot
