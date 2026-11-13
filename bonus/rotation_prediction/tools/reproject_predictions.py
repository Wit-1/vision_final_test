#!/usr/bin/env python3
"""Overlay model armor poses on the source-time image; legacy targets show centers only."""

import math

import cv2
from cv_bridge import CvBridge, CvBridgeError
from ft_msgs.msg import PredictionDebug, Target
import message_filters
import numpy as np
import rospy
from sensor_msgs.msg import CameraInfo, Image


def calibration(info, image, header):
    """检查与 PnP 相同的针孔标定条件，不混用裁剪、鱼眼或其他坐标系。"""
    if (info is None or info.width != image.shape[1] or info.height != image.shape[0] or
            info.header.frame_id != header.frame_id or not header.frame_id or
            info.binning_x not in (0, 1) or info.binning_y not in (0, 1) or
            any((info.roi.x_offset, info.roi.y_offset, info.roi.width, info.roi.height, info.roi.do_rectify))):
        return None
    matrix, distortion = np.asarray(info.K, dtype=np.float64).reshape(3, 3), np.asarray(info.D, dtype=np.float64)
    supported = ((info.distortion_model == 'plumb_bob' and distortion.size in (0, 4, 5)) or
                 (info.distortion_model == 'rational_polynomial' and distortion.size == 8))
    if (not supported or not np.isfinite(matrix).all() or not np.isfinite(distortion).all() or
            matrix[0, 0] <= 0 or matrix[1, 1] <= 0 or
            abs(matrix[0, 1]) > 1e-9 or abs(matrix[1, 0]) > 1e-9 or
            not np.allclose(matrix[2], [0., 0., 1.], rtol=0, atol=1e-9)):
        return None
    return matrix, distortion


def project_pose(pose, width, height, camera):
    """定义一的四端点：左上、右上、右下、左下；先检查每个三维点深度。"""
    if not (math.isfinite(width) and math.isfinite(height) and width > 0 and height > 0):
        return None
    quaternion = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
    position = np.array([pose.position.x, pose.position.y, pose.position.z], dtype=np.float64)
    if not np.isfinite(quaternion).all() or not np.isfinite(position).all() or np.linalg.norm(quaternion) < 1e-9:
        return None
    x, y, z, w = quaternion / np.linalg.norm(quaternion)
    rotation = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                         [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                         [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]], dtype=np.float64)
    points = np.array([[-width/2, -height/2, 0], [width/2, -height/2, 0],
                       [width/2, height/2, 0], [-width/2, height/2, 0]], dtype=np.float64)
    transformed = points @ rotation.T + position
    if np.any(transformed[:, 2] <= .001):
        return None
    pixels, _ = cv2.projectPoints(points, cv2.Rodrigues(rotation)[0], position, *camera)
    pixels = pixels.reshape(4, 2)
    return pixels if np.isfinite(pixels).all() and np.max(np.abs(pixels)) < 1e6 else None


def view_status(points, image_size, margin):
    """图像范围只判断视野，不判断目标之间的实际遮挡。"""
    width, height = image_size
    inside = ((points[:, 0] >= 0) & (points[:, 0] < width) &
              (points[:, 1] >= 0) & (points[:, 1] < height))
    if not inside.all():
        return 'outside FOV'
    safe = ((points[:, 0] >= margin) & (points[:, 0] < width-margin) &
            (points[:, 1] >= margin) & (points[:, 1] < height-margin))
    return 'inside FOV' if safe.all() else 'near image edge'


def draw_quad(image, points, color, thickness):
    """保留真实投影，只裁剪绘图线段，不把出界角点强行吸到图像边缘。"""
    height, width = image.shape[:2]
    rounded = np.rint(points).astype(int)
    for first, second in zip(rounded, np.roll(rounded, -1, axis=0)):
        visible, start, end = cv2.clipLine((0, 0, width, height), tuple(first), tuple(second))
        if visible:
            cv2.line(image, start, end, color, thickness, cv2.LINE_AA)


class PredictionOverlay:
    def __init__(self):
        self.bridge = CvBridge()
        self.info = None
        self.margin = rospy.get_param('~edge_margin_px', 3)
        self.full_pose = rospy.get_param('~full_pose', True)
        if not isinstance(self.margin, int) or self.margin < 0:
            raise ValueError('edge_margin_px must be a nonnegative integer')
        image_topic = rospy.get_param('~image_topic', '/ft/debug/image')
        if self.full_pose:
            state_topic = rospy.get_param('~prediction_topic', '/ft/debug/rotation_prediction')
        else:
            state_topic = rospy.get_param('~target_topic', '/ft/target')
        self.publisher = rospy.Publisher(rospy.get_param('~output_topic', '/ft/debug/prediction_image'), Image, queue_size=1)
        self.camera = rospy.Subscriber(rospy.get_param('~camera_info_topic', '/hk_camera/camera_info'),
                                       CameraInfo, self._camera, queue_size=1)
        self.inputs = [message_filters.Subscriber(image_topic, Image, queue_size=10),
                       message_filters.Subscriber(state_topic, PredictionDebug if self.full_pose else Target, queue_size=10)]
        self.synchronizer = message_filters.TimeSynchronizer(self.inputs, 20, reset=True)
        self.synchronizer.registerCallback(self.receive)

    def _camera(self, message):
        self.info = message

    def receive(self, image, prediction):
        if image.header.frame_id != prediction.header.frame_id:
            return
        try:
            canvas = self.bridge.imgmsg_to_cv2(image, 'bgr8').copy()
            camera = calibration(self.info, canvas, image.header)
            lines = ['Prediction overlay: waiting for matching calibration']
            if camera is not None:
                if self.full_pose:
                    lines = self._full(canvas, prediction, camera)
                else:
                    lines = self._legacy(canvas, prediction, camera, self.margin)
            # 状态文字放在原图底部；输出大小和图像时间戳不变。
            top = max(0, canvas.shape[0] - 24*len(lines) - 12)
            cv2.rectangle(canvas, (0, top), (canvas.shape[1]-1, canvas.shape[0]-1), (20, 20, 20), -1)
            for index, line in enumerate(lines):
                cv2.putText(canvas, line, (10, top+22+24*index), cv2.FONT_HERSHEY_SIMPLEX,
                            .55, (235, 235, 235), 1, cv2.LINE_AA)
            self.publisher.publish(self.bridge.cv2_to_imgmsg(canvas, 'bgr8', image.header))
        except (CvBridgeError, cv2.error, ValueError, OverflowError) as error:
            rospy.logwarn_throttle(2., 'Prediction overlay rejected a frame: %s', error)

    def _full(self, canvas, prediction, camera):
        if not prediction.valid:
            return ['Prediction overlay: no valid observed target / corner geometry']
        current = project_pose(prediction.current, prediction.width_m, prediction.height_m, camera)
        future = project_pose(prediction.predicted, prediction.width_m, prediction.height_m, camera)
        if current is None or future is None:
            return ['Prediction overlay: invalid pose or nonpositive corner depth']
        draw_quad(canvas, current, (220, 80, 230), 2)
        draw_quad(canvas, future, (255, 160, 0), 3)
        status = view_status(future, (canvas.shape[1], canvas.shape[0]), self.margin)
        return ['Purple: current model | Blue: t+{:.3f}s slot {}->{} | {}'.format(
                    prediction.prediction_time_s, prediction.current_slot, prediction.predicted_slot, status),
                'r={:.3f} dr={:+.3f} dh(up)={:+.3f} m | sigma={:.3f}/{:.3f}/{:.3f} | {}'.format(
                    prediction.radius_m, prediction.delta_radius_m, prediction.delta_height_m,
                    prediction.geometry_stddev.x, prediction.geometry_stddev.y, prediction.geometry_stddev.z,
                    'geometry observed' if prediction.geometry_ready else 'geometry warming up')]

    @staticmethod
    def _legacy(canvas, target, camera, margin):
        if not target.valid:
            return ['Legacy target: no valid observed target']
        position = target.position_predict
        point = np.array([position.x, position.y, position.z], dtype=np.float64)
        if not np.isfinite(point).all() or point[2] <= .001:
            return ['Legacy target: invalid prediction depth']
        projected, _ = cv2.projectPoints(point.reshape(1, 3), np.zeros(3), np.zeros(3), *camera)
        pixel = projected.reshape(2)
        if np.isfinite(pixel).all() and np.max(np.abs(pixel)) < 1e6:
            x, y = np.rint(pixel).astype(int)
            if 0 <= x < canvas.shape[1] and 0 <= y < canvas.shape[0]:
                cv2.drawMarker(canvas, (x, y), (0, 220, 255), cv2.MARKER_CROSS, 22, 2)
            return ['Legacy target: predicted CENTER only | ' +
                    view_status(pixel.reshape(1, 2), (canvas.shape[1], canvas.shape[0]), margin)]
        return ['Legacy target: invalid projection']


if __name__ == '__main__':
    rospy.init_node('prediction_overlay')
    overlay = PredictionOverlay()
    rospy.spin()
