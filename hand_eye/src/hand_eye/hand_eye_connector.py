#!/usr/bin/env python
import rospy
import tf
import tf_conversions.posemath as tfconv
import PyKDL
import cv2
from numpy import matrix
import numpy as np

from geometry_msgs.msg import Vector3, Quaternion, Transform, TransformStamped
from visp_hand2eye_calibration.msg import TransformArray
from visp_hand2eye_calibration.srv import compute_effector_camera_quick
from aruco_hand_eye.srv import aruco_info, aruco_infoRequest, hand_eye_calibration, hand_eye_calibrationResponse

class HandEyeConnector(object):
    def __init__(self):

        # Frame which is rigidly attached to the camera
        # The transform from this frame to the camera optical frame is what
        # we're trying to compute.
        # For the eye-in-hand case, this is the end-effector frame.
        # For the eye-on-base case, this is the world or base frame.
        self.camera_parent_frame_id = rospy.get_param('~camera_parent_frame')

        # Frame which is rigidly attached to the marker
        # The transform from the camera parent frame to the marker parent frame
        # is given by forward kinematics.
        # For the eye-in-hand case, this is the world or base frame.
        # For the eye-on-base case, this is the end-effector frame.
        self.marker_parent_frame_id = rospy.get_param('~marker_parent_frame')

        self.publish_tf = rospy.get_param('~publish_tf')
        self.tf_suffix = rospy.get_param('~tf_suffix')
        self.sample_rate = rospy.get_param('~sample_rate')
        self.interactive = rospy.get_param('~interactive')
        self.marker_id = rospy.get_param('~marker_id')

        # Compute the camera base to optical transform
        self.xyz_optical_base = rospy.get_param('~xyz_optical_base', [0,0,0])
        self.rpy_optical_base = rospy.get_param('~rpy_optical_base', [0,0,0])
        self.F_optical_base = PyKDL.Frame(
                PyKDL.Rotation.RPY(*self.rpy_optical_base),
                PyKDL.Vector(*self.xyz_optical_base))
        self.F_base_optical = self.F_optical_base.Inverse()

        # tf structures
        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()

        # rate limiter
        self.rate = rospy.Rate(self.sample_rate)

        # input data
        self.hand_world_samples = TransformArray()
        self.camera_marker_samples = TransformArray()

        # marker subscriber
        # self.aruco_subscriber = rospy.Subscriber(
        #         'aruco_tracker/transform',
        #         TransformStamped,
        #         self.aruco_cb,
        #         queue_size=1)

        # calibration service

        self.hand_eye_service = rospy.Service('hand_eye_calibration', hand_eye_calibration, self.aruco_cb)

        rospy.wait_for_service('compute_effector_camera_quick')
        self.calibrate = rospy.ServiceProxy(
                'compute_effector_camera_quick',
                compute_effector_camera_quick)

        self.caculate = False
        
        

    def aruco_tracker(self, cmd):
        req = aruco_infoRequest()
        req.id = self.marker_id
        req.cmd = cmd
        rospy.wait_for_service('get_ar_marker')
        try:
            get_aruco = rospy.ServiceProxy('get_ar_marker', aruco_info)
            res = get_aruco(req)
            return res
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def compute_calibration(self, msg):
        rospy.loginfo("Computing from %g poses..." % len(self.hand_world_samples.transforms) )
        result = None

        # Get the camera optical frame for convenience
        optical_frame_id = msg.header.frame_id
        print('self.camera_marker_sample:',self.camera_marker_samples)
        print('self.hand_world_samples:',self.hand_world_samples)
        try:
            result = self.calibrate(self.camera_marker_samples, self.hand_world_samples)
        except rospy.ServiceException as ex:
            rospy.logerr("Calibration failed: "+str(ex))
            print('FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF')
            return None

        if True: #self.publish_tf:
            self.broadcaster.sendTransform(
                    (result.effector_camera.translation.x, result.effector_camera.translation.y, result.effector_camera.translation.z),
                    (result.effector_camera.rotation.x, result.effector_camera.rotation.y, result.effector_camera.rotation.z, result.effector_camera.rotation.w),
                    rospy.Time.now(),
                    optical_frame_id + self.tf_suffix,
                    self.camera_parent_frame_id)

        rospy.loginfo("Result:\n"+str(result))
        R = tf.transformations.quaternion_matrix([result.effector_camera.rotation.x, result.effector_camera.rotation.y, result.effector_camera.rotation.z, result.effector_camera.rotation.w])
        R[0, 3] = result.effector_camera.translation.x
        R[1, 3] = result.effector_camera.translation.y
        R[2, 3] = result.effector_camera.translation.z

        rospy.loginfo("RESULT:\n"+str(R))
        print('RRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR: /n', R)
        ec = result.effector_camera
        xyz = (ec.translation.x, ec.translation.y, ec.translation.z)
        xyzw = (ec.rotation.x, ec.rotation.y, ec.rotation.z, ec.rotation.w)
        # rpy = tuple(PyKDL.Rotation.Quaternion(*xyzw).GetRPY())

        F_optical_world = PyKDL.Frame(PyKDL.Rotation.Quaternion(*xyzw), PyKDL.Vector(*xyz))
        F_base_world = F_optical_world * self.F_base_optical

        bw = tfconv.toMsg(F_base_world)
        xyz = (bw.position.x, bw.position.y, bw.position.z)
        xyzw = (bw.orientation.x, bw.orientation.y, bw.orientation.z, bw.orientation.w)
        # rpy = tuple(PyKDL.Rotation.Quaternion(*xyzw).GetRPY())

        rospy.loginfo("Base xyz: ( %f %f %f ) xyzw: ( %f %f %f %f )" % (xyz+xyzw))
        R = np.reshape(R, (1,16))[0]
        print('RRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR')
        print(R)
        return R

    def aruco_cb(self, end_trans):
        res = hand_eye_calibrationResponse()
        ar_marker = self.aruco_tracker(end_trans.cmd)
        if end_trans.cmd == 'hello':
            self.caculate = True
        if ar_marker.rvecs is not None:
            r = cv2.Rodrigues(ar_marker.rvecs)[0]
            rotation = [[r[0][0], r[0][1], r[0][2], 0],
                        [r[1][0], r[1][1], r[1][2], 0],
                        [r[2][0], r[2][1], r[2][2], 0],
                        [    0,     0,     0,       1]]
            print("rotation is ", rotation)
            quaternion = tf.transformations.quaternion_from_matrix(rotation)
            print("quaternion is ", quaternion)
            msg = TransformStamped()
            msg.transform.translation.x = ar_marker.tvecs[0]
            msg.transform.translation.y = ar_marker.tvecs[1]
            msg.transform.translation.z = ar_marker.tvecs[2]
            msg.transform.rotation.x = quaternion[0]
            msg.transform.rotation.y = quaternion[1]
            msg.transform.rotation.z = quaternion[2]
            msg.transform.rotation.w = quaternion[3]
            msg.header.frame_id = 'ar_marker'
            
            rospy.loginfo("Received marker sample.")
    
            # Get the camera optical frame for convenience
            optical_frame_id = msg.header.frame_id
    
            # try:
            #     # Get the transform between the marker and camera frames (from FK)
            #     self.listener.waitForTransform(
            #         self.marker_parent_frame_id, self.camera_parent_frame_id,
            #         msg.header.stamp, rospy.Duration(0.1))
    
            #     (trans,rot) = self.listener.lookupTransform(
            #         self.marker_parent_frame_id, self.camera_parent_frame_id,
            #         msg.header.stamp)
            # except tf.Exception as ex:
            #     rospy.logwarn(str(ex))
            #     return
    
            # Update data
            self.hand_world_samples.header.frame_id = 'ee'#optical_frame_id
            self.hand_world_samples.transforms.append(end_trans.end_trans)
    
            self.camera_marker_samples.header.frame_id = 'cc'#optical_frame_id
            self.camera_marker_samples.transforms.append(msg.transform)
    
        if len(self.hand_world_samples.transforms) != len(self.camera_marker_samples.transforms):
            rospy.logerr("Different numbers of hand-world and camera-marker samples.")
            res.is_done = False
            return res

        n_min = 22
        # n_min = 10
        
        if len(self.hand_world_samples.transforms) < n_min and self.caculate == False:
            rospy.logwarn("%d more samples needed..." % (n_min-len(self.hand_world_samples.transforms)))
            res.is_done = False
            return res
        else:
            res.end2cam_trans = self.compute_calibration(msg)
            res.is_done = True
            return res
        # interactive
        if self.interactive:
            i = raw_input('Hit [enter] to accept this latest sample, or `d` to discard: ')
            if i == 'd':
                del self.hand_world_samples.transforms[-1]
                del self.camera_marker_samples.transforms[-1]
                self.compute_calibration(msg)
            raw_input('Hit [enter] to capture the next sample...')
        else:
            self.rate.sleep()
