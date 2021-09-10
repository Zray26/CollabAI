#!/usr/bin/env python  
import roslib
#roslib.load_manifest('learning_tf')
from threading import Thread
import rospy
import tf
from detect_qr.msg import qrmsg
import numpy as np

class qr_tftransform(object):
    def __init__(self):
        self.listener = tf.TransformListener()
        self.br = tf.TransformBroadcaster()
        self.rate = rospy.Rate(10.0)
        self.qr_width = 0.067
        self.qr_tr_pos = []
        self.qr_bl_pos = []
        self.pl_array_width = []
        self.pl_array_height = []
        self.pl_width_mean = None
        self.pl_height_mean = None
        self.pl_ratio = None # pixel to length ratio
        self.count = 0
        tr1 = Thread(target = self.set_qr_frame)
        tr2 = Thread(target = self.get_qr_frame)
        tr3 = Thread(target = self.get_camera_to_qr_frame)
        self.qr_img_pos_subscriber()
        tr1.start()
        tr2.start()
        tr3.start()
    def set_qr_frame(self):
        while not rospy.is_shutdown():
            self.br.sendTransform((0.06, 0, 0.0),
                            (0.0, 0.0, 0.0, 1.0),
                            rospy.Time.now(),
                            "qr_tr",
                            "ar_marker_7")
            # print("in set")
            self.rate.sleep()
    def get_qr_frame(self):
        while not rospy.is_shutdown():
            # print("in get ")
            try:
                # listener.waitForTransform('/camera_link', '/qr_1', rospy.Time.now(), rospy.Duration(4.0))
                (trans,rot) = self.listener.lookupTransform('/camera_link', '/qr_tr', rospy.Time(0))
                # print(trans)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                # print( " no tf")
                continue
            self.rate.sleep()




    def bolt_cal(self):
#  Translation: [0.219, -0.255, 0.805]
# - Rotation: in Quaternion [-0.000, 0.999, -0.009, -0.036]
#             in RPY (radian) [-3.123, -0.073, -3.142]
#             in RPY (degree) [-178.923, -4.175, -179.998]
# [ -0.9974064, -0.0006484, -0.0719728;
#    0.0006484,  0.9998379, -0.0179932;
#    0.0719728, -0.0179932, -0.9972443 ]

# - Translation: [0.806, -0.219, 0.256]
# - Rotation: in Quaternion [-0.515, -0.475, 0.483, 0.524]
#             in RPY (radian) [-1.554, -0.000, 1.489]
#             in RPY (degree) [-89.036, -0.018, 85.330]
# [  0.0809911, -0.0169558, -0.9965706;
#    0.9967148,  0.0016892,  0.0809740;
#    0.0003104, -0.9998548,  0.0170369 ]
        qr_tr_x = self.qr_tr_pos[0] + 0.5 * self.qr_tr_pos[2]
        qr_tr_y = self.qr_tr_pos[1] + 0.5 * self.qr_tr_pos[3]
        # bolt_x = 719 63 272 128
        bolt_x = 719 + 272*0.5
        bolt_y = 63 + 128 *0.5
        v_ab_x = self.pl_ratio * (bolt_x - qr_tr_x)
        v_ab_y = self.pl_ratio * (bolt_y - qr_tr_y)
        v_ab = [[1, 0 ,0, v_ab_x], [ 0, 1,0, v_ab_y], [0, 0, 1, 0], [0, 0, 0 ,1]]
        # v_ab = []
        T = [[  0.0809911, -0.0169558, -0.9965706, 0.806],
        [0.9967148,  0.0016892,  0.0809740, -0.219],
        [0.0003104, -0.9998548,  0.0170369, 0.256  ],
        [0, 0, 0, 1]]
        v_ab_qr_frame = np.dot(T, v_ab)
        while not rospy.is_shutdown():
            x = v_ab_qr_frame[0,3]
            y = v_ab_qr_frame[1,3]
            z = v_ab_qr_frame[2,3]

            self.br.sendTransform((x, y, z),
                            (0.0, 0.0, 0.0, 1.0),
                            rospy.Time.now(),
                            "bolt1",
                            "qr_tr")
            # print("in set")
            self.rate.sleep()

#         T = [0.]
    def get_camera_to_qr_frame(self):
        while not rospy.is_shutdown():
            # print("in get ")
            try:
                # listener.waitForTransform('/camera_link', '/qr_1', rospy.Time.now(), rospy.Duration(4.0))
                (trans,rot) = self.listener.lookupTransform('qr_tr', '/camera_link' , rospy.Time(0))
                # print(trans)
                # print(rot)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                # print( " no tf")
                continue
            self.rate.sleep()
    def get_qr_to_base(self):
        while not rospy.is_shutdown():
            try:
                # listener.waitForTransform('/camera_link', '/qr_1', rospy.Time.now(), rospy.Duration(4.0))
                (trans,rot) = self.listener.lookupTransform('/base_link', '/qr_tr', rospy.Time(0))
                # print(trans)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                # print( " no tf")
                continue
            self.rate.sleep()
        
    def qr_img_callback(self,data):
        self.qr_bl_pos = data.qrpos[0:4]
        self.qr_tr_pos = data.qrpos[4:8]
        if self.count <300:
            # print(self.count)
            self.pl_array_width.append(data.qrpos[6])
            self.pl_array_height.append(data.qrpos[7])
            self.count +=1
        else:
            self.pl_width_mean = np.mean(self.pl_array_width)
            self.pl_height_mean = np.mean(self.pl_array_height)
            self.pl_ratio = self.qr_width / np.minimum(self.pl_height_mean, self.pl_width_mean)
            # print(self.ratio)
            self.bolt_cal()


    def qr_img_pos_subscriber(self):
		rospy.Subscriber("/qr_pos", qrmsg, self.qr_img_callback,queue_size =1, buff_size= 2**24)
if __name__ == '__main__':
    rospy.init_node('qr_listener')
    qr = qr_tftransform()
    # br = tf.TransformBroadcaster()
    # listener = tf.TransformListener()
    # rate = rospy.Rate(10.0)
    # while not rospy.is_shutdown():
    #     try:
    #         # listener.waitForTransform('/camera_link', '/qr_1', rospy.Time.now(), rospy.Duration(4.0))
    #         (trans,rot) = listener.lookupTransform('/camera_link', '/qr_1', rospy.Time(0))
    #         print(trans)
    #     except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    #         print( " no tf")
    #         continue
    #     rate.sleep()
