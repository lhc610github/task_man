#!/usr/bin/env python
import rospy
import scipy.io as sio
import numpy as np 
from geometry_msgs.msg import PoseStamped
from demo_test.srv import teleop_ctrl

bias_map = np.array([-1.5,-2.5,0])

def check_in_Ab():
    global quads_pos,Ab_constrain
    for i in range(quads_pos.shape[1]):
        for j in range(Ab_constrain['A'][0,0].shape[0]):
            #print 'Ax:',Ab_constrain['A'][0,0][j,:].dot(quads_pos[:,i])
            #print 'b :',Ab_constrain['b'][0,0][j,0]
            if (Ab_constrain['A'][0,0][j,:].dot(quads_pos[:,i])) > (Ab_constrain['b'][0,0][j,0] + 0.1):
                return False
    return True

def UAV_pos_cb(msg, cb_args):
    global quads_pos,bias_map
    quads_pos[0,cb_args-1] = msg.pose.position.x - bias_map[0,0]
    quads_pos[1,cb_args-1] = msg.pose.position.y - bias_map[1,0]
    quads_pos[2,cb_args-1] = -msg.pose.position.z - bias_map[2,0]


if __name__ == '__main__':
    global Ab_constrain,quads_pos,bias_map
    data = sio.loadmat("/home/lhc/work/demo_ws/src/task_man/script/test_5uavs_forreal.mat")
    quads_pos_d_arry = data['rc_quad_pos_d']
    Ab_constrain_arry = data['rc_Ab_constarin']
    sizeof_quads_pos_d = quads_pos_d_arry.shape
    quads_pos = np.zeros([3,5],dtype=float)

    rospy.init_node('test',anonymous=True)
    rospy.Subscriber('/mavros1/mocap/pose', PoseStamped, callback= UAV_pos_cb, callback_args=1)
    rospy.Subscriber('/mavros2/mocap/pose', PoseStamped, callback= UAV_pos_cb, callback_args=2)
    rospy.Subscriber('/mavros3/mocap/pose', PoseStamped, callback= UAV_pos_cb, callback_args=3)
    rospy.Subscriber('/mavros4/mocap/pose', PoseStamped, callback= UAV_pos_cb, callback_args=4)
    rospy.Subscriber('/mavros5/mocap/pose', PoseStamped, callback= UAV_pos_cb, callback_args=5)

    srv_vector = []
    srv_vector.append(rospy.ServiceProxy('/teleop_ctrl_service1', teleop_ctrl))
    srv_vector.append(rospy.ServiceProxy('/teleop_ctrl_service2', teleop_ctrl))
    srv_vector.append(rospy.ServiceProxy('/teleop_ctrl_service3', teleop_ctrl))
    srv_vector.append(rospy.ServiceProxy('/teleop_ctrl_service4', teleop_ctrl))
    srv_vector.append(rospy.ServiceProxy('/teleop_ctrl_service5', teleop_ctrl))

    for i in range(sizeof_quads_pos_d[1]): # i iter
        print "stage ",i
        quads_pos_d = quads_pos_d_arry[0,i]
        num_quads = quads_pos_d.shape[1]
        if i != (sizeof_quads_pos_d[1]-1):
            Ab_constrain = Ab_constrain_arry[0,i]
        else:
            Ab_constrain = Ab_constrain_arry[0,i-1]
        rate = rospy.Rate(10);
        while not rospy.is_shutdown():
            print quads_pos
            for j in range(num_quads): # no.j UAV
                print "send pos_d"
                print quads_pos_d[:,j]+bias_map[:,0]
                teleop_ctrlRequest temp_teleop_req
                temp_teleop_req.teleop_ctrl_mask = temp_teleop_req.MASK_HOVER_POS
                temp_teleop_req.hover_pos_x = quads_pos_d[0,j] + bias_map[0,0]
                temp_teleop_req.hover_pos_y = quads_pos_d[1,j] + bias_map[1,0]
                temp_teleop_req.hover_pos_z = -quads_pos_d[2,j] + bias_map[2,0]
                temp_teleop_req.hover_pos_yaw = -1.57
                srv_vector[j].call(temp_teleop_req)
                #quads_pos[:,j] = quads_pos_d[:,j]
            if check_in_Ab():
                break
            rate.sleep()
        

        