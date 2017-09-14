#!/usr/bin/env python

# Copyright 2017 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rospy
from prius_msgs.msg import Control
from sensor_msgs.msg import Joy, Image
from std_msgs.msg import Header
from gazebo_msgs.srv import GetModelState, SetModelState
from geometry_msgs.msg import Point, Quaternion ,Vector3, Vector3Stamped
import numpy as np
import cv2
import threading
import multiprocessing as mp
from cv_bridge import CvBridge, CvBridgeError
import tf
from  gazebo_msgs.msg import ContactsState
from tf import transformations

##test
from tf import TransformListener


QUEUE_MAX_SIZE = 6    # 30 / 5
IMG_SIZE = (320, 240)

class CameraInfo(object):
    def __init__(self, car, name):
        self.car = car
        self.name = name
        self.sub = None
        self.imageQueue = mp.Queue(QUEUE_MAX_SIZE)
        self.imageSeq = 0
    
class ContactInfo(object):
    def __init__(self):
        self.sub = None
        self.src_frame= "/map"
        self.target_frame= "/base_link"
        self.ContactQueue = mp.Queue(QUEUE_MAX_SIZE)



class CarPrius(object):
    def __init__(self, name = "prius", display = False):
        self._name = name
        self._flagEnd = False
        self._flagRender = display

        self.target_frame = "/base_link"
        self.src_frame="/map"

        self._cvBridge = CvBridge()

        from functools import partial
        from collections import OrderedDict
        self._cameraInfos = OrderedDict()     # type: dict[str, CameraInfo]

        for camera in ['front_camera', 'left_camera', 'right_camera', 'back_camera']:
            self._cameraInfos[camera] = si = CameraInfo(self, camera)
            si.sub = rospy.Subscriber('prius/{}/image_raw'.format(camera), Image, partial(self.callbackCamera, si))

        self.pub = rospy.Publisher('prius', Control, queue_size=1)

        ##add contact sensor
        self._contactInfos = sc = ContactInfo()
        sc.sub = rospy.Subscriber('/chassis_bumper',ContactsState,partial(self.callbackContact, sc))

        ## add tf Transformer
        self._tflistener=tf.TransformListener()
        self._tf=tf.TransformerROS()
        self.tf = TransformListener()


        try:
            rospy.wait_for_service('/gazebo/set_model_state')
            self._set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            rospy.wait_for_service('gazebo/get_model_state')
            self._get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        except rospy.ServiceException as e:
            print(e)
            raise e

        self._lastPublishTime = rospy.get_rostime()
        self._lastPublished = None
        self._seqPublished = 0

    def reset(self):
        self._resetPos()
        return self._observation()

    def _resetPos(self):
        # for a complicate request, use basic msg type to construct
        req = SetModelState._request_class()
        req.model_state.model_name = self._name
        req.model_state.pose.position = Point(3, 12, 0.5)
        req.model_state.pose.orientation = Quaternion(0, 0, 1, 0)
        req.model_state.twist.linear = Vector3(0, 0, 0)
        req.model_state.twist.angular = Vector3(0, 0, 0)
        res = self._set_model_state(req)
        # print(res)

    def _resetWorld(self):
        pass


    def _close(self):
        self._flagEnd = True
        for k, v in self._cameraInfos.items():
            v.imageQueue.close()
        pass

    def _observation(self):
        images = []
        for name, ci in self._cameraInfos.items():
            _images = []
            for i in range(QUEUE_MAX_SIZE):
                img = ci.imageQueue.get()
                if self._flagEnd: return
                # print(img.shape, img.dtype)
                _images.append(img)
            image = np.concatenate(_images, -1)
            images.append(image)
        
        _contact_pos = [] 
        ## add for contact 
        for i in range(1): # 5hz
            contact_pos = self._contactInfos.ContactQueue.get()
            print('contact  sensor state is  {}'.format(contact_pos) )
            _contact_pos.append(contact_pos)

        if self._flagEnd: return
        if self._flagRender:
            w = IMG_SIZE[0]
            h = IMG_SIZE[1]
            img = np.zeros((h*3, w*2, 3), dtype=np.uint8)
            img[:h, w/2:w*3/2] = images[0][:, :, -3:]
            img[h:2*h, :w] = images[1][:, :, -3:]
            img[h:2*h, w:] = images[2][:, :, -3:]
            img[2*h:, w/2:w*3/2] = images[3][:, :, -3:]
            print('image = {}'.format(img.shape))
            cv2.imshow('image', img)
            cv2.waitKey(1)
        # for '/gazebo/get_model_state' service ,your shuold specify two args ,
        #  [model name, relative entity] ,relative entity could be empty ('')
        # req=GetModelState._request_class()
        # req.model_name=self.model_name
        # resp=get_model_state(req)

        position, orientation, linear_vec, angular_vec = self.get_model_state()

        model_state = np.concatenate([position, orientation, linear_vec, angular_vec])
        

        # print('prius is at {}'.format(model_state))
        return images + [model_state]



    def step(self, action):
        assert(isinstance(action, Control))
        if action.header is None:
            action.header = Header()
            action.header.seq = self._seqPublished
            self._seqPublished += 1
            action.header.stamp = rospy.Time.now()

        self.pub.publish(action)
        self._lastPublishTime = rospy.get_rostime()
        self._lastPublished = action
        ob = self._observation()
        isOver = False
        reward = 0.0
        return ob, reward, isOver

    def sampleAction(self):
        ret = Control()
        ret.throttle = 3
        ret.brake = 0.0
        ret.steer = np.random.rand() * 0.5 -0.25
        return ret

    def callbackCamera(self, cameraInfo, image):
        try:
            assert(image.width == IMG_SIZE[0] and image.height == IMG_SIZE[1])
            cv_image = self._cvBridge.imgmsg_to_cv2(image, "bgr8")
            cameraInfo.imageQueue.put(np.asarray(cv_image))
            rospy.logdebug("sensor %s received image %dx%d", cameraInfo.name, image.width, image.height)
        except CvBridgeError as e:
            print(e)

    ##add contact callback
    def callbackContact(self,contactInfo, contact):
        #if there is no collision ,contact states will be empty []
        if  len(contact.states) == 0: 
            contactInfo.ContactQueue.put('Safe')
            return
        

        #contact positon in in inertial frame (global frame /map )
        #transform it to target frame "/base_link" as car 
        #show contact msg type : rostopic type /chassis_bumper | rosmsg show
        #show Vector3Stamped type :rosmsg show Vector3Stamped
        #first : transform Vector3 to Vector3Stamped with time_stamp and frame_id
        v3s=Vector3Stamped()
        v3s.header=contact.header
        v3s.header.frame_id=contactInfo.src_frame
        v3s.vector=contact.states[0].contact_positions[0]
        

        contact_position = self.rigid_transform(v3s)
        # c_pos = contact_position.
        contactInfo.ContactQueue.put(contact_position)

        rospy.logdebug('there is  collision at {} '.format(contact_position))
    
    def rigid_transform(self,v3s=None):
        # there is bug with in aoto frame transformation between /map and /base_link

        #so  apply rigid_transform by hand
        # not so much farmilar with matrx ,but result look like ok 


        #get car local position quaternion
        position, quaternion, linear_vec, angular_vec = self.get_model_state()

        quaternion=[ -i for i in quaternion]
        #caculate relative_position
        relative_position = np.array([v3s.vector.x - position[0] , v3s.vector.y - position[1] , v3s.vector.z - position[2] , 1 ])
        
        #caculate rotation_matrix ,aplly rotation to  relative_position
        # get a 4 dim array [x,y,z,1]
        r_position=np.dot(relative_position, transformations.quaternion_matrix(quaternion))
        contact_position=r_position[:4]
 
        return contact_position

    def get_model_state(self):
        # move get state expression  into a function

        resp=self._get_model_state(self._name, '')
        position=[getattr(resp.pose.position, k) for k in ['x','y','z']]
        orientation = [getattr(resp.pose.orientation, k) for k in ['x', 'y', 'z', 'w']]
        linear_vec = [getattr(resp.twist.linear, k) for k in ['x', 'y', 'z']]
        angular_vec = [getattr(resp.twist.angular, k) for k in ['x', 'y', 'z']]


        
        
        return [position, orientation, linear_vec, angular_vec]









if __name__ == '__main__':
    rospy.init_node('prius_api')
    t = CarPrius(display=False)
    t.reset()
    try:
        while True:
            act = t.sampleAction()
            ob, reward, isOver = t.step(act)
            if isOver:
                t.reset()

    except KeyboardInterrupt:
        t._close()