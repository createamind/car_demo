#coding: utf-8
import numpy as np

"""
Deep Deterministic Policy Gradient (DDPG), Reinforcement Learning.
DDPG is Actor Critic based algorithm.
"""

import tensorflow as tf


#####################  hyper parameters  ####################

MAX_EPISODES = 200
MAX_EP_STEPS = 200
LR_A = 0.001    # learning rate for actor
LR_C = 0.002    # learning rate for critic
GAMMA = 0.9     # reward discount
TAU = 0.01      # soft replacement
MEMORY_CAPACITY = 1000
BATCH_SIZE = 32
CAMERA_COUNT = 4
CAMERA_W = 320
CAMERA_H = 240
CAMERA_CHANNEL = 3 * 6

RENDER = False

A_DIM = 2       # steering, throttle
from api.api_demo import CarPrius

###############################  DDPG  ####################################

class Memory(object):
    def __init__(self, capacity):
        self._rng = np.random.RandomState()
        self.cameras = np.zeros((capacity, CAMERA_COUNT, CAMERA_H, CAMERA_W, CAMERA_CHANNEL), dtype=np.uint8)
        self.cameras_ = np.zeros((capacity, CAMERA_COUNT, CAMERA_H, CAMERA_W, CAMERA_CHANNEL), dtype=np.uint8)
        self.actions = np.zeros((capacity, A_DIM), dtype=np.float32)
        self.rewards = np.zeros(capacity, dtype=np.float32)
        self._index = 0

    def put(self, s, a, r, s_):
        index = self._index % MEMORY_CAPACITY
        c_f, c_l, c_r, c_b = s
        self.cameras[index, 0] = c_f
        self.cameras[index, 1] = c_l
        self.cameras[index, 2] = c_r
        self.cameras[index, 3] = c_b
        self.actions[index] = a
        self.rewards[index] = r
        c_f, c_l, c_r, c_b = s_
        self.cameras_[index, 0] = c_f
        self.cameras_[index, 1] = c_l
        self.cameras_[index, 2] = c_r
        self.cameras_[index, 3] = c_b
        self._index = index + 1
        # raise NotImplementedError

    def batch(self):
        indices = self._rng.choice(MEMORY_CAPACITY, size=BATCH_SIZE)
        return self.cameras[indices], self.actions[indices], self.rewards[indices], self.cameras_[indices]

class DDPG(object):
    def __init__(self):
        self.memory = Memory(MEMORY_CAPACITY)
        self.sess = tf.Session()
        self.a_replace_counter, self.c_replace_counter = 0, 0

        # self.a_dim, self.s_dim, self.a_bound = A_DIM, s_dim, a_bound,
        self.S = tf.placeholder(tf.uint8, [None, CAMERA_COUNT, CAMERA_H, CAMERA_W, CAMERA_CHANNEL], 's')
        self.S_ = tf.placeholder(tf.float32, [None, CAMERA_COUNT, CAMERA_H, CAMERA_W, CAMERA_CHANNEL], 's_')
        self.R = tf.placeholder(tf.float32, [None, 1], 'r')

        with tf.variable_scope('Actor'):
            self.a = self._build_a(self.S, scope='eval', trainable=True)
            a_ = self._build_a(self.S_, scope='target', trainable=False)
        with tf.variable_scope('Critic'):
            # assign self.a = a in memory when calculating q for td_error,
            # otherwise the self.a is from Actor when updating Actor
            q = self._build_c(self.S, self.a, scope='eval', trainable=True)
            q_ = self._build_c(self.S_, a_, scope='target', trainable=False)

        # networks parameters
        self.ae_params = tf.get_collection(tf.GraphKeys.GLOBAL_VARIABLES, scope='Actor/eval')
        self.at_params = tf.get_collection(tf.GraphKeys.GLOBAL_VARIABLES, scope='Actor/target')
        self.ce_params = tf.get_collection(tf.GraphKeys.GLOBAL_VARIABLES, scope='Critic/eval')
        self.ct_params = tf.get_collection(tf.GraphKeys.GLOBAL_VARIABLES, scope='Critic/target')

        # target net replacement
        self.soft_replace = [[tf.assign(ta, (1 - TAU) * ta + TAU * ea, name='test'), tf.assign(tc, (1 - TAU) * tc + TAU * ec)]
                             for ta, ea, tc, ec in zip(self.at_params, self.ae_params, self.ct_params, self.ce_params)]
        # for ta, ea, tc, ec in zip(self.at_params, self.ae_params, self.ct_params, self.ce_params):
        #     print('{} - {} \n {} - {} '.format(ta.name, ea.name, tc.name, ec.name))

        q_target = self.R + GAMMA * q_
        # in the feed_dic for the td_error, the self.a should change to actions in memory
        td_error = tf.losses.mean_squared_error(labels=q_target, predictions=q)
        self.ctrain = tf.train.AdamOptimizer(LR_C).minimize(td_error, var_list=self.ce_params)

        a_loss = - tf.reduce_mean(q)    # maximize the q
        opt = tf.train.AdamOptimizer(LR_A)
        act_grads = tf.gradients(q,self.a  )[0]
        print('a_net_grads',act_grads,self.a)
        a_net_grads=tf.gradients(self.a, self.ae_params ,-act_grads )
        
        grads = zip(a_net_grads, self.ae_params)

        self.atrain = opt.apply_gradients(grads)
        # self.atrain = opt.minimize(a_loss, var_list=self.ae_params)

        self.sess.run(tf.global_variables_initializer())

    def choose_action(self, s):
        return self.sess.run(self.a, {self.S: s[np.newaxis, :]})[0]

    def learn(self):
        # soft target replacement
        self.sess.run(self.soft_replace)

        bs, ba, br, bs_ = self.memory.batch()

        # indices = np.random.choice(MEMORY_CAPACITY, size=BATCH_SIZE)
        # bt = self.memory[indices, :]
        # bs = bt[:, :self.s_dim]
        # ba = bt[:, self.s_dim: self.s_dim + self.a_dim]
        # br = bt[:, -self.s_dim - 1: -self.s_dim]
        # bs_ = bt[:, -self.s_dim:]

        self.sess.run(self.atrain, {self.S: bs})
        self.sess.run(self.ctrain, {self.S: bs, self.a: ba, self.R: br, self.S_: bs_})

    def store_transition(self, s, a, r, s_):
        self.memory.put(s, a, r, s_)

    def _build_cnn(self, s):
        cameras = tf.unstack(tf.cast(s, tf.float32) / 255.0, axis=1)
        # print('===**=='*4,cameras)
        outputs = []
        import tensorflow.contrib.slim as slim
        for   i , image  in enumerate(cameras):
            with slim.arg_scope([slim.conv2d], activation_fn=tf.nn.relu):
                l = slim.conv2d(image, 32, 5, scope=str(i) + '/conv0')
                l = slim.max_pool2d(l, 2, scope=str(i) + '/pool0')
                l = slim.conv2d(l, 32, 5, scope=str(i) + '/conv1')
                l = slim.max_pool2d(l, 2, scope=str(i) + '/pool1')
                l = slim.conv2d(l, 64, 4, scope=str(i) + '/conv2')
                l = slim.max_pool2d(l, 2, scope=str(i) + '/pool2')
                l = slim.conv2d(l, 64, 3, scope=str(i) + '/conv3')
            outputs.append(l)
        return tf.concat(outputs, axis=-1)

    def _build_a(self, s, scope, trainable):
        with tf.variable_scope(scope):
            # net = tf.layers.dense(s, 30, activation=tf.nn.relu, name='l1', trainable=trainable)
            net = self._build_cnn(s)
            net = tf.contrib.layers.flatten(net)
            # a = tf.layers.dense(net, self.a_dim, activation=tf.nn.tanh, name='a', trainable=trainable)
            steering = tf.layers.dense(net, 1, activation=tf.nn.tanh, name='steering', trainable=trainable)
            throttle = tf.layers.dense(net, 1, activation=tf.nn.sigmoid, name='throttle', trainable=trainable)
            act=tf.concat([steering * 0.5, throttle], axis=-1, name='scaled_a')
            # return tf.multiply(a, self.a_bound, name='scaled_a')
            print('act',act)
            return act

    def _build_c(self, s, a, scope, trainable):
        with tf.variable_scope(scope):
            net = self._build_cnn(s)
            net = tf.contrib.layers.flatten(net)
            net_shape = net.shape.as_list() #tf.shape(net)
            
            n_l1 = 64
            print(net_shape[1], n_l1)
            w1_s = tf.get_variable('w1_s', [net_shape[1], n_l1], trainable=trainable)
            w1_a = tf.get_variable('w1_a', [A_DIM, n_l1], trainable=trainable)
            b1 = tf.get_variable('b1', [1, n_l1], trainable=trainable)
            net = tf.nn.relu( tf.matmul(net, w1_s) +tf.matmul(a, w1_a) + b1)
            # net =  tf.nn.relu( tf.matmul(net,w1_s ),name='test')
            # grad = tf.gradients()
            return tf.layers.dense(net, 1, trainable=trainable)  # Q(s,a)

###############################  training  ####################################

import rospy
rospy.init_node('prius_api')
env = CarPrius(display=RENDER)

ddpg = DDPG()

var_steering = 0.5  # control exploration
var_throttle = 0.5
for i in range(MAX_EPISODES):
    s = env.reset()
    # s = np.concatenate(s[:4])
    s = np.stack(s[:4],axis=0)
    ep_reward = 0
    for j in range(MAX_EP_STEPS):
        # Add exploration noise
        a = ddpg.choose_action(s)
        a[0] = np.clip(np.random.normal(a[0], var_steering), -0.5, 0.5)
        a[1] = np.clip(np.random.normal(a[1], var_throttle), 0., 1.)
        # a = np.clip(np.random.normal(a, var), -2, 2)    # add randomness to action selection for exploration
        s_, r, done, info = env.step(a)
        s_ = np.stack(s_[:4],axis=0)

        ddpg.store_transition(s, a, r / 10, s_)

        if ddpg.memory._index > MEMORY_CAPACITY:
            var_steering *= .9995    # decay the action randomness
            var_throttle *= .9995    # decay the action randomness
            ddpg.learn()

        s = s_
        ep_reward += r
        if j == MAX_EP_STEPS-1:
            print('Episode:', i, ' Reward: %i' % int(ep_reward), 'Explore: %.2f/%.2f' % (var_steering, var_throttle),)
            break