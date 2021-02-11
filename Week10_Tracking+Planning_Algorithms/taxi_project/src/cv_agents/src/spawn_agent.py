#!/usr/bin/env python3
#-*- coding: utf-8 -*-

import rospy
import numpy as np
import math
import tf

import rospkg
import sys

from scipy.interpolate import interp1d

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion
from object_msgs.msg import Object

import pickle
import argparse

from stanley import stanley_control
from optimal_trajectory_Frenet import *

rospack = rospkg.RosPack()
path = rospack.get_path("map_server")

rn_id = dict()

rn_id[5] = {
    'left': [18, 2, 11, 6, 13, 8, 15, 10, 26, 0]  # ego route
}

def pi_2_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def interpolate_waypoints(wx, wy, space=0.5):
    _s = 0
    s = [0]
    for i in range(1, len(wx)):
        prev_x = wx[i - 1]
        prev_y = wy[i - 1]
        x = wx[i]
        y = wy[i]

        dx = x - prev_x
        dy = y - prev_y

        _s = np.hypot(dx, dy)
        s.append(s[-1] + _s)

    fx = interp1d(s, wx)
    fy = interp1d(s, wy)
    ss = np.linspace(0, s[-1], num=int(s[-1] / space) + 1, endpoint=True)

    dxds = np.gradient(fx(ss), ss, edge_order=1)
    dyds = np.gradient(fy(ss), ss, edge_order=1)
    wyaw = np.arctan2(dyds, dxds)

    return {
        "x": fx(ss),
        "y": fy(ss),
        "yaw": wyaw,
        "s": ss
    }


class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, dt=0.1, WB=2.6):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))
        self.dt = dt
        self.WB = WB

    def update(self, a, delta):
        dt = self.dt
        WB = self.WB

        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.v / WB * math.tan(delta) * dt
        self.yaw = pi_2_pi(self.yaw)
        self.v += a * dt
        self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))

    def calc_distance(self, point_x, point_y):
        dx = self.rear_x - point_x
        dy = self.rear_y - point_y
        return math.hypot(dx, dy)


def get_ros_msg(x, y, yaw, v, id):
    quat = tf.transformations.quaternion_from_euler(0, 0, yaw)

    m = Marker()
    m.header.frame_id = "map"
    m.header.stamp = rospy.Time.now()
    m.id = id
    m.type = m.CUBE

    m.pose.position.x = x + 1.3 * math.cos(yaw)
    m.pose.position.y = y + 1.3 * math.sin(yaw)
    m.pose.position.z = 0.75
    m.pose.orientation = Quaternion(*quat)

    m.scale.x = 4.475
    m.scale.y = 1.850
    m.scale.z = 1.645

    m.color.r = 93 / 255.0
    m.color.g = 122 / 255.0
    m.color.b = 177 / 255.0
    m.color.a = 0.97

    o = Object()
    o.header.frame_id = "map"
    o.header.stamp = rospy.Time.now()
    o.id = id
    o.classification = o.CLASSIFICATION_CAR
    o.x = x
    o.y = y
    o.yaw = yaw
    o.v = v
    o.L = m.scale.x
    o.W = m.scale.y

    return {
        "object_msg": o,
        "marker_msg": m,
        "quaternion": quat
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Spawn a CV agent')

    parser.add_argument("--id", "-i", type=int, help="agent id", default=1)
    parser.add_argument("--route", "-r", type=int,
                        help="start index in road network. select in [1, 3, 5, 10]", default=5)
    parser.add_argument("--dir", "-d", type=str, default="left", help="direction to go: [left, straight, right]")
    args = parser.parse_args()

    rospy.init_node("three_cv_agents_node_" + str(args.id))

    id = args.id
    tf_broadcaster = tf.TransformBroadcaster()
    marker_pub = rospy.Publisher("/objects/marker/car_" + str(id), Marker, queue_size=1)
    object_pub = rospy.Publisher("/objects/car_" + str(id), Object, queue_size=1)

    start_node_id = args.route
    route_id_list = [start_node_id] + rn_id[start_node_id][args.dir]

    ind = 100

    with open(path + "/src/route.pkl", "rb") as f:
        nodes = pickle.load(f, encoding='latin1')

    wx = []
    wy = []
    wyaw = []
    for _id in route_id_list:
        wx.append(nodes[_id]["x"][1:])
        wy.append(nodes[_id]["y"][1:])
        wyaw.append(nodes[_id]["yaw"][1:])
    wx = np.concatenate(wx)
    wy = np.concatenate(wy)
    wyaw = np.concatenate(wyaw)

    waypoints = {"x": wx, "y": wy, "yaw": wyaw}

    target_speed = 20.0 / 3.6
    state = State(x=waypoints["x"][ind], y=waypoints["y"][ind], yaw=waypoints["yaw"][ind], v=0.1, dt=0.1)

    # get maps
    maps = np.zeros(wx.shape)
    for i in range(len(wx)-1):
        x = wx[i]
        y = wy[i]
        sd = get_frenet(x, y, wx, wy)
        maps[i] = sd[0]

    # initial condition, convert global coordinate to frenet coordinate
    s, d = get_frenet(state.x, state.y, wx, wy);
    x, y, yaw_road = get_cartesian(s, d, wx, wy, maps)
    yawi = state.yaw - yaw_road

    # s 방향 초기조건
    si = s
    si_d = state.v*np.cos(yawi)
    si_dd = 0
    sf_d = target_speed
    sf_dd = 0

    # d 방향 초기조건
    di = d
    di_d = state.v*np.sin(yawi)
    di_dd = 0
    df_d = 0
    df_dd = 0

    opt_d = di

    obs = np.array([
        [158, 0.667],
        [238, -0.7],
    ])

    r = rospy.Rate(100)
    while not rospy.is_shutdown():
        path, opt_ind = frenet_optimal_planning(si, si_d, si_dd, sf_d, sf_dd, di, di_d, di_dd, df_d, df_dd, obs, wx, wy, maps, opt_d)

        si = path[0].s[1]
        si_d = path[0].s_d[1]
        si_dd = path[0].s_dd[1]
        di = path[0].d[1]
        di_d = path[0].d_d[1]
        di_dd = path[0].d_dd[1]
        # consistency cost를 위해 update
        opt_d = path[opt_ind].d[-1]
#        opt_d = di

        # generate acceleration ai, and steering di
        speed_error = target_speed - state.v
        ai = 0.5 * speed_error

        if len(path) == 14:
            delta = stanley_control(state.x, state.y, state.yaw, state.v, wx, wy, wyaw)
        else:
            delta = stanley_control(state.x, state.y, state.yaw, state.v, path[opt_ind].x, path[opt_ind].y, path[opt_ind].yaw)

#        delta = stanley_control(state.x, state.y, state.yaw, state.v, path[opt_ind].x, path[opt_ind].y, path[opt_ind].yaw)

        # update state with acc, delta
        state.update(ai, delta)

        print('s-dir: {}, d-dir: {}'.format(si, opt_d))
        print('state.x: {}, state.y: {}'.format(state.x, state.y), '\n')

        # vehicle state --> topic msg
        msg = get_ros_msg(state.x, state.y, state.yaw, state.v, id=id)
        #msg = get_ros_msg(path[opt_ind].x[opt_ind], path[opt_ind].y[opt_ind], path[opt_ind].yaw[opt_ind], state.v, id=id)

        # send tf
        tf_broadcaster.sendTransform(
            (state.x, state.y, 1.5),
            msg["quaternion"],
            rospy.Time.now(),
            "/car_" + str(id), "map"
        )

        # publish vehicle state in ros msg
        object_pub.publish(msg["object_msg"])

        r.sleep()
