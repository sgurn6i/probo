#!/usr/bin/env python2
#coding=UTF-8
# バランスジャイロ。地面の傾きに対してバランスを取る。
# 2017-01-13 14:52:53 Sgurn6i

import sys
import copy
import math
import ratl
import PyKDL
from PyKDL import Vector

# Initial Leg Position
#lp_neutral = {'hip': 2.0, 'thigh':  -35.0, 'shin': 75.0}
#lp_neutral = {'hip': 0.0, 'thigh':  -32.0, 'shin': 83.0}


# params
stroke_amt = 8 # 一周期のストローク数
#dt1 = 100  # 遷移時間(ms)
dt1 = 50  # 遷移時間(ms)
KP = 0.1     # P制御Kp
# leg offsets
leg_offsets = {'lf' : Vector(0, 0.0, 0.0),
               'rf' : Vector(0, 0.0, 0.0),
               'lb' : Vector(0.0, 0.0, 0.0),
               'rb' : Vector(0.0, 0.0, 0.0), }
# trim limits(mm)
tx_limit = 15
ty_limit = 15
tz_limit = 30

# abs limit values
def abs_limit(d, limit):
    if d > limit:
        d = limit
    elif d < -limit:
        d = -limit
    return d

# limited trim vec
def limited_tvec(x, y, z):
    return Vector(abs_limit(x, tx_limit),
                  abs_limit(y, ty_limit),
                  abs_limit(z, tz_limit))
def limited_tvec_vec(vec):
    return limited_tvec(vec[0], vec[1], vec[2])

# rat
rat1 = ratl.Ratl(name="rat1")
rat1.set_kdl_segs()

rc = rat1.prepare_pca()
print "prepare_pca rc =", rc

# start pos
rat1.target_neutral()
rat1.get_body().do_em_in(0)
rat1.get_body().do_em_in(dt1)
print "ready to start"

# neutral position vector
vecs_neutral = {}
for key in ratl.LEG_KEYS:
    vecs_neutral[key], rc = rat1.get_legs()[key].get_fk_vec()
    print " ", key, vecs_neutral[key]
    assert rc >= 0

# rat1 functions
def get_foot_vecs_with_dvecs(dvecs):
    u"""get foot position vectors with delta vectors from neutral positions.
    """
    vecs = {}
    for leg_key in dvecs:
        vec = vecs_neutral[leg_key] + dvecs[leg_key] + leg_offsets[leg_key]
        vecs[leg_key] = vec
    return vecs
def set_ik_targets(vecs):
    u"""set ik targets for legs with foot position vectors
    """
    for leg_key in vecs:
        rc = rat1.get_legs()[leg_key].set_ik_target(vecs[leg_key])
        assert rc >= 0
def set_dvec_targets(dvecs):
    u"""
     set ik targets for legs with delta vectors from neutral positions.
    """
    set_ik_targets(get_foot_vecs_with_dvecs(dvecs))

def get_rpy_rads():
    u"""get roll pitch yaw radians"""
    rc,x,y,z,w = rat1.get_gyro_curr_q()
    r1 = PyKDL.Rotation.Quaternion(x,y,z,w)
    roll, pitch, yaw = r1.GetRPY()
    return roll, pitch, yaw
def get_rpy_degs():
    u"""get roll pitch yaw degs"""
    roll, pitch, yaw = get_rpy_rads()
    return ratl.get_deg(roll), ratl.get_deg(pitch), ratl.get_deg(yaw)
def get_gyro_dz_legs():
    u""" optimal delta z from current positions for each leg.
    to keep horizontal, calculated from gyro data. 
    """
    roll, pitch, yaw = get_rpy_rads()
    tan_pitch = - math.tan(pitch)
    tan_roll = math.tan(roll)
    dz_legs = {}
    for key in ratl.LEG_KEYS:
        foot_vec, rc = rat1.get_legs()[key].get_fk_vec()
        assert rc >= 0
        dz_pitch = tan_pitch * foot_vec[0] # with X position
        dz_roll = tan_roll * foot_vec[1] # with Y position
        dz_legs[key] = dz_pitch + dz_roll
    return dz_legs
def xy_shift(vecs, height = 10, pr = False):
    u""" 揺れ低減のために dvecs を XY方向にシフトする。
    入力
    vecs: ratl.LEG_KEYSをキーとするVectors.
    height: 地面から不動点/回転運動の中心までの高さmm。
            マイナスだと、地面より下になる。
    pr: Trueなら値プリント。
    出力
    s_vecs: vecsに対応する計算結果。
    """
    s_vecs = {}
    for key in vecs:
        s_vecs[key] = copy.deepcopy(vecs[key]) # shifted vecs
    # Y方向(横)
    len_y_f = vecs['lf'][1] - vecs['rf'][1] # front 側 Y 方向 foot間隔
    len_y_b = vecs['lb'][1] - vecs['rb'][1] # back 側 Y 方向 foot間隔
    len_y = (len_y_f + len_y_b) * 0.5 # Y 方向 foot間隔平均
    diff_z_f = vecs['lf'][2] - vecs['rf'][2] # front 側 Y 方向 foot 高さの差
    diff_z_b = vecs['lb'][2] - vecs['rb'][2]
    theta_yz_f = math.atan2(diff_z_f, len_y_f)
    theta_yz_b = math.atan2(diff_z_b, len_y_b)
    theta_yz = (theta_yz_f + theta_yz_b) * 0.5
    shift_y =  height * math.sin(theta_yz) # foot の y方向 shift量
    # X方向(縦)
    len_x_l = vecs['lf'][0] - vecs['lb'][0] # left
    len_x_r = vecs['rf'][0] - vecs['rb'][0] # right
    len_x = (len_x_l + len_x_r) * 0.5
    diff_z_l = vecs['lf'][2] - vecs['lb'][2]
    diff_z_r = vecs['rf'][2] - vecs['rb'][2]
    theta_xz_l = math.atan2(diff_z_l, len_x_l)
    theta_xz_r = math.atan2(diff_z_r, len_x_r)
    theta_xz = (theta_xz_l + theta_xz_r) * 0.5
    shift_x = height * math.sin(theta_xz)
    # result
    if pr:
        print "theta_yz: ", ratl.get_deg(theta_yz), " shift_y: ", shift_y
        print "theta_xz: ", ratl.get_deg(theta_xz), " shift_x: ", shift_x
    for key in vecs:
        s_vecs[key][1] = vecs_neutral[key][1] + shift_y
        s_vecs[key][0] = vecs_neutral[key][0] + shift_x
    return s_vecs
        
# Routine starts
# dvec: 各足先のbody座標系内目標位置ベクトルのneutral位置からの差分。
# dvec0: dvec初期値。
# 
dvecs0 = {}        
dvecs0["lf"] = Vector(0, 0, 0)
dvecs0["rf"] = Vector(0, 0, 0)
dvecs0["lb"] = Vector(0, 0, 0)
dvecs0["rb"] = Vector(0, 0, 0)
set_dvec_targets(dvecs0)
rat1.get_body().do_em_in(dt1)
#print "debug lf shin", rat1.get_legs()["lf"].get_mj("shin")

# prompt
aa = raw_input('press enter > ')

# gyro
rc = rat1.prepare_mpu6050()
print "prepare_mpu6050 rc =", rc

# reset time
rat1.get_body().set_tick(40.0)
rat1.get_body().reset_time()

# walk
r0,p0,y0 = get_rpy_degs()
p_trim = 0.0
p_trim_limit = tz_limit
r_trim = 0.0
r_trim_limit = tz_limit
dvecs = dvecs0
for cyc1 in range(0, 200):
    print "cycle", cyc1
    for stroke in range(0, stroke_amt):
        dz_legs = get_gyro_dz_legs()
        r,p,y = get_rpy_degs()
        # set dvec
        for key in ratl.LEG_KEYS:
            dvecs[key] = limited_tvec_vec(dvecs[key] + KP * Vector(0.0, 0.0, dz_legs[key]))
        vecs = get_foot_vecs_with_dvecs(dvecs)
        s_vecs = xy_shift(vecs = vecs, height = -100, pr = (stroke == 0))
        vecs = s_vecs
        set_ik_targets(vecs)
        rat1.get_body().do_em_in(dt1)
        if stroke == 0:
            sys.stdout.write("st %2d rpy %5.1f %5.1f %5.1f" % (stroke,r,p,y))
            sys.stdout.write(" dvec f_b %5.3f" % (dvecs['lf'][2] - dvecs['lb'][2]))
            sys.stdout.write(" dvec l_r %5.3f" % (dvecs['lf'][2] - dvecs['rf'][2]))
            print ""
            print "vec lf ", vecs['lf']
    
