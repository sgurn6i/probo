#!/usr/bin/env python2
#coding=UTF-8
# バランスジャイロ。地面の傾きに対してバランスを取る。
# 2017-01-13 14:52:53 Sgurn6i

import sys
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
# step width constants(mm)
STZ = 15
#STZ = 10
# leg offsets
leg_offsets = {'lf' : Vector(0, 0.0, 0),
               'rf' : Vector(0, 0.0, 0),
               'lb' : Vector(0.0, 0.0, 0),
               'rb' : Vector(0.0, 0.0, 0), }

# trim limits
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

# set targets
def set_dvec_targets(dvecs):
    u"""
     set ik targets for legs with delta vectors
    """
    for leg_key in dvecs:
        vec = vecs_neutral[leg_key] + dvecs[leg_key] + leg_offsets[leg_key]
        rc = rat1.get_legs()[leg_key].set_ik_target(vec)
        assert rc >= 0

# get roll pitch yaw radians/degs
def get_rpy_rads():
    rc,x,y,z,w = rat1.get_gyro_curr_q()
    r1 = PyKDL.Rotation.Quaternion(x,y,z,w)
    roll, pitch, yaw = r1.GetRPY()
    return roll, pitch, yaw
def get_rpy_degs():
    roll, pitch, yaw = get_rpy_rads()
    return ratl.get_deg(roll), ratl.get_deg(pitch), ratl.get_deg(yaw)
# delta z for legs to keep horizontal
def get_dz_legs():
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
        
# Routine starts
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

# param
tm_gain = 0.3 # trim / angle gain
dx_pp = 0.3 # dp dr limit per position
dx_c = 1.0  # dp dr limit constant

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
        dz_legs = get_dz_legs()
        r,p,y = get_rpy_degs()
        # set dvec
        for key in ratl.LEG_KEYS:
            dvecs[key] = limited_tvec(0.0, 0.0, dz_legs[key])
        set_dvec_targets(dvecs)
        rat1.get_body().do_em_in(dt1)
        if stroke == 0:
            sys.stdout.write("st %2d rpy %5.1f %5.1f %5.1f" % (stroke,r,p,y))
            sys.stdout.write(" dz_f_b %5.3f" % (dz_legs['lf'] - dz_legs['lb']))
            sys.stdout.write(" dz_l_r %5.3f" % (dz_legs['lf'] - dz_legs['rf']))
            print ""

        
    
