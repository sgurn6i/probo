#!/usr/bin/env python2
#coding=UTF-8
# バランスジャイロ。地面の傾きに対してバランスを取る。
# 2017-01-13 14:52:53 Sgurn6i

import sys
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
               'lb' : Vector(-25.0, 0.0, 0),
               'rb' : Vector(-25.0, 0.0, 0), }

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

# delta position vectors
def set_dvec_targets(dvecs):
    u"""
     set ik targets for legs with delta vectors
    """
    for leg_key in dvecs:
        vec = vecs_neutral[leg_key] + dvecs[leg_key] + leg_offsets[leg_key]
        rc = rat1.get_legs()[leg_key].set_ik_target(vec)
        assert rc >= 0

# get roll pitch yaw degs
def get_rpy_degs():
    rc,x,y,z,w = rat1.get_gyro_curr_q()
    r1 = PyKDL.Rotation.Quaternion(x,y,z,w)
    roll, pitch, yaw = r1.GetRPY()
    return ratl.get_deg(roll), ratl.get_deg(pitch), ratl.get_deg(yaw)

        
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
        r,p,y = get_rpy_degs()
        # pitch
        dp_trim1 = (p - p0) * (- tm_gain)
        dp_limit = abs(abs(p_trim) - p_trim_limit) * dx_pp + dx_c
        dp_trim = abs_limit(dp_trim1, dp_limit)
        p_trim = abs_limit(p_trim + dp_trim, p_trim_limit)
        # roll
        dr_trim1 = (r - r0) * (- tm_gain)
        dr_limit = abs(abs(r_trim) - r_trim_limit) * dx_pp + dx_c
        dr_trim = abs_limit(dr_trim1, dr_limit)
        r_trim = abs_limit(r_trim + dr_trim, r_trim_limit)
        # set dvec
        dvec_x = - p_trim * 0.5
        dvec_y = r_trim * 0.5
        dvecs["lf"] = limited_tvec(dvec_x, dvec_y, p_trim - r_trim)
        dvecs["rf"] = limited_tvec(dvec_x, dvec_y, p_trim + r_trim)
        dvecs["lb"] = limited_tvec(dvec_x, dvec_y, -p_trim - r_trim)
        dvecs["rb"] = limited_tvec(dvec_x, dvec_y, -p_trim + r_trim)
        set_dvec_targets(dvecs)
        rat1.get_body().do_em_in(dt1)
        if stroke == 0:
            sys.stdout.write("st %2d rpy %5.1f %5.1f %5.1f" % (stroke,r,p,y))
            sys.stdout.write(" p_trim %5.3f dp %5.3f" % (p_trim, dp_trim))
            sys.stdout.write(" r_trim %5.3f dr %5.3f" % (r_trim, dr_trim))
            print ""

        
    
