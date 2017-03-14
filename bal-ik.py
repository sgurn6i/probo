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
KP = 0.36  # P制御Kp
KI = 0.05  # PID制御Ki
KD = 0.36  # PID制御Kd
T_RANGE = 4 # 離散時間記憶範囲
SHIFT_HEIGHT = 70 # xy_shift時の不動点高さ(mm)
#TAN_LIMIT = 1.0 # 上限 tanθ
MV_LIMIT = 0.5 # manipulated variable mv 絶対値上限
FLOATING_DDZ = 2.0 # trim_floating時dz変化量(mm)
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
def get_gyro_te():
    u"""
    現状gyro pitch roll値のmath.tan()値を返す。
    (set point = 0) - (process variable tan(theta))
    返り値:
    tangent errors ['pitch', 'roll']
    """
    te = {}
    roll, pitch, yaw = get_rpy_rads()
    te['pitch'] = - math.tan(pitch)
    te['roll'] = - math.tan(roll)
    return te
def update_te_mn(te_mn, te, pr = False):
    u"""
    te_mn を更新する。
    引数:
    te_mn['pitch', 'roll'][range(T_RANGE)] : tan角error履歴入出力
    te['pitch', 'roll'] : 現在のtan角error 入力
    """
    for key in te_mn:
        for ix in reversed(xrange(T_RANGE)):
            if ix <= 0:
                te_mn[key][0] = te[key]
            else:
                te_mn[key][ix] = te_mn[key][ix - 1]
            if pr and (ix == 0):
                print "te[%5s][%d] = %.7f" % (key, ix, te_mn[key][ix])

def get_curr_mv(prev_mv, te_mn, pr = False):
    u"""
    te_mnから manipulated variable mv を求める。PID制御値。
    引数:
    prev_mv['pitch', 'roll'] : 一つ前のmv。
    te_mn['pitch', 'roll'][range(T_RANGE)] : tan角error履歴入力。
    pr: Trueなら値プリント。
    返り値:
    mv['pitch', 'roll']
    """
    mv = {}
    for key in prev_mv:
        te_n = te_mn[key]
        d_mv_p = KP * (te_n[0] - te_n[1])
        d_mv_i = KI * te_n[0]
        d_mv_d = KD * ((te_n[0] - te_n[1]) - (te_n[1] - te_n[2]))
        d_mv = d_mv_p + d_mv_i + d_mv_d
        mv[key] = abs_limit(prev_mv[key] + d_mv, MV_LIMIT)
        if pr:
            print ("%5s: dP %.7f dI %.7f dD %.7f MV %.5f"
                   % (key, d_mv_p, d_mv_i, d_mv_d, mv[key]))
    return mv

def get_dz_legs_from_mv(mv):
    u"""
    manipulated variable mv から各足のZ方向移動量 dz_legsを求める。
    引数:
    mv['pitch', 'roll'] : 傾き角度のtangentで現したmanipulated variable.
    返り値:
    dz_legs[ratl.LEG_KEYS] : Z方向移動量(mm).
    """
    dz_legs = {}
    for key in ratl.LEG_KEYS:
        foot_vec, rc = rat1.get_legs()[key].get_fk_vec()
        assert rc >= 0
        dz_pitch = mv['pitch'] * foot_vec[0] # with X position
        dz_roll = - mv['roll'] * foot_vec[1] # with Y position
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
def get_floating_feet_amt():
    u""" 浮き上がってる足の数を返す。"""
    floating_amt = 0;
    for key in ratl.LEG_KEYS:
        val = rat1.get_legs()[key].get_sw_val('foot_sw')
        if val == 1:
            floating_amt += 1
    return floating_amt
def reset_pid(mv, te_mn):
    u"""mv, te_mnをリセット"""
    for key in mv:
        mv[key] = 0
        for ix in range(T_RANGE):
            te_mn[key][ix] = 0.0
def reset_dz(dz_legs):
    u"""dz_legsをリセット"""
    for key in ratl.LEG_KEYS:
        dz_legs[key] = 0.0

def trim_floating_dvecs(dvecs, floating_amt):
    u"""浮いてる足がある場合はdvecsを相互に調整する"""
    if floating_amt > 0:
        dvec_z_sum = 0.0 # 足のdvec Z 成分値の合計。
        for key in dvecs:
            sw_val = rat1.get_legs()[key].get_sw_val('foot_sw')
            if sw_val == 0:
                dvec_z_sum += dvecs[key][2]
        # 全部浮いていれば真ん中方向に持っていく。
        # 全体的に足が下がっていれば、浮いていない方を上げる。
        # そうでなければ、浮いている方を下げる。
        for key in dvecs:
            sw_val = rat1.get_legs()[key].get_sw_val('foot_sw')
            if (floating_amt == len(ratl.LEG_KEYS)):
                if (dvecs[key][2] < - FLOATING_DDZ):
                    dvecs[key][2] += FLOATING_DDZ
                elif (dvecs[key][2] > FLOATING_DDZ):
                    dvecs[key][2] -= FLOATING_DDZ
            elif (sw_val == 0) and (dvec_z_sum < 0):
                dvecs[key][2] += FLOATING_DDZ
            elif (sw_val == 1) and (dvec_z_sum >= 0):
                dvecs[key][2] -= FLOATING_DDZ

# Routine starts
# dvec: 各足先のbody座標系内目標位置ベクトルのneutral位置からの差分。
# 
# dvec0: dvec初期値。
dvecs0  = {key:Vector(0, 0, 0) for key in ratl.LEG_KEYS}
set_dvec_targets(dvecs0)
rat1.get_body().do_em_in(dt1)
# GPIO switches
rat1.set_sw_gpio()

# prompt
aa = raw_input('press enter > ')

# gyro
rc = rat1.prepare_mpu6050()
print "prepare_mpu6050 rc =", rc

# reset time
rat1.get_body().set_tick(40.0)
rat1.get_body().reset_time()

# start loop
# te_mn[key][n] はkey方向における過去nステップ前のgyroで読んだ所の水平からのtangent角error。
te_mn = {key:[0.0 for ix in xrange(T_RANGE)] for key in ('pitch', 'roll')}
# manipulated variable。as 足の動きによる現状からのtan(theta)修正値。
mv = {key:0.0 for key in ('pitch', 'roll')}
dvecs = dvecs0
for cyc1 in xrange(0, 500):
    print "cycle", cyc1
    for stroke in xrange(0, stroke_amt):
        pr = (stroke == 3) # print flag
        te = get_gyro_te() # tangent errors ['pitch', 'roll']
        update_te_mn(te_mn, te, False)
        # te_mnから mv['pitch', 'roll'] を求める。PID制御値。
        mv = get_curr_mv(mv, te_mn, pr)
        # mv から dz_legs[ratl.LEG_KEYS]を求める。
        dz_legs = get_dz_legs_from_mv(mv)
        # 足の浮き上がりチェック。
        floating_amt = get_floating_feet_amt()
        # 2本以上浮いている時はpid中断する
        if floating_amt >= 2:
            reset_pid(mv, te_mn)
            reset_dz(dz_legs)
        # set dvec
        for key in ratl.LEG_KEYS:
            dvecs[key] = limited_tvec_vec(dvecs[key] + Vector(0.0, 0.0, dz_legs[key]))
        # 浮いてる足がある時は、dvecsを浮かない方向で調整する。
        if floating_amt >= 1:
            trim_floating_dvecs(dvecs, floating_amt)
        vecs = get_foot_vecs_with_dvecs(dvecs)
        s_vecs = xy_shift(vecs = vecs, height = SHIFT_HEIGHT, pr = False)
        vecs = s_vecs
        set_ik_targets(vecs)
        rat1.get_body().do_em_in(dt1)
        if pr:
            if floating_amt >= 1:
                print "floating %d feet" % (floating_amt)
            r,p,y = get_rpy_degs()
            sys.stdout.write("st %2d rpy %5.1f %5.1f %5.1f" % (stroke,r,p,y))
            sys.stdout.write(" dvec f_b %5.3f" % (dvecs['lf'][2] - dvecs['lb'][2]))
            sys.stdout.write(" dvec l_r %5.3f" % (dvecs['lf'][2] - dvecs['rf'][2]))
            print ""
            print "vec lf ", vecs['lf']
    
