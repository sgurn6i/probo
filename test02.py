#!/usr/bin/env python2
#coding=UTF-8
# test 02
# 2016-11-11 12:13:20 Sgurn6i

import probopy
import ratl

# Leg のposition集
leg_pos_zero = {'hip':0.0, 'thigh':0.0, 'shin':0.0}
leg_pos_up = {'hip':0.0, 'thigh':70.0, 'shin':-145.0}
leg_pos_front = {'hip':0.0, 'thigh':-45.0, 'shin':45.0}
leg_pos_back = {'hip':0.0, 'thigh':45.0, 'shin':-45.0}
leg_pos_center = {'hip':0.0, 'thigh':22.5, 'shin':-45.0}

# rat
rat1 = ratl.create_rat1("rat1")
print rat1
rc = rat1.prepare_pca(device="/dev/i2c-1")
print "prepare_pca rc =", rc
# ゼロ位置
for leg_key in ratl.leg_keys:
    rat1.get_leg(leg_key).target(leg_pos_zero)
rat1.get_body().do_em_in(10.0)
# 屈伸
dt1 = 700.0
for leg_key in ratl.leg_keys:
    rat1.get_leg(leg_key).target(leg_pos_center)
rat1.get_body().do_em_in(dt1)
for leg_key in ratl.leg_keys:
    rat1.get_leg(leg_key).target(leg_pos_zero)
rat1.get_body().do_em_in(dt1)
for leg_key in ratl.leg_keys:
    rat1.get_leg(leg_key).target(leg_pos_center)
rat1.get_body().do_em_in(dt1)
# 怯む
for leg_key in ratl.leg_keys:
    rat1.get_leg(leg_key).target(leg_pos_front)
rat1.get_body().do_em_in(dt1)
rat1.get_body().do_em_in(3000)
for leg_key in ratl.leg_keys:
    rat1.get_leg(leg_key).target(leg_pos_center)
rat1.get_body().do_em_in(dt1)
# last state
for leg_key in ratl.leg_keys:
    rat1.get_leg(leg_key).target(leg_pos_zero)
rat1.get_body().do_em_in(500.0)

# end
print "end"


