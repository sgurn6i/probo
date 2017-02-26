#!/usr/bin/env python2
#coding=UTF-8
# 歩く IK版 2

import ratl
from PyKDL import Vector

# params
stroke_amt = 8 # 一周期のストローク数
dt1 = 100  # 遷移時間(ms)
# step width constants(mm)
STZ = 30
STY = 8
STX = 10
STX0 = 0
STX1 = STX * 1
STX2 = STX * 2
STX3 = STX * 3
STX4 = STX * 4
# leg offsets
LEG_OFFSETS = {'lf' : Vector(0, 0.0, -10),
               'rf' : Vector(0, 0.0, -10),
               'lb' : Vector(0, 0.0, -10),
               'rb' : Vector(0, 0.0, -10), }

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
    print "n ", key, vecs_neutral[key]
    assert rc >= 0

# delta position vectors
def set_dvec_targets(dvecs):
    u"""
     set ik targets for legs with delta vectors
    """
    for leg_key in dvecs:
        vec = vecs_neutral[leg_key] + dvecs[leg_key] + LEG_OFFSETS[leg_key]
        rc = rat1.get_legs()[leg_key].set_ik_target(vec)
        assert rc >= 0

dvecs0 = {}        
dvecs0["lf"] = Vector(0, 0, 0)
dvecs0["rf"] = Vector(0, 0, 0)
dvecs0["lb"] = Vector(0, 0, 0)
dvecs0["rb"] = Vector(0, 0, 0)
set_dvec_targets(dvecs0)
rat1.get_body().do_em_in(dt1)

# prompt
aa = raw_input('press enter > ')

# reset time
rat1.get_body().reset_time()

# walk
for cyc1 in range(0, 8):
    print "cycle", cyc1
    for stroke in range(0, stroke_amt):
        dvecs = {}
        if stroke == 0:
            dvecs["lf"] = Vector( STX0, +STY, 0)
            dvecs["rf"] = Vector(-STX2, +STY, 0)
            dvecs["lb"] = Vector( STX3, +STY, STZ)
            dvecs["rb"] = Vector( STX2, +STY, 0)
        elif stroke == 1:
            dvecs["lf"] = Vector(-STX1, 0, 0)
            dvecs["rf"] = Vector(-STX3, 0, 0)
            dvecs["lb"] = Vector( STX3, 0, 0)
            dvecs["rb"] = Vector( STX1, 0, 0)
        elif stroke == 2:
            dvecs["lf"] = Vector(-STX2,-STY, 0)
            dvecs["rf"] = Vector( STX3,-STY, STZ)
            dvecs["lb"] = Vector( STX2,-STY, 0)
            dvecs["rb"] = Vector( STX0,-STY, 0)
        elif stroke == 3:
            dvecs["lf"] = Vector(-STX3, 0, 0)
            dvecs["rf"] = Vector( STX3, 0, 0)
            dvecs["lb"] = Vector( STX1, 0, 0)
            dvecs["rb"] = Vector(-STX1, 0, 0)
        elif stroke == 4:
            dvecs["lf"] = Vector( STX3, +STY, STZ)
            dvecs["rf"] = Vector( STX2, +STY, 0)
            dvecs["lb"] = Vector( STX0, +STY, 0)
            dvecs["rb"] = Vector(-STX2, +STY, 0)
        elif stroke == 5:
            dvecs["lf"] = Vector( STX3, 0, 0)
            dvecs["rf"] = Vector( STX1, 0, 0)
            dvecs["lb"] = Vector(-STX1, 0, 0)
            dvecs["rb"] = Vector(-STX3, 0, 0)
        elif stroke == 6:
            dvecs["lf"] = Vector( STX2,-STY, 0)
            dvecs["rf"] = Vector( STX0,-STY, 0)
            dvecs["lb"] = Vector(-STX2,-STY, 0)
            dvecs["rb"] = Vector( STX3,-STY, STZ)
        elif stroke == 7:
            dvecs["lf"] = Vector( STX1, 0, 0)
            dvecs["rf"] = Vector(-STX1, 0, 0)
            dvecs["lb"] = Vector(-STX3, 0, 0)
            dvecs["rb"] = Vector( STX3, 0, 0)
        else:
            dvecs["lf"] = Vector( 0, 0, 0)
            dvecs["rf"] = Vector( 0, 0, 0)
            dvecs["lb"] = Vector( 0, 0, 0)
            dvecs["rb"] = Vector( 0, 0, 0)
        set_dvec_targets(dvecs)
        rat1.get_body().do_em_in(dt1)
        #for leg_key in ratl.LEG_KEYS:
        #    print stroke, leg_key, rat1.get_legs()[leg_key].get_fk_vec()

        
        
        
    
