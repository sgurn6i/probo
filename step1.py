#!/usr/bin/env python2
#coding=UTF-8
# 足踏み

import ratl

# Leg Positions
lp0  = {'hip': 0.0, 'thigh':  -9.7, 'shin': 40.0} # 足前
lp0u = {'hip': 0.0, 'thigh': -23.8, 'shin': 91.0}
lp1  = {'hip': 0.0, 'thigh': -16.8, 'shin': 43.8}
lp1u = {'hip': 0.0, 'thigh': -31.0, 'shin': 93.7}
lp2  = {'hip': 0.0, 'thigh': -22.7, 'shin': 45.0}
lp2u = {'hip': 0.0, 'thigh': -37.7, 'shin': 94.6}
lp3  = {'hip': 0.0, 'thigh': -27.4, 'shin': 43.8}
lp3u = {'hip': 0.0, 'thigh': -43.3, 'shin': 93.7}
lp4  = {'hip': 0.0, 'thigh': -30.7, 'shin': 40.0} # 足後ろ
lp4u = {'hip': 0.0, 'thigh': -48.0, 'shin': 91.1}
# params
stroke_amt = 8 # 一周期のストローク数
dt1 = 200  # 遷移時間(ms)

# rat
rat1 = ratl.Ratl(name="rat1")
rc = rat1.prepare_pca()
print "prepare_pca rc =", rc
rat1.set_kdl_segs()

def set_targets(lps):
    for leg_key in ratl.LEG_KEYS:
        rat1.get_legs()[leg_key].target(lps[leg_key])

# start pos
lps = {}
lps["lf"] = lp2
lps["rb"] = lp2
lps["lb"] = lp2
lps["rf"] = lp2
set_targets(lps)
rat1.get_body().do_em_in(10)
rat1.get_body().do_em_in(dt1 - 10)
print "ready to start"

# prompt
aa = raw_input('press enter > ')

# walk
for cyc1 in range(0, 15):
    print "cycle", cyc1
    for stroke in range(0, stroke_amt):
        if stroke == 0:
            lps["lf"] = lp4
            lps["rb"] = lp4
            lps["lb"] = lp4u
            lps["rf"] = lp4
        elif stroke == 2:
            lps["lf"] = lp0u
            lps["rb"] = lp0
            lps["lb"] = lp0
            lps["rf"] = lp0
        elif stroke == 4:
            lps["lf"] = lp4
            lps["rb"] = lp4u
            lps["lb"] = lp4
            lps["rf"] = lp4
        elif stroke == 6:
            lps["lf"] = lp0
            lps["rb"] = lp0
            lps["lb"] = lp0
            lps["rf"] = lp0u
        else:
            lps["lf"] = lp2
            lps["rb"] = lp2
            lps["lb"] = lp2
            lps["rf"] = lp2
        set_targets(lps)
        rat1.get_body().do_em_in(dt1)
        for leg_key in ratl.LEG_KEYS:
            print stroke, leg_key, rat1.get_legs()[leg_key].get_fk_vec()
        
        
        
    
