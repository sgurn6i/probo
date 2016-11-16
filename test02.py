#!/usr/bin/env python2
#coding=UTF-8
# test 01
# 2016-11-11 12:13:20 Sgurn6i

import probopy
import collections

# [パラメータ]
# key 呼称
leg_keys = ('lf', 'rf', 'lb', 'rb')
leg_joint_keys = ('hip', 'thigh', 'shin')
joint_parameter_keys = ('offset', 'mag', 'pos_min', 'pos_max', 'servo_ch')
Joint_parameter = collections.namedtuple('Joint_parameter', joint_parameter_keys)
# pj: 各ジョイントのパラメータ。
# { leg_name: { part_in_leg: joint_parameter, ... }, ... }
# joint_parameter = (offset, mag, pos_min, pos_max, servo_ch)
pj1 = {'lf': { 'hip':   (   0.0,  1.0, -90.0,  90.0,  0),
               'thigh': (   0.0,  1.0, -90.0,  90.0,  1),
               'shin':  (   0.0,  1.0, -90.0,  90.0,  2) },
}
jp_lf_hip1 = Joint_parameter(*pj1['lf']['hip'])
print jp_lf_hip1

for k0, v0 in pj1.items():
    print k0, ":"
    for k, v in v0.items():
        print "   ", k0 + "_" + k, v

class Magj:
    """Magnified Joint.
    mj = Magj(Joint& joint, offset, mag, pos_min, pos_max)
    self.target(mpos)
    joint.pos = (mpos + offset) * mag
    としてprobopy.Jointクラスに渡される。
    pos_min <= pos <= pos_max
    に制限してから計算して渡す。
    """
    def __init__(self, joint, offset=0, mag=1.0,
                 pos_min=-90.0, pos_max=90.0):
        if (not isinstance(joint, probopy.Joint)):
            raise TypeError , ("joint type was " + str(type(joint))
                               + ", not probopy.Joint")
        assert pos_min <= pos_max, ("pos_min=" + str(pos_min)
                                    + " should be LE pos_max=" + str(pos_max))
        assert mag != 0, ("mag should not be 0")
        self._joint = joint
        self._offset = offset
        self._mag = mag
        self._pos_min = pos_min
        self._pos_max = pos_max
    def __str__(self):
        """returns joint name, servo ch, self attributes"""
        servo = self._joint.get_pwmservo()
        if servo:
            servo_str = "servo#" + str(servo.get_ch())
        else:
            servo_str = "(no pwmservo)"
        return ("%s of %s %s %s, offset %f, mag %f, min %f, max %f"
                % (self.__class__.__name__, self._joint.__class__.__name__,
                   self._joint.get_name(), servo_str,
                   self._offset, self._mag, self._pos_min, self._pos_max))
    def target(self, pos):
        pos = max(pos, self._pos_min)
        pos = min(pos, self._pos_max)
        jpos = (pos + self._offset) * self._mag
        self._joint.target(jpos)
    def get_curr_pos(self):
        jpos = self._joint.get_curr_pos()
        mpos = (jpos / self._mag) - self._offset
        return mpos
    def get_name(self):
        return self._joint.get_name()
    def get_joint(self):
        return self._joint
    def get_mag(self):
        return self._mag;
    def get_offset(self):
        return self._offset
    def get_pos_min(self):
        return self._pos_min
    def get_pos_max(self):
        return self._pos_max

class Leg:
    """足。hip,thigh,shinからなる一本の足。
    leg_joint_keys をキーとする Magj のリスト mj_list を受け取る。
    
    """
    def __init__(self, mj_list, foot_sense=None, name=""):
        self._name = name
        self._mjs = {};
        # Todo: mj_list item数チェック
        for kk, mj in mj_list.items():
            assert (kk in leg_joint_keys), ("unknown mj_list key '%s'" % kk)
            if (not isinstance(mj, Magj)):
                raise TypeError , ("mj type was" + str(type(mj)) + "not Magj")
            self._mjs[kk] = mj;
    def __str__(self):
        names = ""
        first = True
        for key in leg_joint_keys:
            mj = self._mjs[key]
            if not first:
                names += ", "
            first = False
            names += mj.get_name()
        return ("%s %s: with %s"
                % (self.__class__.__name__, self._name, names))
    def get_mj(self, key):
        return self._mjs[key]
    def str_positions(self):
        """hip, thigh, shin の mpos を表す文字列を返す """
        str1 = ""
        first = True
        for key in leg_joint_keys:
            mj = self._mjs[key]
            if not first:
                str1 += " "
            first = False
            str1 += str(mj.get_curr_pos())
        return (str1)
    def target(self, pos_hip, pos_thigh, pos_shin):
        """hip, thigh, shin各Magjのtarget posを設定する。 """
        self._mjs['hip'].target(pos_hip)
        self._mjs['thigh'].target(pos_thigh)
        self._mjs['shin'].target(pos_shin)

# 四本足 rat with legs クラス
class Ratl:
    """ 通常4本の Leg からなる rat のクラス。
    pj = {'lf' : leg_parameter, # left front
          'rf' : leg_parameter, # right front
          'lb' : leg_parameter, # left back 
          'rb' : leg_parameter} # right back
    leg_parameter = { 'hip'   : joint_parameter,
                      'thigh' : joint_parameter,
                      'shin'  : joint_parameter,
                      ['foot_sw' : foot_sw_parameter] }
    joint_parameter = (offset, mag, pos_min, pos_max, servo_ch)
    """
    def __init__(self, pj, name=""):
        pass

# body から controller, joint 作って、Legを生成する。
body1 = probopy.Body("body1")
print body1.get_name(), type(body1)
ct1 = body1.create_controller("ct1")
print ct1.get_name(), type(ct1), isinstance(ct1, probopy.Controller)
tick = 50.0
body1.set_tick(tick)
mjt_hip0 = Magj(ct1.create_joint("jt_hip0"), mag = -1.0)
mjt_thigh0 = Magj(ct1.create_joint("jt_thigh0"), mag = 1.0)
mjt_shin0 = Magj(ct1.create_joint("jt_shin0"), mag = 1.0)
llmj = {'hip':mjt_hip0, 'thigh':mjt_thigh0, 'shin':mjt_shin0}
leg0 = Leg(llmj, name="leg0")
leg0.target(10, 20, 30)
body1.do_em_in(200.0)
print "leg0 positions:", leg0.str_positions()

# joint に servoを付けてみる。
pca1 = probopy.Pca9685()
rc = pca1.init("/dev/i2c-1", 0x40, 60.0)
pca_exists = (rc == probopy.EA1_OK)
if pca_exists:
    ct1.attach_pwmc(pca1)
    servo0 = probopy.Pwmservo(0, probopy.PWM_SV_RS304MD)



