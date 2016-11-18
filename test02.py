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
joint_parameter_keys = ('offset', 'mag', 'pos_min', 'pos_max')
Joint_parameter = collections.namedtuple('Joint_parameter', joint_parameter_keys)
# pj: 各ジョイントのパラメータ。
# { leg_key: { leg_joint_key: joint_parameter, ... }, ... }
# joint_parameter = (offset, mag, pos_min, pos_max)
pj1 = {'lf': { 'hip':   ( -45.0,  1.0, -30.0,  120.0),
               'thigh': (   0.0, -1.0, -90.0,  45.0),
               'shin':  (   0.0, -1.0, -80.0,  90.0),
               'foot_sw': ( 0 ) },
       'rf': { 'hip':   ( -45.0, -1.0, -30.0,  120.0),
               'thigh': (   0.0,  1.0, -90.0,  45.0),
               'shin':  (   0.0,  1.0, -80.0,  90.0),
               'foot_sw': ( 0 ) },
       'lb': { 'hip':   ( -45.0,  1.0, -30.0,  120.0),
               'thigh': (   0.0, -1.0, -90.0,  45.0),
               'shin':  (   0.0, -1.0, -80.0,  90.0),
               'foot_sw': ( 0 ) },
       'rb': { 'hip':   ( -45.0, -1.0, -30.0,  120.0),
               'thigh': (   0.0,  1.0, -90.0,  45.0),
               'shin':  (   0.0,  1.0, -80.0,  90.0),
               'foot_sw': ( 0 ) },}
# 各ジョイントの pwmservo パラメータ
# (servo_ch, probo.pwmservo_type_enum)
pj_pwms1 = {'lf': { 'hip':   ( 0, probopy.PWM_SV_RS304MD),
                    'thigh': ( 1, probopy.PWM_SV_RS304MD),
                    'shin':  ( 2, probopy.PWM_SV_RS304MD) },
            'rf': { 'hip':   ( 8, probopy.PWM_SV_RS304MD),
                    'thigh': ( 9, probopy.PWM_SV_RS304MD),
                    'shin':  (10, probopy.PWM_SV_RS304MD) },
            'lb': { 'hip':   ( 4, probopy.PWM_SV_RS304MD),
                    'thigh': ( 5, probopy.PWM_SV_RS304MD),
                    'shin':  ( 6, probopy.PWM_SV_RS304MD) },
            'rb': { 'hip':   (12, probopy.PWM_SV_RS304MD),
                    'thigh': (13, probopy.PWM_SV_RS304MD),
                    'shin':  (14, probopy.PWM_SV_RS304MD) },}

# Leg のposition集
leg_pos_zero = {'hip':0.0, 'thigh':0.0, 'shin':0.0}

jp_lf_hip1 = Joint_parameter(*pj1['lf']['hip'])
print jp_lf_hip1

for k0, v0 in pj1.items():
    print k0, ":"
    for k, v in v0.items():
        print "   ", k0 + "_" + k, v

class Magj:
    u"""Magnified Joint.
    mj = Magj(Joint& joint, offset, mag, pos_min, pos_max)
    self.target(mpos)
    joint.pos = (mpos + offset) * mag
    としてprobopy.Jointクラスに渡される。
    pos_min <= mpos <= pos_max
    に制限してから計算して渡す。
    """
    def __init__(self, joint, offset=0, mag=1.0, pos_min=-90.0, pos_max=90.0):
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
        u"""returns joint name, servo ch, self attributes"""
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

def create_magj(ct, parameters, name="joint"):
    u"""probopy.Controller ct から probopy.Joint をcreateし、
    joint_parameter_keys の順に並んだparametersリストを元に
    Magjを作って、それを返す。
    """
    jt = ct.create_joint(name)
    magj = Magj(jt, *parameters)
    return magj

class Leg:
    u"""足。hip,thigh,shinからなる一本の足。
    leg_joint_keys をキーとする Magj のdict mj_dict を受け取る。
    
    """
    def __init__(self, mj_dict, foot_sense=None, name=""):
        self._name = name
        self._mjs = {};
        # Todo: mj_dict item数チェック
        for kk, mj in mj_dict.items():
            assert (kk in leg_joint_keys), ("unknown mj_dict key '%s'" % kk)
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
        u"""hip, thigh, shin の mpos を表す文字列を返す """
        str1 = ""
        first = True
        for key in leg_joint_keys:
            mj = self._mjs[key]
            if not first:
                str1 += " "
            first = False
            str1 += str(mj.get_curr_pos())
        return (str1)
    def target(self, pos_dict):
        u"""各Magjのtarget posを設定する。
        値は、leg_joint_keys のメンバをkeyとするpos_dictによって与えられる。
        pos_dict = {'hip':10.0, 'thigh':20.0, 'shin':30.0} 等。
        一部だけ設定するのも可。
        """
        for key in pos_dict:
            self._mjs[key].target(pos_dict[key])

# 四本足 rat with legs クラス
class Ratl:
    u""" 通常4本の Leg からなる rat のクラス。
    ratl = Ratl(pj, name)
    pj = {'lf' : leg_parameter, # left front
          'rf' : leg_parameter, # right front
          'lb' : leg_parameter, # left back 
          'rb' : leg_parameter} # right back
    leg_parameter = { 'hip'   : joint_parameter,
                      'thigh' : joint_parameter,
                      'shin'  : joint_parameter,
                      ['foot_sw' : foot_sw_parameter] }
    joint_parameter = (offset, mag, pos_min, pos_max)
    """
    def __init__(self, pj, name="rat"):
        self._name = name
        self._body = probopy.Body(name)
        self._ctj = self._body.create_controller(name + "_ctj")
        self._pwmservos = {} # GC阻止の為にpwmservo貯めとく
        self._legs = {}
        for leg_key in pj:
            assert leg_key in leg_keys, ("unknown leg_key: " + str(leg_key))
            mjs = {}
            for joint_key in pj[leg_key]:
                if joint_key in leg_joint_keys:
                    jt_name = "%s_jt_%s_%s" % (name, leg_key, joint_key)
                    mjs[joint_key] = create_magj(self._ctj, pj[leg_key][joint_key], jt_name)
            self._legs[leg_key] = Leg(mj_dict=mjs, name=("%s_leg_%s" % (name, leg_key)))
    def __str__(self):
        str1 = "%s %s:\n" % (self.__class__.__name__, self._name)
        for leg_key in self._legs:
            str1 += "  %s\n" % (str(self._legs[leg_key]))
        return str1
    def get_body(self):
        return self._body
    def get_ctj(self):
        """get probo.Controller for joints."""
        return self._ctj
    def get_leg(self,leg_key):
        """ get probo.Leg.
        leg_key in leg_keys.
        """
        return self._legs[leg_key]
    def set_pwm(self, pwmc, pj_pwms):
        """ set pwm controller and pwm servo interfaces.
        pwmc = probopy.Pwmc init済を期待する。
        pj_pwms = 各ジョイントの pwmservo パラメータ
                  {leg_key: {leg_joint_key: (servo_ch, probo.pwmservo_type_enum), ...}, ... }
        返り値: probopy.EA1_OK, or probopy.ea1_status_enum。
        """
        rc = self._ctj.attach_pwmc(pwmc)
        if rc == probopy.EA1_OK:
            for leg_key in pj_pwms:
                self._pwmservos[leg_key] = {}
                for joint_key in pj_pwms[leg_key]:
                     joint = self._legs[leg_key].get_mj(joint_key).get_joint()
                     pwmservo = probopy.Pwmservo(*pj_pwms[leg_key][joint_key])
                     self._pwmservos[leg_key][joint_key] = pwmservo
                     rc = joint.attach_pwmservo(pwmservo)
                     if rc != probopy.EA1_OK: return rc
        return rc
    
# rat
rat1 = Ratl(pj1, "rat1")
print rat1
for leg_key in leg_keys:
    rat1.get_leg(leg_key).target(leg_pos_zero)
pca1 = probopy.Pca9685()
rc = pca1.init("/dev/i2c-1", 0x40, 60.0)
if rc == probopy.EA1_OK:
    rc = rat1.set_pwm(pca1, pj_pwms1)
print "set_pwm rc =", rc
rat1.get_body().do_em_in(10.0)
rat1.get_body().do_em_in(1200.0)

# end
print "end"


