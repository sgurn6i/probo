#!/usr/bin/env python2
#coding=UTF-8
# rat with legs ロボットモデルクラス
# 2016-11-19 15:29:29 Sgurin6i

import math
import copy
import collections
import PyKDL
import probopy

# [パラメータ]
"""
# key 呼称
"""
LEG_KEYS = ('lf', 'rf', 'lb', 'rb')
LEG_JOINT_KEYS = ('hip', 'thigh', 'shin')
LEG_SW_KEYS = ('foot_sw')
JOINT_PARAMETER_KEYS = ('offset', 'mag', 'pos_min', 'pos_max')
SW_PARAMETER_KEYS = ('on')
PWMSERVO_KEYS = ('ch', 'type')
SEG_KEYS = {'joint', 'tip'}

class Md(collections.Mapping):
    u"""Mapping data コンテナ。
    key=value で指定されたデータを要素とする辞書として働く。
    ただし、後から項目を追加したり、削除したりはできない。
    Attributes:
      0個、1個、または複数のキーワード付き引数。
    """
    def __init__(self, **kwargs):
        self._data = {}
        for key in kwargs:
            self._data[key] = kwargs[key]
    def __len__(self):
        return len(self._data)
    def __str__(self):
        return "Md " + str(self._data)
    def __repr__(self):
        return "Md " + str(self._data)
    def __getitem__(self, key):
        return self._data[key]
    def __iter__(self):
        return iter(self._data)
class MdJoint(Md):
    u"""Md の内、key が JOINT_PARAMETER_KEYS に含まれていなければならないもの。
    """
    def __init_(self, **kwargs):
        for key, val in kwargs.items():
            assert(key in JOINT_PARAMETER_KEYS)
        Md.__init__(self, **kwargs)
class MdSw(Md):
    u"""Md の内、key が SW_PARAMETER_KEYS に含まれていなければならないもの。
    """
    def __init_(self, **kwargs):
        for key, val in kwargs.items():
            assert(key in SW_PARAMETER_KEYS)
        Md.__init__(self, **kwargs)
class MdLegw(Md):
    u"""Md の内、key が LEG_JOINT_KEYS または LEG_SW_KEYS
    に含まれていなければならないもの。 """
    def __init__(self, **kwargs):
        for key, val in kwargs.items():
            assert((key in LEG_JOINT_KEYS) or (key in LEG_SW_KEYS))
        Md.__init__(self, **kwargs)
class MdLegwJoint(MdLegw):
    u"""MdLegwの内、key が LEG_JOINT_KEYS に含まれていたら、
    valueはMdJointのインスタンスに限定。
    key が LEG_SW_KEYS に含まれていたら、
    valueはMdSwのインスタンスに限定。
    それら以外のkeyは不可。
    """
    def __init_(self, **kwargs):
        for key, val in kwargs.items():
            if key in LEG_JOINT_KEYS:
                assert(isinstance(val, MdJoint))
            elif key in LEG_SW_KEYS:
                assert(isinstance(val, MdSw))
            else:
                raise KeyError , ("unknown key: " + str(key))
        MdLegw.__init__(self, **kwargs)
class MdRatw(Md):
    u"""Md の内、key が LEG_KEYS に含まれていなければならないもの。
    更に value が MdLegw のインスタンスでなければならない。
    """
    def __init__(self, **kwargs):
        for key, val in kwargs.items():
            assert(key in LEG_KEYS)
            assert(isinstance(val, MdLegw))
        Md.__init__(self, **kwargs)
class MdRatwJoint(Md):
    u"""MdRatw の内、value が MdLegwJoint のインスタンスでなければならない。
    """
    def __init__(self, **kwargs):
        for key, val in kwargs.items():
            assert(isinstance(val, MdLegwJoint))
        Md.__init__(self, **kwargs)
class MdLeg(Md):
    u"""Md の内、key が LEG_JOINT_KEYS に含まれていなければならないもの。 """
    def __init__(self, **kwargs):
        for key, val in kwargs.items():
            assert(key in LEG_JOINT_KEYS)
        Md.__init__(self, **kwargs)
class MdLegSeg(MdLeg):
    u"""MdLeg の内、value が Seg のインスタンスでなければならないもの。 """
    def __init__(self, **kwargs):
        for key, val in kwargs.items():
            assert(isinstance(val, Seg))
        MdLeg.__init__(self, **kwargs)
class MdRat(Md):
    u"""Md の内、key が LEG_KEYS に含まれていなければならないもの。
    更に value が MdLeg のインスタンスでなければならない。
    """
    def __init__(self, **kwargs):
        for key, val in kwargs.items():
            assert(key in LEG_KEYS)
            assert(isinstance(val, MdLeg))
        Md.__init__(self, **kwargs)
class MdRatSeg(MdRat):
    u"""MdRat の内、value が MdLegSeg のインスタンスでなければならない。
    """
    def __init__(self, **kwargs):
        for key, val in kwargs.items():
            assert(isinstance(val, MdLegSeg))
        MdRat.__init__(self, **kwargs)

# rat1 のデフォルト MdRatwJoint インスタンス。
# 足を下に伸ばしきった状態をposゼロとする。
# ゼロ方向を起点として本体の左側、前側に回る方向を正の方向とする。
# 右手系 Y軸が進行方向、X軸が右方向、Z軸が上方向。
# 軸に向かって反時計回りが正の回転方向
MD_RATW_JOINT1 = MdRatwJoint(
    lf = MdLegwJoint(hip   = MdJoint(offset=-45.0, mag= 1.0, pos_min= -30.0, pos_max=120.0),
                     thigh = MdJoint(offset= 25.0, mag= 1.0, pos_min= -70.0, pos_max= 65.0),
                     shin  = MdJoint(offset=-65.0, mag= 1.0, pos_min= -45.0, pos_max=145.0),
                     foot_sw = MdSw(on=0)),
    rf = MdLegwJoint(hip   = MdJoint(offset= 45.0, mag= 1.0, pos_min=-120.0, pos_max= 30.0),
                     thigh = MdJoint(offset= 25.0, mag=-1.0, pos_min= -70.0, pos_max= 65.0),
                     shin  = MdJoint(offset=-65.0, mag=-1.0, pos_min= -45.0, pos_max=145.0),
                     foot_sw = MdSw(on=0)),
    lb = MdLegwJoint(hip   = MdJoint(offset=-45.0, mag= 1.0, pos_min= -30.0, pos_max=120.0),
                     thigh = MdJoint(offset= 25.0, mag= 1.0, pos_min= -70.0, pos_max= 65.0),
                     shin  = MdJoint(offset=-65.0, mag= 1.0, pos_min= -45.0, pos_max=145.0),
                     foot_sw = MdSw(on=0)),
    rb = MdLegwJoint(hip   = MdJoint(offset= 45.0, mag= 1.0, pos_min=-120.0, pos_max= 30.0),
                     thigh = MdJoint(offset= 25.0, mag=-1.0, pos_min= -70.0, pos_max= 65.0),
                     shin  = MdJoint(offset=-65.0, mag=-1.0, pos_min= -45.0, pos_max=145.0),
                     foot_sw = MdSw(on=0)),
    )

# 各ジョイントの pwmservo パラメータ
# {'ch':<servo_ch>, 'type':<probopy.pwmservo_type_enum>}
pj_pwms1 = {'lf': { 'hip':   {'ch': 0, 'type': probopy.PWM_SV_RS304MD},
                    'thigh': {'ch': 1, 'type': probopy.PWM_SV_RS304MD},
                    'shin':  {'ch': 2, 'type': probopy.PWM_SV_RS304MD} },
            'rf': { 'hip':   {'ch': 8, 'type': probopy.PWM_SV_RS304MD},
                    'thigh': {'ch': 9, 'type': probopy.PWM_SV_RS304MD},
                    'shin':  {'ch':10, 'type': probopy.PWM_SV_RS304MD} },
            'lb': { 'hip':   {'ch': 4, 'type': probopy.PWM_SV_RS304MD},
                    'thigh': {'ch': 5, 'type': probopy.PWM_SV_RS304MD},
                    'shin':  {'ch': 6, 'type': probopy.PWM_SV_RS304MD} },
            'rb': { 'hip':   {'ch':12, 'type': probopy.PWM_SV_RS304MD},
                    'thigh': {'ch':13, 'type': probopy.PWM_SV_RS304MD},
                    'shin':  {'ch':14, 'type': probopy.PWM_SV_RS304MD} },}
# leg 各segmentの長さ(mm)
leg_seg_lens = { 'hip': 0.0, 'thigh': 58.0, 'shin': 59.0 }
# pseg: 各KDL segment パラメータ
# { leg_key: { leg_joint_key: { seg_key: (x,y,z), ... }, ... }, ... }
# SEG_KEYS:
#     'joint': 親につながるジョイントの軸方向ベクトルを表す配列
#     'tip':   jointから先端tipまでの相対位置ベクトルを表す配列(単位mm)
pseg1 = {'lf': {'hip':   {'joint':(0, 1, 0), 'tip':(0.0, 0.0, 0.0) },
                'thigh': {'joint':(1, 0, 0), 'tip':(0.0, 0.0, - leg_seg_lens['thigh']) },
                'shin':  {'joint':(1, 0, 0), 'tip':(0.0, 0.0, - leg_seg_lens['shin']) },  },
         'rf': {'hip':   {'joint':(0, 1, 0), 'tip':(0.0, 0.0, 0.0) },
                'thigh': {'joint':(1, 0, 0), 'tip':(0.0, 0.0, - leg_seg_lens['thigh']) },
                'shin':  {'joint':(1, 0, 0), 'tip':(0.0, 0.0, - leg_seg_lens['shin']) },  },
         'lb': {'hip':   {'joint':(0, 1, 0), 'tip':(0.0, 0.0, 0.0) },
                'thigh': {'joint':(1, 0, 0), 'tip':(0.0, 0.0, - leg_seg_lens['thigh']) },
                'shin':  {'joint':(1, 0, 0), 'tip':(0.0, 0.0, - leg_seg_lens['shin']) },  },
         'rb': {'hip':   {'joint':(0, 1, 0), 'tip':(0.0, 0.0, 0.0) },
                'thigh': {'joint':(1, 0, 0), 'tip':(0.0, 0.0, - leg_seg_lens['thigh']) },
                'shin':  {'joint':(1, 0, 0), 'tip':(0.0, 0.0, - leg_seg_lens['shin']) },  },}

# 算数
pi = math.pi
def get_deg(rad):
    return 180.0 * (rad / pi)
def get_rad(deg):
    return pi * (deg / 180.0)

class Magj:
    u"""Magnified Joint.
    Attributes:
      offset : magnified ジョイント位置(mpos)オフセット。
      mag :    pos magnitude.値0でないこと。
      pos_min: mpos 最小値。
      pos_max: mpos 最大値。
      joint:   probopy.Jointインスタンス。最初は指定せず、
               後で set_joint(joint) 関数で設定してもいい。
    mj = Magj(offset, mag, pos_min, pos_max, probopy.Joint& joint)
    target(mpos)メソッドを呼ぶと、
    joint.pos = (mpos + offset) * mag
    としてprobopy.Jointクラスに渡される。
    pos_min <= mpos <= pos_max
    に制限してから計算して渡す。
    """
    def __init__(self, offset=0, mag=1.0, pos_min=-90.0, pos_max=90.0, joint=None):
        assert pos_min <= pos_max, ("pos_min=" + str(pos_min)
                                    + " should be LE pos_max=" + str(pos_max))
        assert mag != 0, ("mag should not be 0")
        self.set_joint(joint)
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
    def set_joint(self, joint):
        u"""probopy.Jointインスタンスjointを組み入れる。"""
        if not joint:
            self._joint = None
        else:
            if (not isinstance(joint, probopy.Joint)):
                raise TypeError , ("joint type was " + str(type(joint))
                                   + ", not probopy.Joint")
            self._joint = joint
    def target(self, mpos):
        u"""mposからpos値を計算して probopy.Joint インスタンスのtargetメソッドを呼ぶ。 """
        pos = max(mpos, self._pos_min)
        pos = min(mpos, self._pos_max)
        jpos = (mpos + self._offset) * self._mag
        self._joint.target(jpos)
    def get_curr_pos(self):
        u"""現在ジョイント位置のmpos値を返す。"""
        jpos = self._joint.get_curr_pos()
        mpos = (jpos / self._mag) - self._offset
        return mpos
    def get_name(self):
        u"""probopy.Jointインスタンスの名前を返す。"""
        return self._joint.get_name()
    def get_joint(self):
        u"""probopy.Jointインスタンスを返す。"""
        return self._joipnt
    def get_mag(self):
        u"""mag 値を返す。"""
        return self._mag;
    def get_offset(self):
        u"""offset 値を返す。"""
        return self._offset
    def get_pos_min(self):
        u"""pos_min 値を返す。"""
        return self._pos_min
    def get_pos_max(self):
        u"""pos_max 値を返す。"""
        return self._pos_max

# def create_magj(ct, parameters, name="joint"):
#     u"""probopy.Controller ct から probopy.Joint をcreateし、
#     JOINT_PARAMETER_KEYS をキーとする parameters dict.を元に
#     Magjを作って、それを返す。
#     """
#     jt = ct.create_joint(name)
#     magj = Magj(joint=jt, **parameters)
#     return magj

class Leg:
    u"""足。hip, thigh, shin(, foot_sw)からなる一本の足。
    Attributes:
      ctj:           joint controller。probopy.Controllerインスタンス。
      md_legw_joint: joint parameter。MdLegwJointインスタンス。
      name:          このインスタンスの名前 str。
    """
    def __init__(self, ctj, md_legw_joint, name=""):
        assert(isinstance(md_legw_joint, MdLegwJoint))
        self._name = name
        self._mjs = {}
        self._ctj = ctj
        # Todo: mj_dict item数チェック
        for key in md_legw_joint:
            md = md_legw_joint[key]
            if isinstance(md, MdJoint):
                jt_name = "%s_jt_%s" % (name, key)
                jt = self._ctj.create_joint(jt_name)
                self._mjs[key] = Magj(joint=jt, **md)
    def __str__(self):
        names = ""
        first = True
        for key in LEG_JOINT_KEYS:
            mj = self._mjs[key]
            if not first:
                names += ", "
            first = False
            names += mj.get_name()
        return ("%s %s: with %s"
                % (self.__class__.__name__, self._name, names))
    def get_mjs(self):
        u"""取り込んだMagjの辞書を返す。
        LEG_JOINT_KEYS をキーとする。
        """
    def get_mj(self, key):
        u"""keyに対応するMagjインスタンスを返す。"""
        return self._mjs[key]
    def str_positions(self):
        u"""hip, thigh, shin の mpos を表す文字列を返す """
        str1 = ""
        first = True
        for key in LEG_JOINT_KEYS:
            mj = self._mjs[key]
            if not first:
                str1 += " "
            first = False
            str1 += str(mj.get_curr_pos())
        return (str1)
    def target(self, pos_dict):
        u"""各Magjのtarget posを設定する。
        値は、LEG_JOINT_KEYS のメンバをkeyとするpos_dictによって与えられる。
        pos_dict = {'hip':10.0, 'thigh':20.0, 'shin':30.0} 等。
        一部だけ設定するのも可。
        """
        for key in pos_dict:
            self._mjs[key].target(pos_dict[key])


# 四本足 rat with legs クラス
class Ratl:
    u""" 通常4本の Leg からなる rat のクラス。
    Attributes:
      md_ratw_joint: joint parameter。MdRatwJointインスタンス。
      name:          このインスタンスの名前 str。
    """
    def __init__(self, md_ratw_joint=MD_RATW_JOINT1, name="rat1"):
        assert(isinstance(md_ratw_joint, MdRatwJoint))
        self._name = name
        self._body = probopy.Body(name)
        self._ctj = self._body.create_controller(name + "_ctj")
        self._pwmservos = {} # GC阻止の為にpwmservo貯めとく
        self._legs = {}
        assert(isinstance(md_ratw_joint, MdRatwJoint))
        for leg_key in md_ratw_joint:
            self._legs[leg_key] = Leg(ctj = self._ctj,
                                      md_legw_joint = md_ratw_joint[leg_key],
                                      name=("%s_leg_%s" % (name, leg_key)))
    def __str__(self):
        str1 = "%s %s:\n" % (self.__class__.__name__, self._name)
        for leg_key in self._legs:
            str1 += "  %s\n" % (str(self._legs[leg_key]))
        return str1
    def get_body(self):
        return self._body
    def get_ctj(self):
        u"""get probo.Controller for joints."""
        return self._ctj
    def get_leg(self,leg_key):
        u""" get probo.Leg.
        leg_key in LEG_KEYS.
        """
        return self._legs[leg_key]
    def set_pwm(self, pwmc, pj_pwms=pj_pwms1):
        u""" set pwm controller and pwm servo interfaces.
        pwmc = probopy.Pwmc init済を期待する。
        pj_pwms = 各ジョイントの pwmservo パラメータ
                  {leg_key: {leg_joint_key: {pwmservo_key:<val>, ...}, ...}, ... }
        返り値: probopy.EA1_OK, or probopy.ea1_status_enum。
        """
        rc = self._ctj.attach_pwmc(pwmc)
        if rc == probopy.EA1_OK:
            for leg_key in pj_pwms:
                self._pwmservos[leg_key] = {}
                for joint_key in pj_pwms[leg_key]:
                    joint = self._legs[leg_key].get_mj(joint_key).get_joint()
                    # 引数省略に備える
                    dargs = copy.deepcopy(pj_pwms1[leg_key][joint_key])
                    for key in pj_pwms[leg_key][joint_key]:
                        dargs[key] = pj_pwms[leg_key][joint_key][key]
                    pwmservo = probopy.Pwmservo(dargs['ch'], dargs['type'])
                    self._pwmservos[leg_key][joint_key] = pwmservo
                    rc = joint.attach_pwmservo(pwmservo)
                    if rc != probopy.EA1_OK: return rc
        return rc
    def prepare_pca(self, device="/dev/i2c-1", i2c_addr=0x40, pwm_freq=60.0):
        u""" prepare PCA9685 controller and servomotors.
        connect to i2c device,
        set RS304MD servo channels.
        returns:
            probopy.EA1_OK, or probopy.ea1_status_enum。
        """
        self._pca1 = probopy.Pca9685()
        rc = self._pca1.init(device, i2c_addr, pwm_freq)
        if rc == probopy.EA1_OK:
            rc = self.set_pwm(self._pca1)
        return rc
        
def create_rat1(name="rat1"):
    u""" pj1をジョイントパラメータとするRatlインスタンスを生成して返す。
    """
    rat1 = Ratl(name = name)
    return rat1


