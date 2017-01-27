#!/usr/bin/env python2
#coding=UTF-8
# rat with legs ロボットモデルクラス
# 2016-11-19 15:29:29 Sgurin6i

import math
import copy
import collections
import PyKDL
from PyKDL import Vector
import probopy

# 算数
PI = math.pi
def get_deg(rad):
    u"""角度 radian を degree に変えて出力"""
    return 180.0 * (rad / PI)
def get_rad(deg):
    u"""角度 degree を radian に変えて出力"""
    return PI * (deg / 180.0)

# [パラメータ]
"""
# key 呼称
"""
LEG_KEYS = ('lf', 'rf', 'lb', 'rb')
LEG_JOINT_KEYS = ('hip', 'thigh', 'shin')
LEG_SW_KEYS = ('foot_sw')
JOINT_PARAMETER_KEYS = ('offset', 'mag', 'pos_min', 'pos_max')
SW_PARAMETER_KEYS = ('on')
PWMSERVO_KEYS = ('ch', 'svtype')
SEG_PARAMETER_KEYS = ('joint', 'tip')
LEG_SEG_KEYS = ('body', 'hip', 'thigh', 'shin', 'foot_x', 'foot_y', 'foot_z')
RPY_KEYS = ('roll', 'pitch', 'yaw')

# パラメータ格納クラス。
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

# joint, sw 
class MdJoint(Md):
    u"""Md の内、key が JOINT_PARAMETER_KEYS に含まれていなければならないもの。
    """
    def __init__(self, **kwargs):
        for key, val in kwargs.items():
            assert(key in JOINT_PARAMETER_KEYS)
        Md.__init__(self, **kwargs)
class MdSw(Md):
    u"""Md の内、key が SW_PARAMETER_KEYS に含まれていなければならないもの。
    """
    def __init__(self, **kwargs):
        for key, val in kwargs.items():
            assert(key in SW_PARAMETER_KEYS)
        Md.__init__(self, **kwargs)
class MdLegJoint(Md):
    u"""Mdの内、key が LEG_JOINT_KEYS に含まれていたら、
    valueはMdJointのインスタンスに限定。
    key が LEG_SW_KEYS に含まれていたら、
    valueはMdSwのインスタンスに限定。
    それら以外のkeyは不可。
    """
    def __init__(self, **kwargs):
        for key, val in kwargs.items():
            if key in LEG_JOINT_KEYS:
                assert(isinstance(val, MdJoint))
            elif key in LEG_SW_KEYS:
                assert(isinstance(val, MdSw))
            else:
                raise KeyError , ("unknown key: " + str(key))
        Md.__init__(self, **kwargs)
class MdRatJoint(Md):
    u"""rat joint parameters.
    Mdの内、key が LEG_KEYS に含まれていなければならないもの。
    value は MdLegJoint のインスタンスでなければならない。
    """
    def __init__(self, **kwargs):
        for key, val in kwargs.items():
            assert(key in LEG_KEYS)
            assert(isinstance(val, MdLegJoint))
        Md.__init__(self, **kwargs)

# PWM servo
class MdPwmservo(Md):
    u"""Md の内、key が PWMSERVO_KEYS に含まれていなければならないもの。
    さらに、そのすべての key を使用している物。
    """
    def __init__(self, **kwargs):
        assert(len(kwargs) == len(PWMSERVO_KEYS))
        for key, val in kwargs.items():
            assert(key in PWMSERVO_KEYS)
        Md.__init__(self, **kwargs)
class MdLegPwmservo(Md):
    u"""Mdの内、keyがLEG_JOINT_KEYSに含まれていなければならないもの。
    valueはMdJointのインスタンスに限定。
     """
    def __init__(self, **kwargs):
        for key, val in kwargs.items():
            assert(isinstance(val, MdPwmservo))
            assert(key in LEG_JOINT_KEYS)
        Md.__init__(self, **kwargs)
class MdRatPwmservo(Md):
    u"""Md の内、key が LEG_KEYS に含まれていなければならないもの。
    value が MdLegPwmservo のインスタンスでなければならない。
    """
    def __init__(self, **kwargs):
        for key, val in kwargs.items():
            assert(isinstance(val, MdLegPwmservo))
            assert(key in LEG_KEYS)
        Md.__init__(self, **kwargs)

# segment
class MdSeg(Md):
    u"""Md の内、key が SEG_PARAMETER_KEYS に含まれていなければならないもの。
    さらに、そのすべての key を使用している物。
    Attributes:
      joint: 親につながるrotationalジョイントの軸方向 PyKDL.Vector。
             Vector(0,0,0) の場合はrigit connection。
      tip:   jointから先端tipまでの相対位置 PyKDL.Vector(単位mm)。
    """
    def __init__(self, **kwargs):
        assert(len(kwargs) == len(SEG_PARAMETER_KEYS))
        for key, val in kwargs.items():
            assert(key in SEG_PARAMETER_KEYS)
            assert(isinstance(val, Vector))
        Md.__init__(self, **kwargs)
class MdLegSeg(Md):
    u"""Md の内、keyがLEG_SEG_KEYSに含まれていなければならないもの。
    value が MdSeg のインスタンスでなければならないもの。
    Attributes:
      body:   body中心からLeg付け根に至るsegment値。通常 rigit connection。
      hip:    hip jointにつながるsegment値。
      thigh:  thigh jointにつながるsegment値。
      shin:   shin jointにつながるsegment値。
      foot_x: 足先から地面につながる仮想ジョイントX方向のsegment値。省略可。
      foot_y: 足先から地面につながる仮想ジョイントY方向のsegment値。省略可。
      foot_z: 足先から地面につながる仮想ジョイントZ方向のsegment値。省略可。
    """
    def __init__(self, foot_x = MdSeg(joint=Vector(1,0,0), tip=Vector(0,0,0)),
                  foot_y = MdSeg(joint=Vector(0,1,0), tip=Vector(0,0,0)),
                  foot_z = MdSeg(joint=Vector(0,0,1), tip=Vector(0,0,0)),
                 **kwargs):
        for val in foot_x, foot_y, foot_z:
            assert(isinstance(val, MdSeg))
        for key, val in kwargs.items():
            assert(isinstance(val, MdSeg))
            assert(key in LEG_SEG_KEYS)
        Md.__init__(self, foot_x=foot_x, foot_y=foot_y, foot_z=foot_z, **kwargs)
class MdRatSeg(Md):
    u"""Md の内、keyがLEG_KEYSに含まれていなければならないもの。
    value が MdLegSeg のインスタンスでなければならない。
    """
    def __init__(self, **kwargs):
        for key, val in kwargs.items():
            assert(isinstance(val, MdLegSeg))
            assert(key in LEG_KEYS)
        Md.__init__(self, **kwargs)

# Rotation
class MdRpy(Md):
    u"""Md の内、key が RPY_KEYS に含まれていなければならないもの。
    roll, pitch, yaw 回転。単位degree。
    rotation by first applying a rotation of roll around the x-axis,
    then a rotation of pitch around the original y-axis,
    and finally a rotation of yaw around the original z-axis.
    """
    def __init__(self, **kwargs):
        for key, val in kwargs.items():
            assert(key in RPY_KEYS)
        Md.__init__(self, **kwargs)

# rat1 のデフォルト MdRatJoint インスタンス。
# 足を下に伸ばしきった状態をposゼロとする。
# ゼロ方向を起点として本体の左側、前側に回る方向を正の方向とする。
# 右手系 X軸が進行方向、Y軸が左方向、Z軸が上方向。
# 軸に向かって反時計回りが正の回転方向。
# note:
# フタバRS304MDはホーン正面から見てホーンが時計回りに回る方向が正の方向。
MD_RAT_JOINT1 = MdRatJoint(
    lf = MdLegJoint(hip   = MdJoint(offset=-45.0, mag= 1.0, pos_min= -30.0, pos_max=120.0),
                    thigh = MdJoint(offset= 25.0, mag= 1.0, pos_min= -70.0, pos_max= 65.0),
                    shin  = MdJoint(offset=-65.0, mag= 1.0, pos_min= -45.0, pos_max=145.0),
                    foot_sw = MdSw(on=0)),
    rf = MdLegJoint(hip   = MdJoint(offset= 45.0, mag= 1.0, pos_min=-120.0, pos_max= 30.0),
                    thigh = MdJoint(offset= 25.0, mag=-1.0, pos_min= -70.0, pos_max= 65.0),
                    shin  = MdJoint(offset=-65.0, mag=-1.0, pos_min= -45.0, pos_max=145.0),
                    foot_sw = MdSw(on=0)),
    lb = MdLegJoint(hip   = MdJoint(offset=-45.0, mag= 1.0, pos_min= -30.0, pos_max=120.0),
                    thigh = MdJoint(offset= 25.0, mag= 1.0, pos_min= -70.0, pos_max= 65.0),
                    shin  = MdJoint(offset=-65.0, mag= 1.0, pos_min= -45.0, pos_max=145.0),
                    foot_sw = MdSw(on=0)),
    rb = MdLegJoint(hip   = MdJoint(offset= 45.0, mag= 1.0, pos_min=-120.0, pos_max= 30.0),
                    thigh = MdJoint(offset= 25.0, mag=-1.0, pos_min= -70.0, pos_max= 65.0),
                    shin  = MdJoint(offset=-65.0, mag=-1.0, pos_min= -45.0, pos_max=145.0),
                    foot_sw = MdSw(on=0)),
    )

# rat1 のデフォルト MdRatPwmservo インスタンス。
# 各ジョイントの pwmservo パラメータ。
# {'ch':<servo_ch>, 'svtype':<probopy.pwmservo_type_enum>}
MD_RAT_PWMSERVO1 = MdRatPwmservo(
    lf = MdLegPwmservo(hip   = MdPwmservo(ch= 0, svtype=probopy.PWM_SV_RS304MD),
                       thigh = MdPwmservo(ch= 1, svtype=probopy.PWM_SV_RS304MD),
                       shin  = MdPwmservo(ch= 2, svtype=probopy.PWM_SV_RS304MD)),
    rf = MdLegPwmservo(hip   = MdPwmservo(ch= 8, svtype=probopy.PWM_SV_RS304MD),
                       thigh = MdPwmservo(ch= 9, svtype=probopy.PWM_SV_RS304MD),
                       shin  = MdPwmservo(ch=10, svtype=probopy.PWM_SV_RS304MD)),
    lb = MdLegPwmservo(hip   = MdPwmservo(ch= 4, svtype=probopy.PWM_SV_RS304MD),
                       thigh = MdPwmservo(ch= 5, svtype=probopy.PWM_SV_RS304MD),
                       shin  = MdPwmservo(ch= 6, svtype=probopy.PWM_SV_RS304MD)),
    rb = MdLegPwmservo(hip   = MdPwmservo(ch=12, svtype=probopy.PWM_SV_RS304MD),
                       thigh = MdPwmservo(ch=13, svtype=probopy.PWM_SV_RS304MD),
                       shin  = MdPwmservo(ch=14, svtype=probopy.PWM_SV_RS304MD)),
    )

# leg 各segmentの長さ(mm)
#leg_seg_lens = { 'hip': 0.0, 'thigh': 58.0, 'shin': 80.0 }

# 各KDL segment パラメータ デフォルト値
MD_RAT_SEG1 = MdRatSeg(
    lf = MdLegSeg(body  = MdSeg(joint=Vector(0,0,0), tip=Vector(57.5, 64.0, 0.0)),
                  hip   = MdSeg(joint=Vector(1,0,0), tip=Vector(0.0, 0.0,   0.0)),
                  thigh = MdSeg(joint=Vector(0,-1,0), tip=Vector(0.0, 0.0, -58.0)),
                  shin  = MdSeg(joint=Vector(0,-1,0), tip=Vector(0.0, 0.0, -80.0))),
    rf = MdLegSeg(body  = MdSeg(joint=Vector(0,0,0), tip=Vector(57.5,-64.0, 0.0)),
                  hip   = MdSeg(joint=Vector(1,0,0), tip=Vector(0.0, 0.0,   0.0)),
                  thigh = MdSeg(joint=Vector(0,-1,0), tip=Vector(0.0, 0.0, -58.0)),
                  shin  = MdSeg(joint=Vector(0,-1,0), tip=Vector(0.0, 0.0, -80.0))),
    lb = MdLegSeg(body  = MdSeg(joint=Vector(0,0,0), tip=Vector(-57.5, 64.0, 0.0)),
                  hip   = MdSeg(joint=Vector(1,0,0), tip=Vector(0.0, 0.0,   0.0)),
                  thigh = MdSeg(joint=Vector(0,-1,0), tip=Vector(0.0, 0.0, -58.0)),
                  shin  = MdSeg(joint=Vector(0,-1,0), tip=Vector(0.0, 0.0, -80.0))),
    rb = MdLegSeg(body  = MdSeg(joint=Vector(0,0,0), tip=Vector(-57.5,-64.0, 0.0)),
                  hip   = MdSeg(joint=Vector(1,0,0), tip=Vector(0.0, 0.0,   0.0)),
                  thigh = MdSeg(joint=Vector(0,-1,0), tip=Vector(0.0, 0.0, -58.0)),
                  shin  = MdSeg(joint=Vector(0,-1,0), tip=Vector(0.0, 0.0, -80.0))),
    )

# Gyro座標系のbody座標系に対する回転。
# rat1のデフォルト値(degree)
MD_RAT_GYRO_RPY1 = MdRpy(roll = 0.0, pitch = 0.0, yaw =180.0)

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
        servo = self._joint.get_hwj()
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
        return self._joint
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
      md_leg_joint: joint parameter。MdLegJointインスタンス。
      name:          このインスタンスの名前 str。
    """
    def __init__(self, ctj, md_leg_joint, name=""):
        assert(isinstance(md_leg_joint, MdLegJoint))
        self._name = name
        self._mjs = {}
        self._ctj = ctj
        # Todo: mj_dict item数チェック
        for key in md_leg_joint:
            md = md_leg_joint[key]
            if isinstance(md, MdJoint):
                jt_name = "%s_jt_%s" % (name, key)
                jt = probopy.Joint(jt_name)
                self._ctj.add_child(jt)
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
        return self._mjs
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
    def target(self, *args, **kwargs):
        u"""各Magjのtarget posを設定する。
        一部だけ設定するのも可。
        Arguments:
          hip:   hip position数値 (degree)。
          thigh: thigh position数値 (degree)。
          shin:  shin position数値 (degree)。
        旧仕様:
        値は、LEG_JOINT_KEYS のメンバをkeyとするpos_dictによって与えられる。
        pos_dict = {'hip':10.0, 'thigh':20.0, 'shin':30.0} 等。
        一部だけ設定するのも可。
        余計なkey ('foot_x'等)は無視する。
        """
        # 引数解釈
        pos_dict = {}
        ix = 0 
        for arg in args:
            if isinstance(arg, dict): # 旧仕様
                pos_dict = arg
            else:
                if ix <= 3:
                    pos_dict[LEG_JOINT_KEYS[ix]] = arg
                    ix += 1
        for key in kwargs:
            if key == 'pos_dict':
                pos_dict = kwargs[key]
            else:
                pos_dict[key] = kwargs[key]
        # 関数実行    
        for key in pos_dict:
            if key in LEG_JOINT_KEYS:
                self._mjs[key].target(pos_dict[key])
    def set_kdl_segs(self, md_leg_seg):
        u""" set KDL segments.
        Arguments:
          md_leg_seg: セグメントパラメータ。MdLegSegインスタンス。
        """
        self._kdl_chain = PyKDL.Chain()
        for key in LEG_SEG_KEYS:
            name = "%s_%s" % (self._name, key)
            o0 = PyKDL.Vector(0,0,0)
            if key in md_leg_seg:
                md_seg = md_leg_seg[key]
            else:
                md_seg = MdSeg(joint=Vector(0,0,0), tip=Vector(0,0,0))
            if md_seg['joint'] == Vector(0,0,0):
                kdl_joint = PyKDL.Joint("j_" + name, PyKDL.Joint.None)
            else:
                kdl_joint = PyKDL.Joint("j_" + name, o0, md_seg['joint'],
                                        PyKDL.Joint.RotAxis)
            kdl_tip = PyKDL.Frame(md_seg['tip'])
            kdl_seg = PyKDL.Segment("s_" + name, kdl_joint, kdl_tip)
            self._kdl_chain.addSegment(kdl_seg)
        self._kdl_fksolver = PyKDL.ChainFkSolverPos_recursive(self._kdl_chain)
        self._kdl_iksolver_vel = PyKDL.ChainIkSolverVel_pinv(self._kdl_chain)
        self._kdl_iksolver = PyKDL.ChainIkSolverPos_NR(
            self._kdl_chain, self._kdl_fksolver, self._kdl_iksolver_vel, 100, 1e-6)
    def get_kdl_jnt_array(self):
        u""" get current joint values of PyKDL chain.
        Returns:
          j_array: instance of PyKDL.JntArray. with current joint values.
        """
        n_j = self._kdl_chain.getNrOfJoints()
        n_seg = self._kdl_chain.getNrOfSegments()
        j_array = PyKDL.JntArray(n_j)
        assert n_seg == len(LEG_SEG_KEYS), "%d != %d" %(n_j, len(LEG_SEG_KEYS))
        ix_j = 0
        for ix_seg in range(n_seg):
            key = LEG_SEG_KEYS[ix_seg]
            seg = self._kdl_chain.getSegment(ix_seg)
            if seg.getJoint().getType() != PyKDL.Joint.None:
                if key in LEG_JOINT_KEYS:
                    pos = self.get_mjs()[key].get_curr_pos()
                else:
                    pos = 0.0
                j_array[ix_j] = get_rad(pos)
                ix_j += 1
        return j_array
        
    def get_fk_vec(self):
        u""" get current forward kinetic position vector.
        Returns:
          vec: 現在の足先位置を表す PyKDL.Vectorインスタンス。
          rc:  PyKDL fk solver JntToCart()メソッドのリターンコード。
        """
        j_array = self.get_kdl_jnt_array()
        frame = PyKDL.Frame()
        rc = self._kdl_fksolver.JntToCart(j_array, frame)
        return frame.p, rc
    def calc_ik_jnt_array(self, vec):
        u""" 現在位置から vec に移動した場合の PyKdl.JntArray計算値を返す。
        Arguments:
          vec: ターゲット足先位置を表す PyKDL.Vectorインスタンス。
        Returns:
          j_array: calculated instance of PyKDL.JntArray. 
          rc: PyKDL ik solver CartToJnt()メソッドのリターンコード。
        """
        j_start = self.get_kdl_jnt_array()
        f_end = PyKDL.Frame(vec)
        n_j = self._kdl_chain.getNrOfJoints()
        j_array = PyKDL.JntArray(n_j)
        rc = self._kdl_iksolver.CartToJnt(j_start, f_end, j_array)
        return j_array, rc
    def calc_ik_pos(self, vec):
        u""" 現在位置から vec に移動した場合の joint positions計算値を返す。
        Arguments:
          vec:      ターゲット足先位置を表す PyKDL.Vectorインスタンス。
        Returns:
          pos_dict: LEG_SEG_KEYS のメンバのうちjointを有するものをkeyとする 
                    joint positionの辞書。
          rc:       PyKDL ik solver CartToJnt()メソッドのリターンコード。
        """
        pos_dict = {}
        for key in LEG_JOINT_KEYS:
            pos_dict[key] = 0.0
        j_array, rc = self.calc_ik_jnt_array(vec)
        n_j = self._kdl_chain.getNrOfJoints()
        n_seg = self._kdl_chain.getNrOfSegments()
        ix_j = 0
        for ix_seg in range(n_seg):
            key = LEG_SEG_KEYS[ix_seg]
            seg = self._kdl_chain.getSegment(ix_seg)
            if seg.getJoint().getType() != PyKDL.Joint.None:
                if key in LEG_SEG_KEYS:
                    pos_dict[key] = get_deg(j_array[ix_j])
                ix_j += 1
        return pos_dict, rc
    def set_ik_target(self, vec):
        u""" set inverse kinetic target.
        Arguments:
          vec: ターゲット足先位置を表す PyKDL.Vectorインスタンス。
        Returns:
          PyKDL ik solver CartToJnt()メソッドのリターンコード。
        """
        pos_dict, rc = self.calc_ik_pos(vec)
        if rc >= 0:
            self.target(pos_dict)
        return rc

# 四本足 rat with legs クラス
class Ratl:
    u""" 通常4本の Leg からなる rat のクラス。
    Attributes:
      md_rat_joint: joint parameter。MdRatJointインスタンス。
      name:          このインスタンスの名前 str。
    """
    def __init__(self, md_rat_joint=MD_RAT_JOINT1, name="rat1"):
        assert(isinstance(md_rat_joint, MdRatJoint))
        self._name = name
        self._body = probopy.Body(name)
        self._body.set_tick(50.0)
        self._ctj = probopy.Controller(name + "_ctj")
        self._body.add_child(self._ctj)
        self._gyro = None
        self._pwmservos = {} # GC阻止の為にpwmservo貯めとく
        self._legs = {}
        assert(isinstance(md_rat_joint, MdRatJoint))
        for leg_key in md_rat_joint:
            self._legs[leg_key] = Leg(ctj = self._ctj,
                                      md_leg_joint = md_rat_joint[leg_key],
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
    def get_legs(self):
        u""" 保有するLegの辞書、
        即ち LEG_KEYS をキーとし、probo.Leg インスタンスを値とする辞書を返す。
        """
        return self._legs
    def set_pwm(self, pwmc, md_rat_pwmservo=MD_RAT_PWMSERVO1):
        u""" set pwm controller and pwm servo interfaces.
        Arguments:
          pwmc: probopy.Pwmc init済を期待する。
          md_rat_pwmservo: MdRatPwmservoインスタンス。各ジョイントの pwmservo パラメータ。
        返り値: probopy.EA1_OK, or probopy.ea1_status_enum。
        """
        assert(isinstance(md_rat_pwmservo, MdRatPwmservo))
        assert(isinstance(pwmc, probopy.Hwc))
        rc = self._ctj.attach_hwc(pwmc)
        if rc == probopy.EA1_OK:
            for leg_key in md_rat_pwmservo:
                self._pwmservos[leg_key] = {}
                for joint_key in md_rat_pwmservo[leg_key]:
                    joint = self._legs[leg_key].get_mj(joint_key).get_joint()
                    dargs = MD_RAT_PWMSERVO1[leg_key][joint_key]
                    pwmservo = probopy.Pwmservo(dargs['ch'], dargs['svtype'])
                    self._pwmservos[leg_key][joint_key] = pwmservo
                    rc = joint.attach_hwj(pwmservo)
                    if rc != probopy.EA1_OK: return rc
        return rc
    def prepare_pca(self, device="/dev/i2c-1", i2c_addr=0x40, pwm_freq=60.0,
                    md_rat_pwmservo=MD_RAT_PWMSERVO1):
        u""" prepare PCA9685 controller and servomotors.
        connect to i2c device,
        set RS304MD servo channels.
        Arguments:
          device:   i2c device name string
          i2c_addr: i2c address of the device.
          pwm_freq: PWM frequency(Hz).
          md_rat_pwmservo: MdRatPwmservoインスタンス。各ジョイントの pwmservo パラメータ。
        returns:
            probopy.EA1_OK, or probopy.ea1_status_enum。
        """
        self._pca1 = probopy.Pca9685()
        rc = self._pca1.init(device, i2c_addr, pwm_freq)
        if rc == probopy.EA1_OK:
            rc = self.set_pwm(self._pca1, md_rat_pwmservo)
        return rc
    def set_gyro_rpy(self, rpy=MD_RAT_GYRO_RPY1):
        u"""GyroのBodyに対する回転を設定する。
        Arguments:
          rpy:  回転角(degree)。MdRpyのインスタンス。
        設定されるもの:
          gyro出力(Rg)をBodyの回転(Rb)に変換するための行列を設定する。
          self._gyro_rot_left: Rgの左側から掛ける回転行列。
          self._gyro_rot_right: Rgの右側から掛ける回転行列。
        """
        self._gyro_rot_left = PyKDL.Rotation.RPY(0.0, 0.0, get_rad(rpy["yaw"]))
        tf_gyro = PyKDL.Rotation.RPY(
            get_rad(rpy["roll"]), get_rad(rpy["pitch"]), get_rad(rpy["yaw"]))
        self._gyro_rot_right = tf_gyro.Inverse()

    def prepare_mpu6050(self, device="/dev/i2c-1", i2c_addr=0x68, rpy=MD_RAT_GYRO_RPY1):
        u""" prepare MPU6050 Gyro 
        connect to i2c device,
        Arguments:
          device:   i2c device name string
          i2c_addr: i2c address of the device.
        returns:
            probopy.EA1_OK, or probopy.ea1_status_enum。
        """
        gyro = probopy.Pmpu6050(self._name + "_mpu6050")
        rc = gyro.init(device, i2c_addr)
        if rc == probopy.EA1_OK:
            rc = self._body.add_child(gyro)
        if rc == probopy.EA1_OK:
            self._gyro = gyro
        if rc == probopy.EA1_OK:
            self.set_gyro_rpy(rpy)
        return rc
    def set_kdl_segs(self, md_rat_seg=MD_RAT_SEG1):
        u""" set KDL segments.
        Arguments:
          md_rat_seg: セグメントパラメータ。MdRatSegインスタンス。
        """
        assert(isinstance(md_rat_seg, MdRatSeg))
        for key, val in md_rat_seg.items():
            self._legs[key].set_kdl_segs(val)
    def get_gyro_curr_q(self):
        u""" get current Gyro angle quaternion.
        Returns:
          rc: return code. 失敗したら負数を返す。
          x: quaternion x
          y: quaternion y
          z: quaternion z
          w: quaternion w
        """
        if self._gyro:
            rc,gx,gy,gz,gw = self._gyro.get_curr_quat()
            rot_gyro = PyKDL.Rotation.Quaternion(gx,gy,gz,gw)
            rot_body = self._gyro_rot_left \
                       * rot_gyro \
                       * self._gyro_rot_right
            x,y,z,w = rot_body.GetQuaternion()
            return rc,x,y,z,w
        else:
            return probopy.EA1_E_NOT_READY, 0.0, 0.0, 0.0, 1.0

def create_rat1(name="rat1"):
    u""" MD_RAT_JOINT1をジョイントパラメータとするRatlインスタンスを生成して返す。
    """
    rat1 = Ratl(name = name)
    return rat1

# お試し
if __name__ == '__main__':
    dt1 = 50  # 遷移時間(ms)
    rat1 = Ratl(name="ratl")
    rat1.set_kdl_segs()
    rc = rat1.prepare_pca()
    print "prepare_pca rc =", rc
    # fk
    for leg_key in LEG_KEYS:
        rat1.get_legs()[leg_key].target(0, 0, 0)
    rat1.get_body().do_em_in(0)
    rat1.get_body().do_em_in(dt1)
    print "zero position."
    # neutral position vector
    print "zero position foot axis:"
    vecs_zero = {}
    for key in LEG_KEYS:
        vecs_zero[key], rc = rat1.get_legs()[key].get_fk_vec()
        print " ", key, vecs_zero[key]
        assert rc >= 0
    # prompt
    aa = raw_input('press enter > ')
    # neutral position
    target_hts = {'hip':0, 'thigh':-32, 'shin': 83.0}
    for leg_key in LEG_KEYS:
        rat1.get_legs()[leg_key].target(**target_hts)
    rat1.get_body().do_em_in(dt1)
    vec1, rc = rat1.get_legs()['lf'].get_fk_vec()
    print "newtral hts", target_hts
    print "neutral xyz lf", vec1
    print "delta xyz lf  ", vec1 - vecs_zero['lf']
    

