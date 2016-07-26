/* probolua.cpp    -*- C++ -*-
   probo LUAインターフェース
   2016-06-24 14:33:30 Sgurn6i
*/
/* [Lua側操作例]
   local b1 = create_body("body1")
   local c1 = b1:create_controller("ctlr1")
   local j1 = c1:create_joint("joint1")
   print (c1:get_name().." children: "..c1:get_children_amt())
   print (b1:get_name().." children: "..b1:get_children_amt())
   b1:set_tick(20.0)
   print ("tick: " .. b1:get_tick())
   j1:target(22.5)
   b1:do_em_in(50)
   print (j1:get_name() .. " pos: " .. j1:get_curr_pos()) 
 */
#include <string.h>
#include <string>
#include <map>
#include "probolua.hpp"
#include "ea1/ea1_debug.h"
#include "ea1/ea1_benri.h"
#include "probo.hpp"
#include "ppca9685.hpp"
#include "probopwm.hpp"

#define LOG_TAG "probolua"

/* メタテーブル名 */
#define BODY_MT "body_mt"   /* Bodyメンバ関数用メタテーブル名。 */
#define CONTROLLER_MT "controller_mt"   /* Controllerメンバ関数用メタテーブル名。 */
#define JOINT_MT "joint_mt"   /* Jointメンバ関数用メタテーブル名。 */
#define PWMC_MT "pwmc_mt"   /* Pwmcメンバ関数用メタテーブル名。 */
#define PWMSERVO_MT "pwmservo_mt"   /* Pwmservoメンバ関数用メタテーブル名。 */
/* luaから呼び出す時に使う pwmc type名 */
#define PWMC_TYPE_PCA9685   "pca9685"
/* PWM servo type */
#define PWMSERVO_TYPE_DEFAULT  "default"
#define PWMSERVO_TYPE_CUSTOM   "custom"
#define PWMSERVO_TYPE_RS304MD  "rs304md"
static const std::map<std::string, probo::pwmservo_type_t> s_pwmservo_types_map
= {
  /* string for lua, pwmservo_type_t */
  {PWMSERVO_TYPE_DEFAULT, probo::PWM_SV_DEFAULT},
  {PWMSERVO_TYPE_RS304MD, probo::PWM_SV_RS304MD},
};

/* Lua userdataからBaseポインタを取り出して所定クラスポインタにdynamic_castする。 */
/* cast 失敗したら luaL_error. てことで NULL を返すことは無い。 */
#define DCAST_LUA_UD( L, ptype, ud )                 \
  dcast_lua_ud<ptype>( L, ud, #ptype )
template <typename T> T dcast_lua_ud(lua_State* L, void * ud, const char * t_name = "")
{
  T p = NULL;
  if (ud != NULL)
    p = dynamic_cast<T>( *(pfamily::Base **)(ud) );
  if (p == NULL)
    luaL_error( L, "%s: no userdata of type %s", __func__, t_name );
  return p;

}

/* 指定された名前のメタテーブルをゲットしてスタックに積む。
   無ければ作る、関数登録する。 */
static int get_or_new_mt (lua_State* L, const char * mt_name,
                          const luaL_Reg * funcs,
                          lua_CFunction gc_func)
{
  int rc = EA1_OK;
  int rc_lua = luaL_newmetatable( L, mt_name ); // stack mt{} / (?)
  /* 新しくmetatableができたなら各関数登録。 */
  if (rc_lua)
    {
      lua_newtable( L );        // stack: {} / mt{} / (?)
      if (funcs != NULL)
        {
          luaL_setfuncs( L, funcs, 0 );       // stack: {funcs} / mt{} / (?)
          lua_setfield( L, -2, "__index" );    // stack: mt{_index={funcs}} / (?)
        }
      if (gc_func != NULL)
        {
          lua_pushcfunction( L,  gc_func); // stack: gc_func / mt{_index} / (?)
          lua_setfield( L, -2, "__gc" );   // stack: mt{_index, __gc} / (?)
        }
    }
  return rc;
}

/* Parent関数 */
static int parent_get_children_amt_lua(lua_State* L)
{
  pfamily::Parent * obj = DCAST_LUA_UD( L, pfamily::Parent*, lua_touserdata(L,1) );
  int amt = obj->get_children_amt();
  lua_pushnumber(L, amt);
  return 1;
}
static int parent_get_name_lua(lua_State* L)
{
  pfamily::Parent * obj = DCAST_LUA_UD( L, pfamily::Parent*, lua_touserdata(L,1) );
  const std::string name = obj->get_p_name();
  lua_pushstring(L, name.c_str());
  return 1;
}
/* child関数 */
static int child_get_name_lua(lua_State* L)
{
  pfamily::Child * obj = DCAST_LUA_UD( L, pfamily::Child*, lua_touserdata(L,1) );
  const std::string name = obj->get_name();
  lua_pushstring(L, name.c_str());
  return 1;
}

static int body_do_em_in_lua(lua_State* L)
{
  int nargs = lua_gettop(L);
  if ((nargs != 2) || (! lua_isnumber(L,2)))
    return luaL_error( L, "%s: wrong arguments, nargs %d", __func__, nargs);
  probo::Body* body = DCAST_LUA_UD( L, probo::Body*, lua_touserdata(L,1) );
  double time = lua_tonumber(L,2);
  int rc = body->do_em_in( time );
  if (rc != EA1_OK)
    return luaL_error( L, "%s: failed body->do_em_in(%f)", __func__, time);
  return 0;
}

static int body_set_tick_lua(lua_State* L)
{
  int nargs = lua_gettop(L);
  if ((nargs != 2) || (! lua_isnumber(L,2)))
    return luaL_error( L, "%s: wrong arguments, nargs %d", __func__, nargs);
  probo::Body* body = DCAST_LUA_UD( L, probo::Body*, lua_touserdata(L,1) );
  double tick = lua_tonumber(L,2);
  int rc = body->set_tick( tick );
  if (rc != EA1_OK)
    return luaL_error( L, "%s: failed body->set_tick(%f)", __func__, tick);
  return 0;
}
static int body_get_tick_lua(lua_State* L)
{
  probo::Body* body = DCAST_LUA_UD( L, probo::Body*, lua_touserdata(L,1) );
  double tick = body->get_tick();
  lua_pushnumber( L, tick );
  return 1;
}
  
static int joint_target_lua(lua_State* L)
{
  int nargs = lua_gettop(L);
  int rc;
  if ((nargs != 2) || (! lua_isnumber(L,2)))
    return luaL_error( L, "%s: wrong arguments, nargs %d", __func__, nargs);
  probo::Joint* joint = DCAST_LUA_UD( L, probo::Joint*, lua_touserdata(L,1) );
  double f_target = lua_tonumber(L,2);
  rc = joint->target(f_target);
  if (rc != EA1_OK)
    return luaL_error( L, "%s: failed joint->target(%f)", __func__, f_target);
  return 0;
}
static int joint_get_curr_pos_lua(lua_State* L)
{
  probo::Joint* joint = DCAST_LUA_UD( L, probo::Joint*, lua_touserdata(L,1) );
  double pos = joint->get_curr_pos();
  lua_pushnumber( L, pos );
  return 1;
}

static int joint_attach_pwmservo_lua( lua_State* L )
{
  probo::Joint* joint
    = DCAST_LUA_UD(L, probo::Joint*, lua_touserdata(L,1) );
  if (! lua_isuserdata(L, 2))
    return luaL_error( L, "%s: want arg = pwmservo", __func__ );
  probo::Pwmservo* pwmservo
    = DCAST_LUA_UD(L, probo::Pwmservo*, lua_touserdata(L,2) );
  if (pwmservo == NULL)
    return luaL_error( L, "%s: not pwmservo type", __func__ );
  int rc = joint->attach_pwmservo( *pwmservo );
  if (rc != EA1_OK)
    return luaL_error( L, "%s: attach failed rc = %d", __func__, rc );
  return 0;
}
  
/* 各関数登録 */
static const luaL_Reg joint_funcs[] = 
  {
    {"get_name", child_get_name_lua},
    {"target", joint_target_lua},
    {"get_curr_pos", joint_get_curr_pos_lua},
    {"attach_pwmservo", joint_attach_pwmservo_lua},
    {NULL, NULL}
  };

static int controller_create_joint_lua(lua_State* L)
{
  std::string name = "joint";
  int nargs = lua_gettop(L);
  probo::Controller* cp = DCAST_LUA_UD( L, probo::Controller*, lua_touserdata(L,1) );
  if ((nargs >= 2) && lua_isstring(L, 2))
    { name = lua_tostring(L,2); }
  probo::Joint ** jt_pp
    = (probo::Joint **)lua_newuserdata(L, sizeof(probo::Joint *)); // stack: u /..
  if (jt_pp == NULL)
    return luaL_error( L, "failed lua_newuserdata()") ;
  *jt_pp = cp->create_joint(name);
  if (*jt_pp == NULL)
    return luaL_error( L, "failed cp->create_joint(\"%s\")", name.c_str());
  /* controller メソッドメタテーブル */
  get_or_new_mt(L, JOINT_MT, joint_funcs, NULL);  // stack: mt / u
  lua_setmetatable(L, -2);  // stack: u(with mt)
  return 1;
}

static int controller_attach_pwmc_lua( lua_State* L )
{
  probo::Controller* controller
    = DCAST_LUA_UD(L, probo::Controller*, lua_touserdata(L,1) );
  if (! lua_isuserdata(L, 2))
    return luaL_error( L, "%s: want arg = pwmc", __func__ );
  probo::Pwmc* pwmc
    = DCAST_LUA_UD(L, probo::Pwmc*, lua_touserdata(L,2) );
  if (pwmc == NULL)
    return luaL_error( L, "%s: not pwmc type", __func__ );
  
  int rc = controller->attach_pwmc( *pwmc );
  if (rc != EA1_OK)
    return luaL_error( L, "%s: attach failed rc = %d", __func__, rc );
  return 0;
}

/* controller メソッド関数 */
static const luaL_Reg controller_funcs[] = 
  {
    {"create_joint", controller_create_joint_lua},
    {"get_children_amt", parent_get_children_amt_lua},
    {"get_name", parent_get_name_lua},
    {"attach_pwmc", controller_attach_pwmc_lua},
    {NULL, NULL}
  };

static int body_create_controller_lua(lua_State* L)
{
  std::string name = "ctlr";
  int nargs = lua_gettop(L);
  probo::Body* bp = DCAST_LUA_UD( L, probo::Body*, lua_touserdata(L,1) );
  if ((nargs < 1) || (! lua_isuserdata(L,1)))
    return luaL_error( L, "%s: wrong arg. nargs %d", __func__, nargs);
  if ((nargs >= 2) && lua_isstring(L, 2))
    { name = lua_tostring(L,2); }
  probo::Controller ** ct_pp
    = (probo::Controller **)lua_newuserdata(L, sizeof(probo::Controller *)); // stack: u /..
  if (ct_pp == NULL)
    return luaL_error( L, "failed lua_newuserdata()") ;
  *ct_pp = bp->create_controller(name);
  if (*ct_pp == NULL)
    return luaL_error( L, "failed bp->create_controller(\"%s\")", name.c_str());
  /* controller メソッドメタテーブル */
  get_or_new_mt(L, CONTROLLER_MT, controller_funcs, NULL);  // stack: mt / u
  lua_setmetatable(L, -2);  // stack: u(with mt)
  return 1;
}

/* gc destructor */
static int body_delete_lua(lua_State* L)
{
  probo::Body* object = DCAST_LUA_UD( L, probo::Body*, lua_touserdata(L,1) );
  EA1_SAFE_DELETE(object);
  return 0;
}

/* body method funcs. */
static const luaL_Reg body_funcs[] = 
  {
    {"do_em_in", body_do_em_in_lua},
    {"get_tick", body_get_tick_lua},
    {"set_tick", body_set_tick_lua},
    {"create_controller", body_create_controller_lua},
    {"get_children_amt", parent_get_children_amt_lua},
    {"get_name", parent_get_name_lua},
    {NULL, NULL}
  };

static int create_body_lua(lua_State* L)
{
  int rc = EA1_OK;
  std::string name = "body";
  if ((lua_gettop(L) >= 1) && lua_isstring(L, 1))
    { name = lua_tostring(L,1); }
  probo::Body ** bpp
    = (probo::Body **)lua_newuserdata(L, sizeof(probo::Body *)); // stack: u /..
  if (bpp == NULL)
    return luaL_error( L, "failed lua_newuserdata()" );
  *bpp = new probo::Body(name);
  if (*bpp == NULL)
    return luaL_error( L, "failed new probo::Body()");
  rc = get_or_new_mt(L, BODY_MT, body_funcs, body_delete_lua);  // stack: mt / u
  if (rc != EA1_OK)
    {
      /* todo: ここで deleteしても手遅れなような..  */
      EA1_SAFE_DELETE(*bpp);
      return luaL_error( L, "failed get_body_mt(L)");
    }
  lua_setmetatable(L, -2);  // stack: u(with mt)
  return 1; // userdata 1個を返す。
}

/* pwmc method funcs. */
static const luaL_Reg pwmc_funcs[] = 
  {
    {NULL, NULL}
  };

/* pwmc gc delete */
static int pwmc_delete_lua(lua_State* L)
{
  probo::Pwmc* object =  DCAST_LUA_UD( L, probo::Pwmc*, lua_touserdata(L,1) );
  EA1_SAFE_DELETE(object);
  return 0;
}

/* 入力引数argcの数に応じて pca9685 initして
 * is_initialized()でないとエラーにする。 */
static int init_pca9685( probo::Pca9685* obj, int argc, lua_State* L )
{
  int rc = EA1_OK;
  std::string device;
  int i2c_addr;
  double pwm_freq;
  if (argc < 2)
    rc = obj->init();
  else
    {
      device = lua_tostring( L, 2 );
      if (argc < 3)
        rc = obj->init( device );
      else
        {
          i2c_addr = lua_tonumber( L, 3 );
          if (argc < 4 )
            rc = obj->init( device, i2c_addr );
          else
            {
              pwm_freq = lua_tonumber( L, 4 );
              rc = obj->init( device, i2c_addr, pwm_freq );
            }
        }
    }
  if (rc < 0)
    return rc;
  if (! obj->is_initialized())
    {
      LOGE("%s: initialization failed.", __func__);
      return EA1_FAIL;
    }
  return rc;
}

/* Lua: create_pwmc( "pca9685", "/dev/i2c-2", 0x40, 60.0 )  */
static int create_pwmc_lua(lua_State* L)
{
  int rc = EA1_OK;
  int argc = lua_gettop(L);
  if ((argc < 1) || ! lua_isstring(L, 1))
    return luaL_error( L, "need PWMC type string" );
  std::string pwmc_type = lua_tostring(L,1);
  probo::Pwmc ** obj_pp = NULL;
  obj_pp = (probo::Pwmc **)lua_newuserdata(L, sizeof(probo::Pwmc *)); // stack: u /..
  if (obj_pp == NULL)
    return luaL_error( L, "failed lua_newuserdata()" );
  if (pwmc_type == PWMC_TYPE_PCA9685)
    {
      probo::Pca9685 * obj = new probo::Pca9685();
      if (obj == NULL)
        return luaL_error( L, "failed new probo::Pca9685()");
      *obj_pp = obj;
      /* 引数argcの数に応じてinitして is_initialized()でないとエラーにする。 */
      rc = init_pca9685( obj, argc, L );
      if (rc < 0)
        {
          EA1_SAFE_DELETE( obj );
          return luaL_error( L, "failed to init pca9685 rc %d", rc);
        }
    }
  else /* nomatch pwmc_type */
    return luaL_error( L, "unknown pwmc_type %s", pwmc_type.c_str() );

  rc = get_or_new_mt(L, PWMC_MT, pwmc_funcs, pwmc_delete_lua);  // stack: mt / u
  if (rc != EA1_OK)
    {
      /* todo: ここで deleteしても手遅れなような..  */
      EA1_SAFE_DELETE(*obj_pp);
      return luaL_error( L, "failed get_pwmc_mt(L)");
    }
  lua_setmetatable(L, -2);  // stack: u(with mt)
  return 1; // userdata 1個を返す。
}

/* pwm servo */
static int pwmservo_init( lua_State* L, probo::Pwmservo * pwmservo, int ch, const std::string& type_str )
{
  int rc = EA1_OK;
  if (s_pwmservo_types_map.find( type_str ) == s_pwmservo_types_map.end())
    {
      LOGE( "%s: unknown servo type %s", __func__, type_str.c_str() );
      return EA1_EINVAL;
    }
  probo::pwmservo_type_t type_num = s_pwmservo_types_map.at( type_str );
  pwmservo->init( ch, type_num );
  if (! pwmservo->is_initialized())
    {
      LOGE ("%s: failed to pwmservo->init( %d, %s )", __func__, ch, type_str.c_str() );
      return EA1_EINVAL;
    }
  return rc;
}

static int pwmservo_init_lua(lua_State* L)
{
  int argc = lua_gettop(L);
  /* CH、サーボタイプ引数必要 (ch, servo_type) */
  if ((argc < 2) || ! lua_isnumber(L, 2) || ! lua_isstring(L, 3))
    return luaL_error( L, "need PWM servo CH number and type string" );
  probo::Pwmservo* object = DCAST_LUA_UD( L, probo::Pwmservo*, lua_touserdata(L,1) );
  int ch = lua_tonumber( L, 2 );
  const std::string type_str = lua_tostring( L, 3 );
  int rc = pwmservo_init( L, object, ch, type_str );
  if (rc != EA1_OK)
    return luaL_error( L, "failed to pwm servo init( %d, %s )", ch, type_str.c_str() );
  return 0;
}

/* pwm servo method funcs. */
static const luaL_Reg pwmservo_funcs[] = 
  {
    {"init", pwmservo_init_lua},
    /* Todo: list_types */
    {NULL, NULL}
  };

/* pwm servo gc delete */
static int pwmservo_delete_lua(lua_State* L)
{
  probo::Pwmservo* object =  DCAST_LUA_UD( L, probo::Pwmservo*, lua_touserdata(L,1) );
  EA1_SAFE_DELETE(object);
  return 0;
}

/* Lua: create_pwmc( "pca9685", "/dev/i2c-2", 0x40, 60.0 )  */
static int create_pwmservo_lua(lua_State* L)
{
  int argc = lua_gettop(L);
  int rc;
  probo::Pwmservo * pwmservo = new probo::Pwmservo();
  if (pwmservo == NULL)
    return luaL_error( L, "failed to create Pwmservo()");
  probo::Pwmservo ** obj_pp = NULL;
  obj_pp = (probo::Pwmservo **)lua_newuserdata(L, sizeof(probo::Pwmservo *)); // stack: u /..
  if (obj_pp == NULL)
    return luaL_error( L, "failed lua_newuserdata()" );
  *obj_pp = pwmservo;
  if (argc < 1)
    {
      /* 引数なし、後でinit()必要。 */
    }
  else
    {
      /* サーボタイプ引数あり (ch, servo_type) */
      if ((argc < 2) || ! lua_isnumber(L, 1) || ! lua_isstring(L, 2))
        return luaL_error( L, "need PWM servo CH number and type string" );
      int ch = lua_tonumber( L, 1 );
      const std::string type_str = lua_tostring( L, 2 );
      rc = pwmservo_init( L, pwmservo, ch, type_str );
      if (rc != EA1_OK)
        {
          EA1_SAFE_DELETE(pwmservo);
          return luaL_error( L, "failed to create pwm servo( %d, %s )", ch, type_str.c_str() );
        }
    }
  rc = get_or_new_mt(L, PWMSERVO_MT, pwmservo_funcs, pwmservo_delete_lua);  // stack: mt / u
  if (rc != EA1_OK)
    {
      EA1_SAFE_DELETE(pwmservo);
      return luaL_error( L, "failed get_pwmservo_mt(L)");
    }
  lua_setmetatable(L, -2);  // stack: u(with mt)
  return 1; // userdata 1個を返す。
}

/* lua state, C++API関数一式準備 */
lua_State* probo::prepare_lua_api()
{
  int rc = EA1_OK;
  lua_State *L = luaL_newstate();
  if (L == NULL)
    {
      LOGE("%s: failed luaL_newstate()", __func__);
      rc = EA1_FAIL;
      goto RET;
    }
  luaL_openlibs(L);
  lua_register(L, "create_body", create_body_lua);
  lua_register(L, "create_pwmc", create_pwmc_lua);
  lua_register(L, "create_pwmservo", create_pwmservo_lua);
 RET:
  if ((rc != EA1_OK) && (L != NULL))
    {
      lua_close(L);
      L = NULL;
    }
  return L;
}


 /* .luaファイル読み込んで実行。 */
int probo::call_luafile(lua_State* L, const char *filename)
{
  int rc = EA1_OK;
  int lua_rc = luaL_loadfile(L, filename);
  if (lua_rc != LUA_OK)
    {
      LOGE("%s: failed luaL_loadfile(L, \"%s\")", __func__, filename);
      rc = EA1_FILE_ERROR;
      goto RET;
    }
  lua_rc = lua_pcall(L, 0, 0, 0);   /* 読み込んだLuaファイルを実行。 */
  if (lua_rc != LUA_OK)
    {
      if (lua_gettop(L) > 0)
        {
          const std::string msg  = lua_tostring(L,-1);
          LOGI("%s", msg.c_str());
        }
      LOGE("%s: failed lua_pcall(L, 0, 0, 0), rc %d", __func__, lua_rc);
      rc = EA1_FAIL;
      goto RET;
    }

 RET:  
  return rc;
}

/* コマンド実行 main */
int probo::lua_main(int argc, char *argv[])
{
  int rc = EA1_OK;
  lua_State *L = probo::prepare_lua_api();
  const char * luafile = "tseq1.lua";
  if (L == NULL)
    { rc = EA1_FAIL; goto RET; }
  if (argc > 1)
    {
      luafile = argv[1];
    }
  rc = probo::call_luafile( L, luafile );
 RET:
  if(L)
    lua_close(L);
  return rc;
}

