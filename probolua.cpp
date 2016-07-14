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
#include "probolua.hpp"
#include "ea1/ea1_debug.h"
#include "ea1/ea1_benri.h"
#include "probo.hpp"

#define LOG_TAG "probolua"

/* メタテーブル名 */
#define BODY_MT "body_mt"   /* Bodyメンバ関数用メタテーブル名。 */
#define CONTROLLER_MT "controller_mt"   /* Controllerメンバ関数用メタテーブル名。 */
#define JOINT_MT "joint_mt"   /* Jointメンバ関数用メタテーブル名。 */

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
  Pfamily::Parent * obj = *(Pfamily::Parent **)lua_touserdata(L,1);
  int amt = obj->get_children_amt();
  lua_pushnumber(L, amt);
  return 1;
}
static int parent_get_name_lua(lua_State* L)
{
  Pfamily::Parent * obj = *(Pfamily::Parent **)lua_touserdata(L,1);
  const std::string name = obj->get_p_name();
  lua_pushstring(L, name.c_str());
  return 1;
}
/* child関数 */
static int child_get_name_lua(lua_State* L)
{
  Pfamily::Child * obj = *(Pfamily::Child **)lua_touserdata(L,1);
  const std::string name = obj->get_name();
  lua_pushstring(L, name.c_str());
  return 1;
}

static int body_do_em_in_lua(lua_State* L)
{
  int nargs = lua_gettop(L);
  if (nargs != 2)
    return luaL_error( L, "%s: wrong nargs %d != 2", __func__, nargs);
  Probo::Body* body = *(Probo::Body **)lua_touserdata(L,1);
  double time = lua_tonumber(L,2);
  int rc = body->do_em_in( time );
  if (rc != EA1_OK)
    return luaL_error( L, "%s: failed body->do_em_in(%f)", __func__, time);
  return 0;
}

static int body_set_tick_lua(lua_State* L)
{
  int nargs = lua_gettop(L);
  if (nargs != 2)
    return luaL_error( L, "%s: wrong nargs %d != 2", __func__, nargs);
  Probo::Body* body = *(Probo::Body **)lua_touserdata(L,1);
  double tick = lua_tonumber(L,2);
  int rc = body->set_tick( tick );
  if (rc != EA1_OK)
    return luaL_error( L, "%s: failed body->set_tick(%f)", __func__, tick);
  return 0;
}
static int body_get_tick_lua(lua_State* L)
{
  Probo::Body* body = *(Probo::Body **)lua_touserdata(L,1);
  double tick = body->get_tick();
  lua_pushnumber( L, tick );
  return 1;
}
  
  static int joint_target_lua(lua_State* L)
{
  int nargs = lua_gettop(L);
  int rc;
  if (nargs != 2)
    return luaL_error( L, "%s: wrong nargs %d != 2", __func__, nargs);
  Probo::Joint* joint = *(Probo::Joint **)lua_touserdata(L,1);
  double f_target = lua_tonumber(L,2);
  rc = joint->target(f_target);
  if (rc != EA1_OK)
    return luaL_error( L, "%s: failed joint->target(%f)", __func__, f_target);
  return 0;
}
static int joint_get_curr_pos_lua(lua_State* L)
{
  Probo::Joint* joint = *(Probo::Joint **)lua_touserdata(L,1);
  double pos = joint->get_curr_pos();
  lua_pushnumber( L, pos );
  return 1;
}

  
/* 各関数登録 */
static const luaL_Reg joint_funcs[] = 
  {
    {"get_name", child_get_name_lua},
    {"target", joint_target_lua},
    {"get_curr_pos", joint_get_curr_pos_lua},
    {NULL, NULL}
  };

static int controller_create_joint_lua(lua_State* L)
{
  std::string name = "joint";
  int nargs = lua_gettop(L);
  Probo::Controller * cp;
  if ((nargs < 1) || (! lua_isuserdata(L,1)))
    return luaL_error( L, "%s: wrong arg. nargs %d", __func__, nargs);
  cp = *(Probo::Controller **)lua_touserdata(L,1);
  if ((nargs >= 2) && lua_isstring(L, 2))
    { name = lua_tostring(L,2); }
  if(cp == NULL)
    return luaL_error( L, "%s: userdata Controller is NULL", __func__);
  Probo::Joint ** jt_pp
    = (Probo::Joint **)lua_newuserdata(L, sizeof(Probo::Joint *)); // stack: u /..
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

/* controller メソッド関数 */
static const luaL_Reg controller_funcs[] = 
  {
    {"create_joint", controller_create_joint_lua},
    {"get_children_amt", parent_get_children_amt_lua},
    {"get_name", parent_get_name_lua},
    {NULL, NULL}
  };

static int body_create_controller_lua(lua_State* L)
{
  std::string name = "ctlr";
  int nargs = lua_gettop(L);
  Probo::Body * bp;
  if ((nargs < 1) || (! lua_isuserdata(L,1)))
    return luaL_error( L, "%s: wrong arg. nargs %d", __func__, nargs);
  bp = *(Probo::Body **)lua_touserdata(L,1);
  if ((nargs >= 2) && lua_isstring(L, 2))
    { name = lua_tostring(L,2); }
  if(bp == NULL)
    return luaL_error( L, "%s: userdata Body is NULL", __func__);
  Probo::Controller ** ct_pp
    = (Probo::Controller **)lua_newuserdata(L, sizeof(Probo::Controller *)); // stack: u /..
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
  Probo::Body* object = *(Probo::Body **)lua_touserdata(L,1);
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
  if ((lua_gettop(L) >= 1) && lua_isstring(L, -1))
    { name = lua_tostring(L,1); }
  Probo::Body ** bpp
    = (Probo::Body **)lua_newuserdata(L, sizeof(Probo::Body *)); // stack: u /..
  if (bpp == NULL)
    return luaL_error( L, "failed lua_newuserdata()" );
  *bpp = new Probo::Body(name);
  if (*bpp == NULL)
    return luaL_error( L, "failed new Probo::Body()");
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

/* lua state, C++API関数一式準備 */
lua_State* Probolua::prepare_lua_api()
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
 RET:
  if ((rc != EA1_OK) && (L != NULL))
    {
      lua_close(L);
      L = NULL;
    }
  return L;
}

/* .luaファイル読み込んで実行。 */
int Probolua::call_luafile(lua_State* L, const char *filename)
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
int Probolua::com_main(int argc, char *argv[])
{
  int rc = EA1_OK;
  lua_State *L = Probolua::prepare_lua_api();
  const char * luafile = "tseq1.lua";
  if (L == NULL)
    { rc = EA1_FAIL; goto RET; }
  if (argc > 1)
    {
      luafile = argv[1];
    }
  rc = Probolua::call_luafile( L, luafile );
 RET:
  if(L)
    lua_close(L);
  return rc;
}

/* test main */
int main(int argc, char *argv[])
{
  int rc = EA1_OK;
  LOGI("test starts");
  //rc = Probo::test_main(argc, argv);
  rc = Probolua::com_main(argc, argv);
  
  return rc;
}
