/* probolua.hpp    -*- C++ -*-
   probo LUAインターフェース
   2016-06-24 17:59:33 Sgurn6i
*/
#ifndef _PROBOLUA_H_
#define _PROBOLUA_H_
#include "lua.hpp"
#include "lualib.h"
#include "lauxlib.h"

namespace Probolua
{
  /* lua state, C++API関数一式準備 */
  lua_State* prepare_lua_api();
  /* .luaファイル読み込んで実行。 */
  int call_luafile(lua_State* L, const char *filename);
  /* コマンド実行 main */
  int com_main(int argc, char *argv[]);
}


#endif /* _PROBOLUA_H_ */
