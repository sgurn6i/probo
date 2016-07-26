/* main.cpp    -*- C++ -*-
   main関数
   2016-07-15 16:32:34 Sgurn6i
*/
#include "ea1/ea1_debug.h"
#include "ea1/ea1_benri.h"
#include "probo.hpp"
#include "probolua.hpp"
#define LOG_TAG "main"

int main(int argc, char *argv[])
{
  int rc = EA1_OK;
  //LOGI("probo test starts");
  //rc = probo::test_main(argc, argv);
  LOGI("probolua starts");
  rc = probo::lua_main(argc, argv);
  
  return rc;
}
