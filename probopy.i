/* probopy.i -*- C++ -*-
 * python interface SWIG定義
 * 2016-11-08 15:31:29 Sgurn6i
 */
%module probopy
%{
  #include "ea1/ea1_benri.h"
  #include "ea1/ea1_named.hpp"
  #include "ea1/ea1_composite.hpp"
  #include "probo.hpp"
  #include "probopwm.hpp"
  #include "ppca9685.hpp"
  #include "pgyro.hpp"
  #include "pmpu6050.hpp"
  #include "pgpio.hpp"
  #include "pbbbgpio.hpp"
%}
%include <std_string.i>
%include "ea1/ea1_benri.h"

%feature("notabstract") Pca9685;
%apply double* OUTPUT {double * out_x, double * out_y, double * out_z, double * out_w}
%include "ea1/ea1_named.hpp"
%include "ea1/ea1_composite.hpp"
%include "probo.hpp"
%include "probopwm.hpp"
%include "ppca9685.hpp"
%include "pgyro.hpp"
%include "pmpu6050.hpp"
%include "pgpio.hpp"
%include "pbbbgpio.hpp"
