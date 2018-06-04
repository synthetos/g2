#!/usr/bin/env node

// Run this with no arguments, and get the motor code suitable for use in config_app.cpp

console.log('// START Generated with ${PROJECT_ROOT}/Resources/generate_motors_default_config.js');
for (n=1; n<=6; n++) {
  console.log(
`#ifndef M${n}_TMC2130_TPWMTHRS
    #define M${n}_TMC2130_TPWMTHRS         1200                    // ${n}pth
#endif
#ifndef M${n}_TMC2130_TCOOLTHRS
    #define M${n}_TMC2130_TCOOLTHRS        1000                    // ${n}cth
#endif
#ifndef M${n}_TMC2130_THIGH
    #define M${n}_TMC2130_THIGH            10                      // ${n}hth
#endif
#ifndef M${n}_TMC2130_SGT
    #define M${n}_TMC2130_SGT              4                       // ${n}sgt
#endif
#ifndef M${n}_TMC2130_TBL
    #define M${n}_TMC2130_TBL              2                       // ${n}tbl
#endif
#ifndef M${n}_TMC2130_PWM_GRAD
    #define M${n}_TMC2130_PWM_GRAD         1                       // ${n}pgrd
#endif
#ifndef M${n}_TMC2130_PWM_AMPL
    #define M${n}_TMC2130_PWM_AMPL         200                     // ${n}pamp
#endif
#ifndef M${n}_TMC2130_HEND
    #define M${n}_TMC2130_HEND             0                       // ${n}hend
#endif
#ifndef M${n}_TMC2130_HSTRT
    #define M${n}_TMC2130_HSTRT            0                       // ${n}hsrt
#endif
#ifndef M${n}_TMC2130_SMIN
    #define M${n}_TMC2130_SMIN             5                       // ${n}smin
#endif
#ifndef M${n}_TMC2130_SMAX
    #define M${n}_TMC2130_SMAX             5                       // ${n}smax
#endif
#ifndef M${n}_TMC2130_SUP
    #define M${n}_TMC2130_SUP              2                       // ${n}sup
#endif
#ifndef M${n}_TMC2130_SDN
    #define M${n}_TMC2130_SDN              1                       // ${n}sdn
#endif
`
);
}
console.log('// END Generated');
