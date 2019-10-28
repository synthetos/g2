#!/usr/bin/env node

// Run this with no arguments, and get the motor code suitable for use in config_app.cpp

console.log(`
  // Motor parameters
  // generated with \${PROJECT_ROOT}/Resources/generate_motors_cfgArray.js
`)

for (n=1; n<=6; n++) {
  console.log(`
#if (MOTORS >= ${n})
    { "${n}","${n}ma", _iip,  0, st_print_ma, st_get_ma, st_set_ma, &st_cfg.mot[MOTOR_${n}].motor_map,      M${n}_MOTOR_MAP },
    { "${n}","${n}sa", _fip,  3, st_print_sa, st_get_sa, st_set_sa, &st_cfg.mot[MOTOR_${n}].step_angle,     M${n}_STEP_ANGLE },
    { "${n}","${n}tr", _fipc, 4, st_print_tr, st_get_tr, st_set_tr, &st_cfg.mot[MOTOR_${n}].travel_rev,     M${n}_TRAVEL_PER_REV },
    { "${n}","${n}su", _fipi, 5, st_print_su, st_get_su, st_set_su, &st_cfg.mot[MOTOR_${n}].steps_per_unit, M${n}_STEPS_PER_UNIT },
    { "${n}","${n}mi", _iip,  0, st_print_mi, st_get_mi, st_set_mi, &st_cfg.mot[MOTOR_${n}].microsteps,     M${n}_MICROSTEPS },
    { "${n}","${n}po", _iip,  0, st_print_po, st_get_po, st_set_po, &st_cfg.mot[MOTOR_${n}].polarity,       M${n}_POLARITY },
    { "${n}","${n}pm", _iip,  0, st_print_pm, st_get_pm, st_set_pm, &cs.null,                            M${n}_POWER_MODE },
    { "${n}","${n}pl", _fip,  3, st_print_pl, st_get_pl, st_set_pl, &st_cfg.mot[MOTOR_${n}].power_level,    M${n}_POWER_LEVEL },
    { "${n}","${n}ep", _iip,  0, st_print_ep, st_get_ep, st_set_ep, nullptr,                             M${n}_ENABLE_POLARITY },
    { "${n}","${n}sp", _iip,  0, st_print_sp, st_get_sp, st_set_sp, nullptr,                             M${n}_STEP_POLARITY },
//  { "${n}","${n}pi", _fip,  3, st_print_pi, st_get_pi, st_set_pi, &st_cfg.mot[MOTOR_${n}].power_idle,     M${n}_POWER_IDLE },
//  { "${n}","${n}mt", _fip,  2, st_print_mt, st_get_mt, st_set_mt, &st_cfg.mot[MOTOR_${n}].motor_timeout,  M${n}_MOTOR_TIMEOUT },
#ifdef MOTOR_${n}_IS_TRINAMIC
    { "${n}","${n}ts",  _i0,  0, tx_print_nul, motor_${n}.get_ts_fn,  set_ro,              &motor_${n}, 0 },
    { "${n}","${n}pth", _iip, 0, tx_print_nul, motor_${n}.get_pth_fn, motor_${n}.set_pth_fn,  &motor_${n}, M${n}_TMC2130_TPWMTHRS },
    { "${n}","${n}cth", _iip, 0, tx_print_nul, motor_${n}.get_cth_fn, motor_${n}.set_cth_fn,  &motor_${n}, M${n}_TMC2130_TCOOLTHRS },
    { "${n}","${n}hth", _iip, 0, tx_print_nul, motor_${n}.get_hth_fn, motor_${n}.set_hth_fn,  &motor_${n}, M${n}_TMC2130_THIGH },
    { "${n}","${n}sgt", _iip, 0, tx_print_nul, motor_${n}.get_sgt_fn, motor_${n}.set_sgt_fn,  &motor_${n}, M${n}_TMC2130_SGT },
    { "${n}","${n}sgr", _i0,  0, tx_print_nul, motor_${n}.get_sgr_fn, set_ro,              &motor_${n}, 0 },
    { "${n}","${n}csa", _i0,  0, tx_print_nul, motor_${n}.get_csa_fn, set_ro,              &motor_${n}, 0 },
    { "${n}","${n}sgs", _i0,  0, tx_print_nul, motor_${n}.get_sgs_fn, set_ro,              &motor_${n}, 0 },
    { "${n}","${n}tbl", _iip, 0, tx_print_nul, motor_${n}.get_tbl_fn, motor_${n}.set_tbl_fn,  &motor_${n}, M${n}_TMC2130_TBL },
    { "${n}","${n}pgrd",_iip, 0, tx_print_nul, motor_${n}.get_pgrd_fn,motor_${n}.set_pgrd_fn, &motor_${n}, M${n}_TMC2130_PWM_GRAD },
    { "${n}","${n}pamp",_iip, 0, tx_print_nul, motor_${n}.get_pamp_fn,motor_${n}.set_pamp_fn, &motor_${n}, M${n}_TMC2130_PWM_AMPL },
    { "${n}","${n}hend",_iip, 0, tx_print_nul, motor_${n}.get_hend_fn,motor_${n}.set_hend_fn, &motor_${n}, M${n}_TMC2130_HEND },
    { "${n}","${n}hsrt",_iip, 0, tx_print_nul, motor_${n}.get_hsrt_fn,motor_${n}.set_hsrt_fn, &motor_${n}, M${n}_TMC2130_HSTRT },
    { "${n}","${n}smin",_iip, 0, tx_print_nul, motor_${n}.get_smin_fn,motor_${n}.set_smin_fn, &motor_${n}, M${n}_TMC2130_SMIN },
    { "${n}","${n}smax",_iip, 0, tx_print_nul, motor_${n}.get_smax_fn,motor_${n}.set_smax_fn, &motor_${n}, M${n}_TMC2130_SMAX },
    { "${n}","${n}sup", _iip, 0, tx_print_nul, motor_${n}.get_sup_fn, motor_${n}.set_sup_fn,  &motor_${n}, M${n}_TMC2130_SUP },
    { "${n}","${n}sdn", _iip, 0, tx_print_nul, motor_${n}.get_sdn_fn, motor_${n}.set_sdn_fn,  &motor_${n}, M${n}_TMC2130_SDN },
#endif
#endif`);
}

console.log(`
  // END generated with \${PROJECT_ROOT}/Resources/generate_motors_cfgArray.js
`)
