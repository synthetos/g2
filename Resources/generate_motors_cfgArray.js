#!/usr/bin/env node

// Run this with no arguments, and get the motor code suitable for use in config_app.cpp

for (n=1; n<=6; n++) {
  console.log(`
#if (MOTORS >= ${n})
    { "${n}","${n}ma", _fip,  0, st_print_ma, get_ui8, st_set_ma,  (float *)&st_cfg.mot[MOTOR_${n}].motor_map,      M${n}_MOTOR_MAP },
    { "${n}","${n}sa", _fip,  3, st_print_sa, get_flt, st_set_sa,  (float *)&st_cfg.mot[MOTOR_${n}].step_angle,     M${n}_STEP_ANGLE },
    { "${n}","${n}tr", _fipc, 4, st_print_tr, get_flt, st_set_tr,  (float *)&st_cfg.mot[MOTOR_${n}].travel_rev,     M${n}_TRAVEL_PER_REV },
    { "${n}","${n}mi", _fip,  0, st_print_mi, get_int, st_set_mi,  (float *)&st_cfg.mot[MOTOR_${n}].microsteps,     M${n}_MICROSTEPS },
    { "${n}","${n}su", _fipi, 5, st_print_su, st_get_su,st_set_su, (float *)&st_cfg.mot[MOTOR_${n}].steps_per_unit, M${n}_STEPS_PER_UNIT },
    { "${n}","${n}po", _fip,  0, st_print_po, get_ui8, set_01,     (float *)&st_cfg.mot[MOTOR_${n}].polarity,       M${n}_POLARITY },
    { "${n}","${n}pm", _fip,  0, st_print_pm, st_get_pm,st_set_pm, (float *)&cs.null,                            M${n}_POWER_MODE },
    { "${n}","${n}pl", _fip,  3, st_print_pl, get_flt, st_set_pl,  (float *)&st_cfg.mot[MOTOR_${n}].power_level,    M${n}_POWER_LEVEL },
//  { "${n}","${n}pi", _fip,  3, st_print_pi, get_flt, st_set_pi,  (float *)&st_cfg.mot[MOTOR_${n}].power_idle,     M${n}_POWER_IDLE },
//  { "${n}","${n}mt", _fip,  2, st_print_mt, get_flt, st_set_mt,  (float *)&st_cfg.mot[MOTOR_${n}].motor_timeout,  M${n}_MOTOR_TIMEOUT },
#ifdef MOTOR_${n}_IS_TRINAMIC
    { "${n}","${n}ts",  _f0,  0, tx_print_nul, motor_${n}.get_ts_fn,  set_ro,              (float *)&motor_${n}, 0 },
    { "${n}","${n}pth", _fip, 0, tx_print_nul, motor_${n}.get_pth_fn, motor_${n}.set_pth_fn,  (float *)&motor_${n}, M${n}_TMC2130_TPWMTHRS },
    { "${n}","${n}cth", _fip, 0, tx_print_nul, motor_${n}.get_cth_fn, motor_${n}.set_cth_fn,  (float *)&motor_${n}, M${n}_TMC2130_TCOOLTHRS },
    { "${n}","${n}hth", _fip, 0, tx_print_nul, motor_${n}.get_hth_fn, motor_${n}.set_hth_fn,  (float *)&motor_${n}, M${n}_TMC2130_THIGH },
    { "${n}","${n}sgt", _fip, 0, tx_print_nul, motor_${n}.get_sgt_fn, motor_${n}.set_sgt_fn,  (float *)&motor_${n}, M${n}_TMC2130_SGT },
    { "${n}","${n}sgr", _f0,  0, tx_print_nul, motor_${n}.get_sgr_fn, set_ro,              (float *)&motor_${n}, 0 },
    { "${n}","${n}csa", _f0,  0, tx_print_nul, motor_${n}.get_csa_fn, set_ro,              (float *)&motor_${n}, 0 },
    { "${n}","${n}sgs", _f0,  0, tx_print_nul, motor_${n}.get_sgs_fn, set_ro,              (float *)&motor_${n}, 0 },
    { "${n}","${n}tbl", _fip, 0, tx_print_nul, motor_${n}.get_tbl_fn, motor_${n}.set_tbl_fn,  (float *)&motor_${n}, M${n}_TMC2130_TBL },
    { "${n}","${n}pgrd",_fip, 0, tx_print_nul, motor_${n}.get_pgrd_fn,motor_${n}.set_pgrd_fn, (float *)&motor_${n}, M${n}_TMC2130_PWM_GRAD },
    { "${n}","${n}pamp",_fip, 0, tx_print_nul, motor_${n}.get_pamp_fn,motor_${n}.set_pamp_fn, (float *)&motor_${n}, M${n}_TMC2130_PWM_AMPL },
    { "${n}","${n}hend",_fip, 0, tx_print_nul, motor_${n}.get_hend_fn,motor_${n}.set_hend_fn, (float *)&motor_${n}, M${n}_TMC2130_HEND },
    { "${n}","${n}hsrt",_fip, 0, tx_print_nul, motor_${n}.get_hsrt_fn,motor_${n}.set_hsrt_fn, (float *)&motor_${n}, M${n}_TMC2130_HSTRT },
    { "${n}","${n}smin",_fip, 0, tx_print_nul, motor_${n}.get_smin_fn,motor_${n}.set_smin_fn, (float *)&motor_${n}, M${n}_TMC2130_SMIN },
    { "${n}","${n}smax",_fip, 0, tx_print_nul, motor_${n}.get_smax_fn,motor_${n}.set_smax_fn, (float *)&motor_${n}, M${n}_TMC2130_SMAX },
    { "${n}","${n}sup", _fip, 0, tx_print_nul, motor_${n}.get_sup_fn, motor_${n}.set_sup_fn,  (float *)&motor_${n}, M${n}_TMC2130_SUP },
    { "${n}","${n}sdn", _fip, 0, tx_print_nul, motor_${n}.get_sdn_fn, motor_${n}.set_sdn_fn,  (float *)&motor_${n}, M${n}_TMC2130_SDN },
#endif
#endif`);
}
