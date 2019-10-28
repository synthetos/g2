#!/usr/bin/env node

// Run this with no arguments, and get the motor code suitable for use in config_app.cpp

console.log(`
    // Digital input configs
    // generated with \${PROJECT_ROOT}/Resources/generate_dins_cfgArray.js
`)

for (n=1; n<=12; n++) {
  console.log(`
#if (D_IN_CHANNELS >= ${n})
    { "di${n}","di${n}en",_bip,   0, din_print_en, din_get_en, din_set_en, &din${n},  DI${n}_ENABLED },
    { "di${n}","di${n}po",_iip,   0, din_print_po, din_get_po, din_set_po, &din${n},  DI${n}_POLARITY },
    { "di${n}","di${n}ac",_iip,   0, din_print_ac, din_get_ac, din_set_ac, &din${n},  DI${n}_ACTION },
    { "di${n}","di${n}in",_iip,   0, din_print_in, din_get_in, din_set_in, &din${n},  DI${n}_EXTERNAL_NUMBER },
#endif`);
}

console.log(`
    // END generated with \${PROJECT_ROOT}/Resources/generate_dins_cfgArray.js
`)
