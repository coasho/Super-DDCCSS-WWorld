/* fsim's opaque C VecEnv handle type (design 9.2): the one C value the C ABI (fsim/fsim_c.h) and the C++ VecEnv
 * facade (fsim/VecEnv.h, whose handle() hands it out for mixed-language use) share. A declaration only: the functions
 * that take it are fsim_c.h's - include that header to call them. */
#ifndef FSIM_VECENV_HANDLE_H
#define FSIM_VECENV_HANDLE_H

typedef struct fsim_vecenv fsim_vecenv;

#endif
