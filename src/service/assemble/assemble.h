#ifndef _assemble_h_
#define _assemble_h_

#include <stdbool.h>

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

typedef enum {
    SYSTEM_STATUS_OK = 0,
    SYSTEM_STATUS_ERROR,
} SystemStatus;

extern volatile bool tim6_500hz_flag;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

SystemStatus assemble_delay(void);
SystemStatus assemble_log(void);
SystemStatus assemble_rgb(void);
SystemStatus assemble_imu(void);
SystemStatus assemble_motor(void);
SystemStatus assemble_vision(void);
SystemStatus assemble_balance(void);
SystemStatus assemble_tim6_500hz(void);

#endif
