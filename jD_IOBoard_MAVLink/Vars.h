// Vars.h

static bool motor_armed = 0;

static uint8_t base_mode=0;
static uint8_t osd_mode = 0;
static uint8_t osd_nav_mode = 0;

static float        osd_vbat_A = 0;
static int8_t       osd_battery_remaining_A = 0;

static uint16_t     chan3_raw = 0;
static uint16_t     osd_chan8_raw = 1000;