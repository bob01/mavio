///////////////////////////
// Global variables

static int Out[] = {8,9,10,4,3,2};   // Output I/O pin array

static byte patt_pos;
static byte patt;
static byte pattByteA;
static byte pattByteB;

// MAVLink session control
static boolean  mavbeat = 0;
static float    lastMAVBeat = 0;
static boolean  waitingMAVBeats = 1;
static uint8_t  apm_mav_type;
static uint8_t  apm_mav_system; 
static uint8_t  apm_mav_component;
static boolean  enable_mav_request = 0;

// read from MAVLink
static bool motor_armed = 0;

static uint8_t base_mode=0;
static uint8_t osd_mode = 0;
static uint8_t osd_nav_mode = 0;

static float osd_vbat_A = 0;
static int8_t osd_battery_remaining_A = 0;

static uint16_t chan3_raw = 0;
static uint16_t osd_chan8_raw = 1000;