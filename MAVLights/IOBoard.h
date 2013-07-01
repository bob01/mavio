///////////////////////////
// Constants

#define PATT_UNKNOWN            0
#define PATT_NOMAVLINK          17


///////////////////////////
// Global variables

static int Out[] = { 8, 9, 10, 4, 3, 2 };   // Output I/O pin array

static byte patt_pos;
static byte patt = PATT_UNKNOWN;
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
static bool ml_motor_armed = 0;

static uint8_t ml_base_mode=0;
static uint8_t ml_mode = 0;
static uint8_t ml_nav_mode = 0;

static float ml_vbat_A = 0;
static int8_t ml_battery_remaining_A = 0;

static uint16_t ml_chan3_raw = 0;
static int16_t ml_chan8_scaled = -1000;