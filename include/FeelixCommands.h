#ifndef FEELIXCOMMANDS_h
#define FEELIXCOMMANDS_h


#define M_DATA_SEQUENCE     'F'
#define M_ID                'I'
#define M_POLE_PAIRS        'P'
#define M_ZERO_EL_ANGLE     'Z'
#define M_SENSOR_DIRECTION  'N'
#define M_SENSOR_OFFSET     'O'
#define M_MAG_ENC_PARTNR    'E'
#define M_MAG_ENC_BIT_RES   'B'
#define M_MAG_ENC_CLK_SPD   'C'
#define M_SUPPLY_VOL        'S'
#define M_VOL_LIMIT         'L'
#define M_VEL_LIMIT         'V'
#define M_COMM_SPEED        'T'
#define M_SENSOR_DIR        'D'
#define M_CALIBRATE         'R'
#define M_ANGLE_PID         'A'
#define M_VEL_PID           'Q'
#define M_RANGE             'J'
#define M_CONSTRAIN_RANGE   'M'
#define M_CURRENT_SENSE     'G'
#define M_CALIBRATE_CS      'U'
#define M_CS_THRESHOLD      'Y'
#define M_OVERHEAT_PROTECT  'X'
// #define M_I2C_CONNECTION    'c'
#define M_STARTPOS          '%'

#define M_PLAY              'K'
#define M_LOOP              'H'
#define M_RETURN            'W'


#define CMD_E_DIR           'D' //!< effect experienced in direction cw / ccw
#define CMD_E_FLIP          'F' //!< effect flipped horizontal / vertical
#define CMD_E_POS           'P' //!< effect translation x / y
#define CMD_E_ANGLE         'A' //!< effect angle/width
#define CMD_E_SCALE         'S' //!< effect scale width / height (%)
#define CMD_E_INF           'I' //!< effect repeat infinitely (every rotation)
#define CMD_E_COPIES        'C' //!< copies of same effect
#define CMD_E_CONTROL_TYPE  'T' //!< control type
#define CMD_E_EFFECT_TYPE   'E' //!< visualization type
#define CMD_E_MIDI_CC       'M' //!< Effect information for CC changes created by the midi
#define CMD_E_DATA_SIZE     'Z'
#define CMD_E_POINTER       'R'
#define CMD_E_QUALITY       'Q'
#define CMD_READ            'G'

#define F_AMPLIFY           'A'
#define F_CONSTRAIN         'C'
#define F_NOISE             'N'
#define F_RESET             'R'

#define G_ANGLE             'A'

#endif