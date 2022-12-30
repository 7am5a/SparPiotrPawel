#ifndef ASCII_TABLE
#define ASCII_TABLE

/*  
    Create some custom characters and set addres. 
    Definition of characters from Raystar RC1602XX documentation (LCD manufacturer).
    Some of characters  from manufacturer data table was not write - it will be
    not used in project. 
*/

/**
 * @brief Definition of C shape character - empty/20% cell-  begin of loading bar
 * 
 */
uint8_t open_bar[] = {
    0b00000,
    0b11111,
    0b10000,
    0b10000,
    0b10000,
    0b10000,
    0b11111,
    0b00000
};

/**
 * @brief Definition of = shape character - empty cell - part of loading bar
 * 
 */
uint8_t empty_bar[] = {
    0b00000,
    0b11111,
    0b00000,
    0b00000,
    0b00000,
    0b00000,
    0b11111,
    0b00000
};

/**
 * @brief Definition of mirrored C shape character- empty end cell of loading bar
 * 
 */
uint8_t close_bar[] = {
    0b00000,
    0b11111,
    0b00001,
    0b00001,
    0b00001,
    0b00001,
    0b11111,
    0b00000
};

/**
 * @brief Definition of C shape character - 40% full cell of loading bar
 * 
 */
uint8_t fil_40_bar[] = {
    0b00000,
    0b11111,
    0b11000,
    0b11000,
    0b11000,
    0b11000,
    0b11111,
    0b00000
};

/**
 * @brief Defintion of C shape character - 60% full cell of loading bar
 * 
 */
uint8_t fil_60_bar[] = {
    0b00000,
    0b11111,
    0b11100,
    0b11100,
    0b11100,
    0b11100,
    0b11100,
    0b00000
};

/**
 * @brief Definition of C shape character - 80% full cell of loading bar
 * 
 */
uint8_t fil_80_bar[] = {
    0b00000,
    0b11111,
    0b11110,
    0b11110,
    0b11110,
    0b11110,
    0b11110,
    0b00000
};

/**
 * @brief Definition of rectangular shape - 100% full cell of loading bar
 * 
 */
uint8_t fil_100_bar[] = {
    0b00000,
    0b11111,
    0b11111,
    0b11111,
    0b11111,
    0b11111,
    0b11111,
    0b00000
};

//                          0b0000XXXX - cusom characters
#define OPEN_BAR            0b00000000 //can be used as FILL_20_BAR - same shape
#define FILL_20_BAR         0b00000000
#define EMPTY_BAR           0b00000001
#define CLOSE_BAR           0b00000010
//#define FILL_20_BAR       0b00000011
#define FILL_40_BAR         0b00000100
#define FILL_50_BAR         0b00000101
#define FILL_60_BAR         0b00000110
#define FILL_100_BAR        0b00000111
//Probably 8 slots left for another characters

//                          0b0001XXXX
#define TRADE_MARK          0b00010000
#define CROSS               0b00010001
#define PARAGRAPH           0b00010010
#define ENTER_SIGN          0b00010011
#define GAMMA               0b00010100
#define DELTA               0b00010101
#define THETA               0b00010110
#define LAMBDA              0b00010111
#define KSI                 0b00011000
#define PI                  0b00011001
#define SIGMA               0b00011010
#define TEMP_CHAR           0b00011011
#define PHI                 0b00011100
#define PSI                 0b00011101
#define OMEGA               0b00011110
#define ALPHA               0b00011111

//                          0b0010XXXX
#define EMPTY               0b00100000
#define EXCLAMATION         0b00100001
#define QUOTE               0b00100010
#define HASH                0b00100011
#define DOLLAR              0b00100100
#define PERCENT             0b00100101
#define AND                 0b00100110
#define PRIME               0b00100111
#define OP_PARENTHESIS      0b00101000
#define CL_PARENTHESIS      0b00101001
#define ASTERISK            0b00101010
#define PLUS                0b00101011
#define COMMA               0b00101100
#define DASH                0b00101101
#define DOT                 0b00101110
#define SLASH               0b00101111

//                          0b0011XXXX
#define _0                  0b00110000
#define _1                  0b00110001
#define _2                  0b00110010
#define _3                  0b00110011
#define _4                  0b00110100
#define _5                  0b00110101
#define _6                  0b00110110
#define _7                  0b00110111
#define _8                  0b00111000
#define _9                  0b00111001
#define COLON               0b00111010
#define SEMICOLON           0b00111011
#define LESS_THAN           0b00111100
#define EQUALS              0b00111101
#define GREATER_THAN        0b00111110
#define QUESTION            0b00111111

//                          0b0100XXXX
#define AT                  0b01000000
#define _A                  0b01000001               
#define _B                  0b01000010
#define _C                  0b01000011
#define _D                  0b01000100
#define _E                  0b01000101
#define _F                  0b01000110
#define _G                  0b01000111
#define _H                  0b01001000
#define _I                  0b01001001
#define _J                  0b01001010
#define _K                  0b01001011
#define _L                  0b01001100
#define _M                  0b01001101
#define _N                  0b01001110
#define _O                  0b01001111

//                          b0101XXXX
#define _P                  0b01010000
#define _Q                  0b01010001
#define _R                  0b01010010
#define _S                  0b01010011
#define _T                  0b01010100
#define _U                  0b01010101
#define _V                  0b01010110
#define _W                  0b01010111
#define _X                  0b01011000
#define _Y                  0b01011001
#define _Z                  0b01011010
#define OP_BRACKET          0b01011011
#define YEN                 0b01011100
#define CL_BRACKET          0b01011101
#define CIRCUMFLEX          0b01011110
#define UNDERSCORE          0b01011111

//                          0b0110XXXX
//#define LONG_BACKTICK     0b01100000
#define _a                  0b01100001
#define _b                  0b01100010
#define _c                  0b01100011
#define _d                  0b01100100
#define _e                  0b01100101
#define _f                  0b01100110
#define _g                  0b01100111
#define _h                  0b01101000  
#define _i                  0b01101001
#define _j                  0b01101010
#define _k                  0b01101011
#define _l                  0b01101100
#define _m                  0b01101101
#define _n                  0b01101110
#define _o                  0b01101111

//                          0b0111XXXX
#define _p                  0b01110000
#define _q                  0b01110001
#define _r                  0b01110010
#define _s                  0b01110011
#define _t                  0b01110100
#define _u                  0b01110101
#define _v                  0b01110110
#define _w                  0b01110111
#define _x                  0b01111000
#define _y                  0b01111001
#define _z                  0b01111010
#define OPEN_BRACE          0b01111011
#define PIPE                0b01111100
#define CLOSE_BRACE         0b01111101
#define RIGHT_ARROW         0b01111110
#define LEFT_ARROW          0b01111111

//Skipped - unused chars    0b1000XXXX

//                          0b1001XXXX
#define ROT_QUESTION        0b10011111

//                          0b1010XXXX
#define MID_DOT             0b10100101

//Skipped - unused chars    0b1011XXXX

//Skipped - unused chars    0b1100XXXX

//                          0b1101XXXX
#define DEGREE              0b10111111

//Skipped - unused chars    0b1110XXXX

//                          0b1111XXXX
#define UP_POINT            0b11110000
#define UP_2_POINTS         0b11110001
#define SLIM_DEGREE         0b11110010
#define BACKTICK            0b11110011
#define TICK                0b11110100
#define HALF                0b11110101
#define QUATER              0b11110110
#define MULTIPLE            0b11110111
#define DIVIDE              0b11111000
#define LESS_OR_EQUAL       0b11111001
#define GREATER_OR_EQUAL    0b11111010
#define MUCH_LESS_THAN      0b11111011
#define MUCH_GREATER_THAN   0b11111100
#define NOT_EQUAL           0b11111101
#define ROOT                0b11111110
#define UPSCORE             0b11111111

#endif