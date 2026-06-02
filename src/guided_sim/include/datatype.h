#ifndef __datatype_h_
#define __datatype_h_


enum {
    LOCAL_FRAME         = 1,
    BODY_FRAME          = 8,
};

enum fly_status{
    MODE_POSITION       = 0,    
    MODE_VELOCITY       = 1,
    MODE_HOVER          = 2,
    MODE_APRIALTAG      = 3,
    MODE_CIRCLE         = 4,
};

enum {
    KEY_W           = 119,
    KEY_A           = 97,
    KEY_S           = 115,
    KEY_D           = 100,

    KEY_I           = 105,
    KEY_J           = 106,
    KEY_K           = 107,
    KEY_L           = 108,

    KEY_M           = 109,
    KEY_X           = 120,

    KEY_V           = 118,
    KEY_P           = 112,

    KEY_NUM_1       = 49,

    KEY_ENTER       = 13,
    KEY_SPACE       = 32,
    KEY_ESC         = 27,
    KEY_BACKSPACE   = 127,

};



#endif