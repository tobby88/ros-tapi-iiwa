#ifndef OPENIGTLSTATE_H
#define OPENIGTLSTATE_H

// important and used states for the OpenIGTLink-Communication @see rosopenigtlbridge.h
enum OPENIGTL_STATE
{
    NO_STATE = -1,
    IDLE, //Idle state
    FREE, //Gravitation compensation
    MOVE_TO_POSE
};

#endif // OPENIGTLSTATE_H
