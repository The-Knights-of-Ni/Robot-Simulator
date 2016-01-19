#ifndef BUTTON
#define BUTTON

#define offset 40

enum Buttons
{
    RIGHT_BUMPER,
    LEFT_BUMPER,
    BACK,
    START,
    LOGITECH,
    Y,
    X,
    B,
    A,
    DPAD_RIGHT,
    DPAD_LEFT,
    DPAD_DOWN,
    DPAD_UP,
    RIGHT_STICK_BUTTON,
    LEFT_STICK_BUTTON,
    SIZEOF_BUTTON
};

struct Button
{
    int currentByte;
    int previousByte;
    //int prevStatus = 0;
    int toggleByte;
    
    static bool getState(int btn, int allBits)
    {
        if((allBits >> (btn)) & 1)
            return 1;
        else
            return 0;
    }
    
    bool press(int btn)
    {
        return getState(btn, currentByte);
    }
    
    bool singlePress(int btn)
    {
        return getState(btn, currentByte) && !getState(btn, previousByte);
    }
    
    bool toggle(int btn)
    {
        return getState(btn, toggleByte);
    }
    
    void updateButtons(int stickArray)
    {
        previousByte = currentByte;
        currentByte = stickArray;
        toggleByte = toggleByte^(currentByte & ~previousByte);
    }
};

#pragma pack(push, 4)
struct gamepad
{
    v2f joystick1;
    v2f joystick2;

    float left_trigger;
    float right_trigger;
    
    int buttons;
};
#pragma pack(pop)

#endif
