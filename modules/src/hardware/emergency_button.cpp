#include "emergency_button.h"
#include "pin_out_default.h" 

//=====[ Variables internas ]===============================================

static InterruptIn emergencyButton(BUTTON_STOP_PIN);
static volatile bool emergencyState = false; //Puede cambiar por ISR o hardware accesible desde el main().

/// ISR: se ejecuta al presionar el botón (flanco de bajada o subida)
static void emergencyButtongCallback()
{
    emergencyState = ON;
}

void emergencyButtonInit(PinMode mode)
{
    if (mode != PullNone) {
        emergencyButton.mode(mode);
    }

    emergencyButton.fall(&emergencyButtongCallback);  // Detecta flanco descendente (presionado)
    emergencyState = OFF;
}

bool emergencyButtonPressed()
{
    return emergencyState;
}

void emergencyButtonSetState(bool state)
{
    emergencyState = state;
}