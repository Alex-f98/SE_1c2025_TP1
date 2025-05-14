#include "emergency_button.h"
#include "pin_out_default.h"  // Asegúrate que BUTTON1 está definido ahí

//=====[ Variables internas ]===============================================

static InterruptIn emergencyButton(BUTTON_STOP_PIN);
static volatile bool emergencyState = false; //Puede cambiar por ISR o hardware accesible desde el main().

/// ISR: se ejecuta al presionar el botón (flanco de bajada o subida)
static void emergencyButtongCallback()
{
    emergencyState = ON;
}

void emergencyButtonInit()
{
    emergencyButton.fall(&emergencyButtongCallback);  // Detecta flanco descendente (presionado)
    //emergencyButton.mode(PullDown)                  // Es el de la placa.
    emergencyState = OFF;
}

bool emergencyButtonPressed()
{
    return emergencyState;
}
