#ifndef EMERGENCY_BUTTON_H
#define EMERGENCY_BUTTON_H

#include "mbed.h"
#include "arm_book_lib.h"

/// Inicializa el botón de emergencia con interrupción.
void emergencyButtonInit(PinMode mode = PullNone);

/// Indica si se activó el estado de emergencia.
bool emergencyButtonPressed();

/// Permite cambiar el estado de forma manual.
void emergencyButtonSetState(bool state);

#endif // EMERGENCY_BUTTON_H
