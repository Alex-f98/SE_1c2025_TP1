#ifndef EMERGENCY_BUTTON_H
#define EMERGENCY_BUTTON_H

#include "mbed.h"
#include "arm_book_lib.h"

/// Inicializa el botón de emergencia con interrupción.
void emergencyButtonInit();

/// Indica si se activó el estado de emergencia.
bool emergencyButtonPressed();

#endif // EMERGENCY_BUTTON_H
