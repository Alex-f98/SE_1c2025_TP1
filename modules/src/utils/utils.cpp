#include "mbed.h"
#include "utils.h"
#include "string.h"
#include "pin_out_default.h"

/// @brief Comunicación serie con la PC.
UnbufferedSerial uartUsb(USBTX_PIN, USBRX_PIN, USB_BAUD_RATE); ///< UART para comunicación USB.

void _printUart(const char* msg) {
    uartUsb.write(msg, strlen(msg));
}

bool isReadableUart(){
    return uartUsb.readable();
}

void readUartCommand(char *command){
    uartUsb.read(command, 1);
}



void availableCommands()
{
    /*
    Basado en el siguiente caso ejemplo
    potentiometerReading = potentiometer.read();
    sprintf ( str, "Potentiometer: %.2f\r\n", potentiometerReading );
    stringLength = strlen(str);
    uartUsb.write( str, stringLength );
    */
    _printUart("Available commands:\r\n");
    _printUart("Press 'w' to increase speed\r\n");
    _printUart("Press 's' to decrease speed\r\n");
    _printUart("Press 'd' to decrease rotation\r\n");
    _printUart("Press 'a' to increase rotation\r\n");
    _printUart("Press 'q' to stop\r\n");
    _printUart("Press 'c' to change mode close loop\r\n");
}

float _min(float a, float b) {
    return (a < b) ? a : b;
}

// Función para calcular el máximo entre dos valores
float _max(float a, float b) {
    return (a > b) ? a : b;
}

