// debug_utils.h
#ifndef UTILS_H
#define UTILS_H


float _min(float a, float b);

// Función para calcular el máximo entre dos valores
float _max(float a, float b);

/// @brief Envia mensajes por UART.
void _printUart(const char* msg);

bool isReadableUart();

void readUartCommand(char *command);

/// @brief Lista de comandos disponibles para el usuario.
void availableCommands();

#endif