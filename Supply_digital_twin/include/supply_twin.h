#ifndef SUPPLY_TWIN
#define SUPPLY_TWIN

// #define PI 3.14159265358979323846f
#define FREQ 50.0f          // Частота 50 Гц
#define RESISTANCE 0.1f     // Внутреннее сопротивление источника (в Омах)
#define AMPLITUDE 220.0f     // Амплитуда напряжения (например, 220 В)
#define TIME_STEP 0.001f     // Шаг времени (в секундах)

float calculate_voltage(float current, float time);

#endif 