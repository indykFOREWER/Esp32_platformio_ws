#include <stdio.h>
#include <math.h>
#include "supply_twin.h"


// Функция для расчета мгновенного напряжения на выходе источника
float calculate_voltage(float current, float time) {
    // Мгновенное значение напряжения источника без учета сопротивления
    // float source_voltage = AMPLITUDE * sinf(2.0f * PI * FREQ * time);
    float source_voltage = 5;

    // Напряжение на выходе с учетом внутреннего сопротивления
    float output_voltage = source_voltage - RESISTANCE * current;
    if (output_voltage < 0) return 0;
    
    return output_voltage;
}

// int main() {
//     float time = 0.0f;  // Начальное время
//     float current = 0.0f;  // Входной ток (может изменяться по вашему выбору)

//     // Симуляция на протяжении 1 секунды
//     for (int i = 0; i < 1000; i++) {
//         float voltage = calculate_voltage(current, time);

//         // Вывод времени, тока и напряжения
//         printf("Time: %.3f s, Current: %.2f A
