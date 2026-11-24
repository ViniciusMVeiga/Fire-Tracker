/**
 * @file calibrate_servo.cpp
 * @brief Ferramenta de calibração manual dos servomotores.
 *
 * Uso:
 *   sudo ./calibrate_servo <gpio> <pulso_ms>
 *
 * Exemplos:
 *   sudo ./calibrate_servo 12 1.5   → envia 1.5 ms ao GPIO 12 (pan)
 *   sudo ./calibrate_servo 13 1.5   → envia 1.5 ms ao GPIO 13 (tilt)
 *   sudo ./calibrate_servo 12 1.0   → testa 1.0 ms no pan
 *   sudo ./calibrate_servo 13 1.2   → testa 1.2 ms no tilt
 *
 * O sinal é mantido por 5 segundos para que você observe a posição do servo.
 * Use CTRL+C para encerrar antes.
 *
 * Compile com:
 *   g++ -std=c++17 -O2 tools/calibrate_servo.cpp -o calibrate_servo -lpigpio -lrt -lpthread
 */

#include <iostream>
#include <cstdlib>   // atoi, atof
#include <cmath>     // round
#include <thread>
#include <chrono>
#include <pigpio.h>

int main(int argc, char* argv[])
{
    if (argc != 3) {
        std::cerr << "Uso: sudo ./calibrate_servo <gpio> <pulso_ms>\n"
                  << "Exemplo: sudo ./calibrate_servo 12 1.5\n";
        return 1;
    }

    const unsigned int gpio    = static_cast<unsigned int>(std::atoi(argv[1]));
    const double       pulseMs = std::atof(argv[2]);

    // Valida os limites
    if (pulseMs < 0.5 || pulseMs > 2.5) {
        std::cerr << "Erro: pulso deve estar entre 0.5 ms e 2.5 ms.\n";
        return 1;
    }

    const unsigned int pulseUs = static_cast<unsigned int>(std::round(pulseMs * 1000.0));

    // Inicializa pigpio
    const int version = gpioInitialise();
    if (version < 0) {
        std::cerr << "Erro: gpioInitialise() falhou. Execute com sudo.\n";
        return 1;
    }

    std::cout << "pigpio v" << version << " inicializado.\n"
              << "Enviando " << pulseMs << " ms (" << pulseUs << " µs) "
              << "ao GPIO " << gpio << " por 5 segundos...\n";

    gpioSetMode(gpio, PI_OUTPUT);
    gpioServo(gpio, pulseUs);

    // Mantém o sinal por 5 segundos
    std::this_thread::sleep_for(std::chrono::seconds(5));

    // Encerra sem mover o servo (mantém posição)
    gpioTerminate();

    std::cout << "Encerrado. O servo ficou em " << pulseMs << " ms.\n";
    return 0;
}
