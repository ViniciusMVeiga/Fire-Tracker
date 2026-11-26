/**
 * @file Laser_Driver.cpp
 * @brief Implementação do driver do módulo laser KY-008 via pigpio.
 *
 * pigpio API utilizada:
 *   gpioSetMode(pin, PI_OUTPUT) — configura pino como saída digital
 *   gpioWrite(pin, level)       — escreve nível lógico (0 ou 1) no pino
 *
 * NOTA: gpioInitialise() e gpioTerminate() são gerenciados pelo Servo_Driver.
 * Este módulo pressupõe que pigpio já foi inicializado antes de initialize()
 * ser chamado.
 *
 * Referência: http://abyz.me.uk/rpi/pigpio/cif.html#gpioWrite
 *
 * NOTA SOBRE COMPILAÇÃO CONDICIONAL:
 *   Com -DHARDWARE_MOCK, as chamadas pigpio são substituídas por stubs que
 *   apenas imprimem o comando no terminal.
 */

#include "Laser_Driver.hpp"

#include <iostream>

// Inclusão condicional da biblioteca pigpio
#ifndef HARDWARE_MOCK
#  include <pigpio.h>
#else
// ─── Stubs para compilação sem pigpio ────────────────────────────────────────
namespace {
    constexpr int PI_OUTPUT = 1;

    inline int  gpioSetMode(unsigned g, unsigned m) {
        (void)g; (void)m;
        return 0;
    }
    inline int  gpioWrite(unsigned g, unsigned level) {
        std::cout << "[MOCK] gpioWrite(gpio=" << g
                  << ", level=" << level << ")\n";
        return 0;
    }
}
// ─────────────────────────────────────────────────────────────────────────────
#endif

namespace Actuators {

Laser_Driver::Laser_Driver(unsigned int gpioPin)
    : mGpioPin(gpioPin)
{}

Laser_Driver::~Laser_Driver()
{
    if (mInitialized) {
        shutdown();
    }
}

bool Laser_Driver::initialize()
{
    if (mInitialized) {
        std::cerr << "[Laser_Driver] Já inicializado.\n";
        return true;
    }

    // Configura o pino como saída digital
    const int result = gpioSetMode(mGpioPin, static_cast<unsigned>(PI_OUTPUT));
    if (result != 0) {
        std::cerr << "[Laser_Driver] gpioSetMode falhou no pino GPIO "
                  << mGpioPin << " (código " << result << ").\n"
                  << "  Verifique se pigpio foi inicializado pelo Servo_Driver.\n";
        return false;
    }

    // Garante estado inicial desligado
    if (!write_gpio(0)) {
        return false;
    }

    mInitialized = true;
    mOn          = false;

    std::cout << "[Laser_Driver] Laser inicializado no GPIO "
              << mGpioPin << " (pino físico 37). Estado: DESLIGADO.\n";
    return true;
}

bool Laser_Driver::turn_on()
{
    if (!mInitialized) {
        std::cerr << "[Laser_Driver] turn_on: driver não inicializado.\n";
        return false;
    }
    if (mOn) {
        return true;  // já está ligado — nenhuma escrita necessária
    }

    if (!write_gpio(1)) {
        return false;
    }

    mOn = true;
    return true;
}

bool Laser_Driver::turn_off()
{
    if (!mInitialized) {
        std::cerr << "[Laser_Driver] turn_off: driver não inicializado.\n";
        return false;
    }
    if (!mOn) {
        return true;  // já está desligado
    }

    if (!write_gpio(0)) {
        return false;
    }

    mOn = false;
    return true;
}

void Laser_Driver::shutdown()
{
    if (!mInitialized) {
        return;
    }

    // Garante que o laser está desligado antes de encerrar
    write_gpio(0);
    mOn          = false;
    mInitialized = false;

    std::cout << "[Laser_Driver] Laser desligado e driver encerrado.\n";
}

bool Laser_Driver::is_initialized() const
{
    return mInitialized;
}

bool Laser_Driver::is_on() const
{
    return mOn;
}

// ─────────────────────────────────────────────────────────────────────────────
// Função privada
// ─────────────────────────────────────────────────────────────────────────────

bool Laser_Driver::write_gpio(unsigned int level)
{
    const int result = gpioWrite(mGpioPin, level);
    if (result != 0) {
        std::cerr << "[Laser_Driver] gpioWrite(gpio=" << mGpioPin
                  << ", level=" << level << ") falhou (código "
                  << result << ").\n";
        return false;
    }
    return true;
}

} // namespace Actuators
