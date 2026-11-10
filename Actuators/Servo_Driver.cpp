/**
 * @file Servo_Driver.cpp
 * @brief Implementação do driver de servos via pigpio.
 *
 * pigpio API utilizada:
 *   gpioInitialise()          — inicializa a biblioteca
 *   gpioTerminate()           — libera recursos
 *   gpioSetMode(pin, PI_OUTPUT) — configura pino como saída
 *   gpioServo(pin, pulseUs)   — envia sinal de servo (500–2500 µs, 50 Hz)
 *
 * gpioServo() gera automaticamente o sinal PWM com período fixo de 20 ms
 * (50 Hz) e largura de pulso em µs. Valor 0 desativa o sinal.
 *
 * Referência: http://abyz.me.uk/rpi/pigpio/cif.html#gpioServo
 *
 * NOTA SOBRE COMPILAÇÃO CONDICIONAL:
 *   Quando compilado com -DHARDWARE_MOCK (modo de desenvolvimento sem RPi),
 *   todas as chamadas pigpio são substituídas por stubs que apenas registram
 *   o comando. Isso permite compilar e testar no PC de desenvolvimento.
 */

#include "Servo_Driver.hpp"

#include <iostream>
#include <stdexcept>
#include <cmath>

// Inclusão condicional da biblioteca pigpio
// No Raspberry Pi: instalar com  sudo apt install libpigpio-dev pigpio
// Na máquina de desenvolvimento: compilar com -DHARDWARE_MOCK
#ifndef HARDWARE_MOCK
#  include <pigpio.h>
#else
// ─── Stubs para compilação sem pigpio ────────────────────────────────────────
namespace {
    constexpr int PI_OUTPUT   = 1;
    constexpr int PI_BAD_GPIO = -143;

    inline int  gpioInitialise()               { return 0; }  // version
    inline void gpioTerminate()                {}
    inline int  gpioSetMode(unsigned, unsigned){ return 0; }
    inline int  gpioServo(unsigned g, unsigned w) {
        std::cout << "[MOCK] gpioServo(gpio=" << g << ", pulseUs=" << w << ")\n";
        return 0;
    }
}
// ─────────────────────────────────────────────────────────────────────────────
#endif

namespace Actuators {

// ─────────────────────────────────────────────────────────────────────────────
// Constantes internas
// ─────────────────────────────────────────────────────────────────────────────

/// Valor de pulso 0 µs instrui o pigpio a parar de emitir o sinal PWM.
static constexpr unsigned int PIGPIO_STOP_PULSE = 0u;

// ─────────────────────────────────────────────────────────────────────────────
// Construtor / Destrutor
// ─────────────────────────────────────────────────────────────────────────────

Servo_Driver::Servo_Driver(Servo_Params params)
    : mParams(std::move(params))
{
    if (mParams.absMinMs >= mParams.absMaxMs) {
        throw std::invalid_argument(
            "Servo_Driver: absMinMs deve ser menor que absMaxMs.");
    }
}

Servo_Driver::~Servo_Driver()
{
    if (mInitialized) {
        shutdown();
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Inicialização
// ─────────────────────────────────────────────────────────────────────────────

bool Servo_Driver::initialize()
{
    if (mInitialized) {
        std::cerr << "[Servo_Driver] Já inicializado.\n";
        return true;
    }

    // Inicializa pigpio (ou stub em modo mock)
    const int initResult = gpioInitialise();
    if (initResult < 0) {
        std::cerr << "[Servo_Driver] gpioInitialise() falhou (código "
                  << initResult << ").\n"
                  << "  Verifique se o daemon pigpiod está ativo "
                     "(sudo pigpiod) e se o programa tem permissão.\n";
        return false;
    }

    std::cout << "[Servo_Driver] pigpio inicializado (versão " << initResult << ").\n";

    // Configura pinos como saída
    if (gpioSetMode(mParams.panGpio, static_cast<unsigned>(PI_OUTPUT)) != 0) {
        std::cerr << "[Servo_Driver] gpioSetMode falhou no pino pan (GPIO "
                  << mParams.panGpio << ").\n";
        gpioTerminate();
        return false;
    }

    if (gpioSetMode(mParams.tiltGpio, static_cast<unsigned>(PI_OUTPUT)) != 0) {
        std::cerr << "[Servo_Driver] gpioSetMode falhou no pino tilt (GPIO "
                  << mParams.tiltGpio << ").\n";
        gpioTerminate();
        return false;
    }

    mInitialized = true;

    // Aplica pulso seguro (posição neutra) em ambos os servos
    const bool okPan  = set_pan_pulse(mParams.safePulseMs);
    const bool okTilt = set_tilt_pulse(mParams.safePulseMs);

    if (!okPan || !okTilt) {
        std::cerr << "[Servo_Driver] Falha ao aplicar pulso seguro inicial.\n";
        shutdown();
        return false;
    }

    std::cout << "[Servo_Driver] Servos inicializados.\n"
              << "  Pan  → GPIO " << mParams.panGpio
              << ", pulso seguro " << mParams.safePulseMs << " ms\n"
              << "  Tilt → GPIO " << mParams.tiltGpio
              << ", pulso seguro " << mParams.safePulseMs << " ms\n";

    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Acionamento dos servos
// ─────────────────────────────────────────────────────────────────────────────

bool Servo_Driver::set_pan_pulse(double pulseMs)
{
    if (!mInitialized) {
        std::cerr << "[Servo_Driver] set_pan_pulse: driver não inicializado.\n";
        return false;
    }

    unsigned int pulseUs = 0;
    if (!validate_and_convert(pulseMs, pulseUs)) {
        return false;
    }

    if (!send_pulse(mParams.panGpio, pulseUs)) {
        return false;
    }

    mPanPulseMs = pulseMs;
    return true;
}

bool Servo_Driver::set_tilt_pulse(double pulseMs)
{
    if (!mInitialized) {
        std::cerr << "[Servo_Driver] set_tilt_pulse: driver não inicializado.\n";
        return false;
    }

    unsigned int pulseUs = 0;
    if (!validate_and_convert(pulseMs, pulseUs)) {
        return false;
    }

    if (!send_pulse(mParams.tiltGpio, pulseUs)) {
        return false;
    }

    mTiltPulseMs = pulseMs;
    return true;
}

bool Servo_Driver::set_both_pulses(double panMs, double tiltMs)
{
    const bool okPan  = set_pan_pulse(panMs);
    const bool okTilt = set_tilt_pulse(tiltMs);
    return okPan && okTilt;
}

// ─────────────────────────────────────────────────────────────────────────────
// Encerramento
// ─────────────────────────────────────────────────────────────────────────────

void Servo_Driver::shutdown()
{
    if (!mInitialized) {
        return;
    }

    // Aplica pulso seguro antes de parar
    unsigned int safeUs = 0;
    if (validate_and_convert(mParams.safePulseMs, safeUs)) {
        send_pulse(mParams.panGpio,  safeUs);
        send_pulse(mParams.tiltGpio, safeUs);
    }

    // Para os sinais PWM
    gpioServo(mParams.panGpio,  PIGPIO_STOP_PULSE);
    gpioServo(mParams.tiltGpio, PIGPIO_STOP_PULSE);

    gpioTerminate();

    mInitialized = false;
    std::cout << "[Servo_Driver] pigpio encerrado. Servos em posição segura.\n";
}

// ─────────────────────────────────────────────────────────────────────────────
// Consultas de estado
// ─────────────────────────────────────────────────────────────────────────────

bool Servo_Driver::is_initialized() const
{
    return mInitialized;
}

double Servo_Driver::get_pan_pulse_ms() const
{
    return mPanPulseMs;
}

double Servo_Driver::get_tilt_pulse_ms() const
{
    return mTiltPulseMs;
}

// ─────────────────────────────────────────────────────────────────────────────
// Funções privadas
// ─────────────────────────────────────────────────────────────────────────────

bool Servo_Driver::validate_and_convert(double pulseMs,
                                         unsigned int& pulseUs) const
{
    if (pulseMs < mParams.absMinMs || pulseMs > mParams.absMaxMs) {
        std::cerr << "[Servo_Driver] Pulso fora dos limites: " << pulseMs
                  << " ms (limites: [" << mParams.absMinMs << ", "
                  << mParams.absMaxMs << "] ms).\n";
        return false;
    }

    // pigpio trabalha em µs (inteiro) com range 500–2500 µs para gpioServo()
    // Nota: gpioServo aceita 500–2500 µs; nossos limites padrão (0.8–2.2 ms)
    // estão dentro desse intervalo.
    pulseUs = static_cast<unsigned int>(std::round(pulseMs * 1000.0));
    return true;
}

bool Servo_Driver::send_pulse(unsigned int gpio, unsigned int pulseUs)
{
    // gpioServo() retorna 0 em sucesso, código de erro negativo em falha.
    const int result = gpioServo(gpio, pulseUs);

    if (result != 0) {
        std::cerr << "[Servo_Driver] gpioServo(gpio=" << gpio
                  << ", pulse=" << pulseUs << " µs) falhou (código "
                  << result << ").\n";
        return false;
    }

    return true;
}

} // namespace Actuators
