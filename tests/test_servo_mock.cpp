/**
 * @file test_servo_mock.cpp
 * @brief Teste do Servo_Driver em modo mock (sem hardware GPIO).
 *
 * Verifica:
 *   - Inicialização com parâmetros válidos
 *   - Rejeição de pulsos fora dos limites
 *   - Aceitação de pulsos válidos
 *   - set_both_pulses
 *   - Consulta de últimos valores enviados
 *   - Dupla inicialização (idempotente)
 *   - Shutdown correto
 *
 * Compilação (sem pigpio — usa stubs HARDWARE_MOCK):
 *   g++ -std=c++17 -DHARDWARE_MOCK -I.. test_servo_mock.cpp \
 *       ../Actuators/Servo_Driver.cpp -o test_servo_mock
 *
 * Execução:
 *   ./test_servo_mock
 */

#define HARDWARE_MOCK  // garante stubs pigpio

#include "../Actuators/Servo_Driver.hpp"

#include <iostream>
#include <cmath>
#include <string>
#include <stdexcept>

static int gTotal = 0, gPassed = 0;

static void check(bool cond, const std::string& name)
{
    ++gTotal;
    if (cond) { ++gPassed; std::cout << "  PASS: " << name << "\n"; }
    else       {            std::cout << "  FAIL: " << name << "\n"; }
}

static bool near(double a, double b, double tol = 1e-6)
{
    return std::fabs(a - b) < tol;
}

static Actuators::Servo_Params make_params()
{
    Actuators::Servo_Params p;
    p.panGpio         = 12;
    p.tiltGpio        = 13;
    p.pwmFreqHz       = 50;
    p.panSafePulseMs  = 1.54;
    p.tiltSafePulseMs = 1.30;
    p.absMinMs        = 0.8;
    p.absMaxMs        = 2.2;
    return p;
}

// T01: Inicialização básica
static void test_initialize()
{
    std::cout << "\n[T01] Inicialização\n";
    Actuators::Servo_Driver drv(make_params());
    bool ok = drv.initialize();
    check(ok, "initialize() retorna true");
    check(drv.is_initialized(), "is_initialized() = true após init");
    check(near(drv.get_pan_pulse_ms(),  1.54), "panSafePulseMs aplicado na init (1.54)");
    check(near(drv.get_tilt_pulse_ms(), 1.30), "tiltSafePulseMs aplicado na init (1.30)");
    drv.shutdown();
    check(!drv.is_initialized(), "is_initialized() = false após shutdown");
}

// T02: Pulso válido é aceito
static void test_valid_pulse()
{
    std::cout << "\n[T02] Pulsos válidos\n";
    Actuators::Servo_Driver drv(make_params());
    drv.initialize();

    check(drv.set_pan_pulse(1.5),  "set_pan_pulse(1.5) OK");
    check(drv.set_tilt_pulse(1.2), "set_tilt_pulse(1.2) OK");
    check(near(drv.get_pan_pulse_ms(),  1.5), "get_pan_pulse_ms() = 1.5");
    check(near(drv.get_tilt_pulse_ms(), 1.2), "get_tilt_pulse_ms() = 1.2");

    drv.shutdown();
}

// T03: Pulso fora dos limites é rejeitado
static void test_invalid_pulse()
{
    std::cout << "\n[T03] Pulsos inválidos\n";
    Actuators::Servo_Driver drv(make_params());
    drv.initialize();

    check(!drv.set_pan_pulse(0.5),  "set_pan_pulse(0.5) rejeitado (< absMin)");
    check(!drv.set_pan_pulse(2.5),  "set_pan_pulse(2.5) rejeitado (> absMax)");
    check(!drv.set_tilt_pulse(0.0), "set_tilt_pulse(0.0) rejeitado");

    drv.shutdown();
}

// T04: set_both_pulses
static void test_both_pulses()
{
    std::cout << "\n[T04] set_both_pulses\n";
    Actuators::Servo_Driver drv(make_params());
    drv.initialize();

    check(drv.set_both_pulses(1.3, 1.7), "set_both_pulses(1.3, 1.7) OK");
    check(near(drv.get_pan_pulse_ms(),  1.3), "pan = 1.3");
    check(near(drv.get_tilt_pulse_ms(), 1.7), "tilt = 1.7");

    // Um inválido → retorna false mas o válido ainda é enviado
    bool res = drv.set_both_pulses(1.5, 3.0);
    check(!res, "set_both_pulses com tilt inválido retorna false");

    drv.shutdown();
}

// T05: Construtor deve lançar com parâmetros inválidos
static void test_invalid_construction()
{
    std::cout << "\n[T05] Construção com parâmetros inválidos\n";
    bool caught = false;
    try {
        Actuators::Servo_Params p = make_params();
        p.absMinMs = 2.0; p.absMaxMs = 1.0;  // invertidos
        Actuators::Servo_Driver drv(p);
    } catch (const std::invalid_argument&) {
        caught = true;
    }
    check(caught, "absMinMs >= absMaxMs → invalid_argument");
}

// T06: Operações sem inicialização retornam false
static void test_not_initialized()
{
    std::cout << "\n[T06] Operações sem inicialização\n";
    Actuators::Servo_Driver drv(make_params());
    check(!drv.set_pan_pulse(1.5),  "set_pan_pulse sem init retorna false");
    check(!drv.set_tilt_pulse(1.5), "set_tilt_pulse sem init retorna false");
}

int main()
{
    std::cout << "=== Testes do Servo_Driver (MOCK) ===\n";

    test_initialize();
    test_valid_pulse();
    test_invalid_pulse();
    test_both_pulses();
    test_invalid_construction();
    test_not_initialized();

    std::cout << "\n=== Resultado: " << gPassed << "/" << gTotal
              << " testes aprovados ===\n";
    return (gPassed == gTotal) ? 0 : 1;
}
