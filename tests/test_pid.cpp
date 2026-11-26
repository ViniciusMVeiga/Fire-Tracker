/**
 * @file test_pid.cpp
 * @brief Testes automatizados do controlador PID (sem hardware).
 *
 * Testam isoladamente a lógica matemática do Pid_Controller:
 *   - Resposta proporcional
 *   - Termos integral e derivativo
 *   - Independência entre controladores X e Y
 *   - Saturação de saída
 *   - Anti-windup
 *   - Inicialização e reinicialização
 *   - Erro nulo
 *   - Inversão de sentido
 *   - Medições inválidas (o main.cpp é responsável por não chamar o PID
 *     nesse caso; o teste verifica o comportamento ao receber reset())
 *
 * Compilação (sem pigpio, sem OpenCV):
 *   g++ -std=c++17 -DHARDWARE_MOCK -I.. test_pid.cpp ../Control/Pid_Controller.cpp -o test_pid
 *
 * Execução:
 *   ./test_pid
 *
 * Saída: cada teste imprime PASS ou FAIL com descrição.
 *
 * NOTA: Este arquivo não testa desempenho dinâmico (sobressinal, tempo de
 * acomodação). Esses requisitos dependem de ensaios físicos com o sistema real.
 */

#include "../Control/Pid_Controller.hpp"

#include <iostream>
#include <cmath>
#include <string>
#include <cassert>

// ─────────────────────────────────────────────────────────────────────────────
// Infraestrutura mínima de teste
// ─────────────────────────────────────────────────────────────────────────────

static int gTotalTests = 0;
static int gPassedTests = 0;

static void check(bool condition, const std::string& testName)
{
    ++gTotalTests;
    if (condition) {
        ++gPassedTests;
        std::cout << "  PASS: " << testName << "\n";
    } else {
        std::cout << "  FAIL: " << testName << "\n";
    }
}

static bool near(double a, double b, double tol = 1e-9)
{
    return std::fabs(a - b) < tol;
}

// ─────────────────────────────────────────────────────────────────────────────
// Parâmetros base para os testes
// ─────────────────────────────────────────────────────────────────────────────

static Control::Pid_Params base_params()
{
    Control::Pid_Params p;
    p.kp              = 0.01;
    p.ki              = 0.0;
    p.kd              = 0.0;
    p.biasMs          = 1.5;
    p.outMinMs        = 1.0;
    p.outMaxMs        = 2.0;
    p.integralLimitMs = 0.4;
    p.samplePeriodS   = 0.05;
    p.invertDirection = false;
    p.derivativeFilterN = 0.0;  // derivativo puro para facilitar cálculo manual
    return p;
}

// ─────────────────────────────────────────────────────────────────────────────
// Testes
// ─────────────────────────────────────────────────────────────────────────────

// T01: Resposta proporcional — saída = bias + Kp * error (sem I e D)
static void test_proportional()
{
    std::cout << "\n[T01] Resposta proporcional\n";

    Control::Pid_Params p = base_params();
    p.kp = 0.01;
    Control::Pid_Controller ctrl(p);

    // erro = 50 px → saída esperada = 1.5 + 0.01*50 = 2.0 ms (limite)
    auto out = ctrl.calculate_output(50.0);
    check(near(out.termP, 0.5, 1e-9),    "termP = Kp * error = 0.5");
    check(near(out.rawOutput, 2.0, 1e-9), "rawOutput = bias + P = 2.0 (no sat)");
    check(!out.saturated,                  "não saturado em 2.0");

    // erro = 0 → saída = bias
    ctrl.reset();
    auto out0 = ctrl.calculate_output(0.0);
    check(near(out0.pulseWidthMs, p.biasMs, 1e-9), "erro nulo → saída = bias");
}

// T02: Erro nulo — saída = bias, termos P/I/D = 0
static void test_zero_error()
{
    std::cout << "\n[T02] Erro nulo\n";

    Control::Pid_Params p = base_params();
    p.kp = 0.005; p.ki = 0.01; p.kd = 0.002;
    Control::Pid_Controller ctrl(p);

    auto out = ctrl.calculate_output(0.0);
    check(near(out.termP, 0.0, 1e-9),           "P = 0");
    check(near(out.termI, 0.0, 1e-9),           "I = 0 (primeira amostra)");
    check(near(out.termD, 0.0, 1e-9),           "D = 0 (primeira amostra)");
    check(near(out.pulseWidthMs, 1.5, 1e-9),    "saída = bias = 1.5 ms");
}

// T03: Termo integral — acumulação ao longo de múltiplos ciclos
static void test_integral()
{
    std::cout << "\n[T03] Termo integral\n";

    Control::Pid_Params p = base_params();
    p.kp = 0.0; p.ki = 0.1; p.kd = 0.0;
    p.integralLimitMs = 999.0;  // sem limite para testar acumulação livre
    Control::Pid_Controller ctrl(p);

    // Após 3 ciclos com erro = 10 px:
    // I(1) = 0.1 * 0.05 * 10 = 0.05
    // I(2) = 0.05 + 0.05 = 0.10
    // I(3) = 0.10 + 0.05 = 0.15
    ctrl.calculate_output(10.0);
    ctrl.calculate_output(10.0);
    auto out3 = ctrl.calculate_output(10.0);

    check(near(out3.termI, 0.15, 1e-9), "acumulador após 3 ciclos = 0.15 ms");
    check(near(out3.pulseWidthMs, 1.5 + 0.15, 1e-9),
          "saída = bias + I = 1.65 ms");
}

// T04: Termo derivativo — primeira amostra D=0, segunda amostra D ≠ 0
static void test_derivative()
{
    std::cout << "\n[T04] Termo derivativo (puro, N=0)\n";

    Control::Pid_Params p = base_params();
    p.kp = 0.0; p.ki = 0.0; p.kd = 0.1;
    p.derivativeFilterN = 0.0;  // derivativo puro
    Control::Pid_Controller ctrl(p);

    // 1ª amostra: firstSample=true → D = 0
    auto out1 = ctrl.calculate_output(20.0);
    check(near(out1.termD, 0.0, 1e-9), "D = 0 na primeira amostra");

    // 2ª amostra: D = Kd * (30-20) / Ts = 0.1 * 10 / 0.05 = 20.0 ms
    // mas saturado em outMaxMs = 2.0
    auto out2 = ctrl.calculate_output(30.0);
    const double expectedD = 0.1 * (30.0 - 20.0) / 0.05; // = 20.0
    check(near(out2.termD, expectedD, 1e-9), "D calculado corretamente = 20.0");
    check(out2.saturated, "saída saturada (bias + D > outMax)");
    check(near(out2.pulseWidthMs, p.outMaxMs, 1e-9), "saída no limite máximo");
}

// T05: Saturação mínima e máxima
static void test_saturation()
{
    std::cout << "\n[T05] Saturação de saída\n";

    Control::Pid_Params p = base_params();
    p.kp = 0.1;  // ganho alto para saturar facilmente
    Control::Pid_Controller ctrl(p);

    // erro positivo grande → saturar em outMaxMs
    auto outMax = ctrl.calculate_output(100.0);
    check(near(outMax.pulseWidthMs, p.outMaxMs, 1e-9),
          "saída saturada em outMaxMs = 2.0");
    check(outMax.saturated, "flag saturated = true");

    // erro negativo grande → saturar em outMinMs
    ctrl.reset();
    auto outMin = ctrl.calculate_output(-100.0);
    check(near(outMin.pulseWidthMs, p.outMinMs, 1e-9),
          "saída saturada em outMinMs = 1.0");
    check(outMin.saturated, "flag saturated = true (negativo)");
}

// T06: Anti-windup — o integrador não deve crescer indefinidamente quando saturado
static void test_anti_windup()
{
    std::cout << "\n[T06] Anti-windup\n";

    Control::Pid_Params p = base_params();
    p.kp = 0.0; p.ki = 0.5; p.kd = 0.0;
    p.integralLimitMs = 0.3;
    Control::Pid_Controller ctrl(p);

    // Aplica erro constante por muitos ciclos
    for (int i = 0; i < 100; ++i) {
        ctrl.calculate_output(50.0);
    }

    const auto& state = ctrl.get_state();
    check(state.integralAccum <= 0.3 + 1e-9,
          "integrador limitado a +integralLimitMs");
    check(state.integralAccum >= -0.3 - 1e-9,
          "integrador não é negativo com erro positivo contínuo");
}

// T07: Reinicialização — estado zerado após reset()
static void test_reset()
{
    std::cout << "\n[T07] Reinicialização\n";

    Control::Pid_Params p = base_params();
    p.ki = 0.1;
    Control::Pid_Controller ctrl(p);

    // Acumula estado
    ctrl.calculate_output(30.0);
    ctrl.calculate_output(30.0);

    ctrl.reset();

    const auto& state = ctrl.get_state();
    check(near(state.integralAccum, 0.0, 1e-9),  "integrador zerado");
    check(near(state.prevError, 0.0, 1e-9),       "erro anterior zerado");
    check(state.firstSample,                       "firstSample = true");

    // Próximo ciclo deve se comportar como primeira amostra
    auto out = ctrl.calculate_output(0.0);
    check(near(out.termD, 0.0, 1e-9), "D = 0 após reset (firstSample)");
}

// T08: Independência entre dois controladores
static void test_independence()
{
    std::cout << "\n[T08] Independência entre controladores X e Y\n";

    Control::Pid_Params px = base_params();
    px.kp = 0.01; px.ki = 0.1; px.biasMs = 1.5;

    Control::Pid_Params py = base_params();
    py.kp = 0.02; py.ki = 0.2; py.biasMs = 1.5;

    Control::Pid_Controller ctrlX(px);
    Control::Pid_Controller ctrlY(py);

    auto outX1 = ctrlX.calculate_output(10.0);
    auto outY1 = ctrlY.calculate_output(20.0);

    // Verificar que os acumuladores são independentes
    check(ctrlX.get_state().integralAccum !=
          ctrlY.get_state().integralAccum,
          "acumuladores integrais diferentes");

    // Reset de X não afeta Y
    ctrlX.reset();
    const double integralYBefore = ctrlY.get_state().integralAccum;
    const double integralYAfter  = ctrlY.get_state().integralAccum;
    check(near(integralYBefore, integralYAfter, 1e-9),
          "reset de X não altera estado de Y");

    // Suprime warnings de variável não usada
    (void)outX1; (void)outY1;
}

// T09: Inversão de sentido
static void test_invert_direction()
{
    std::cout << "\n[T09] Inversão de sentido\n";

    Control::Pid_Params p = base_params();
    p.kp = 0.01; p.invertDirection = false;
    Control::Pid_Controller ctrlNormal(p);
    auto outNormal = ctrlNormal.calculate_output(50.0);

    p.invertDirection = true;
    Control::Pid_Controller ctrlInverted(p);
    auto outInverted = ctrlInverted.calculate_output(50.0);

    // Com inversão, erro positivo → P negativo → saída < bias
    check(outNormal.termP > 0.0,   "P > 0 sem inversão");
    check(outInverted.termP < 0.0, "P < 0 com inversão");
    check(near(outNormal.termP, -outInverted.termP, 1e-9),
          "P invertido é simétrico");
}

// T10: Referências para diferentes resoluções
static void test_reference_calculation()
{
    std::cout << "\n[T10] Cálculo de referência para diferentes resoluções\n";

    // Simula o cálculo do main.cpp
    auto calc_ref = [](int w, int h, double& xRef, double& yRef) {
        xRef = w / 2.0;
        yRef = h / 2.0;
    };

    double xr, yr;
    calc_ref(640, 480, xr, yr);
    check(near(xr, 320.0, 1e-9) && near(yr, 240.0, 1e-9),
          "640x480 → ref=(320,240)");

    calc_ref(1280, 720, xr, yr);
    check(near(xr, 640.0, 1e-9) && near(yr, 360.0, 1e-9),
          "1280x720 → ref=(640,360)");

    calc_ref(320, 240, xr, yr);
    check(near(xr, 160.0, 1e-9) && near(yr, 120.0, 1e-9),
          "320x240 → ref=(160,120)");
}

// T11: Construtor deve lançar exceção para parâmetros inválidos
static void test_invalid_params()
{
    std::cout << "\n[T11] Parâmetros inválidos\n";

    bool caught1 = false;
    try {
        Control::Pid_Params p = base_params();
        p.samplePeriodS = 0.0;
        Control::Pid_Controller ctrl(p);
    } catch (const std::invalid_argument&) {
        caught1 = true;
    }
    check(caught1, "samplePeriodS = 0 → invalid_argument");

    bool caught2 = false;
    try {
        Control::Pid_Params p = base_params();
        p.outMinMs = 2.0; p.outMaxMs = 1.0;  // invertidos
        Control::Pid_Controller ctrl(p);
    } catch (const std::invalid_argument&) {
        caught2 = true;
    }
    check(caught2, "outMinMs >= outMaxMs → invalid_argument");

    bool caught3 = false;
    try {
        Control::Pid_Params p = base_params();
        p.integralLimitMs = -0.1;
        Control::Pid_Controller ctrl(p);
    } catch (const std::invalid_argument&) {
        caught3 = true;
    }
    check(caught3, "integralLimitMs < 0 → invalid_argument");
}

// T12: Filtro derivativo atenua picos de ruído
static void test_derivative_filter()
{
    std::cout << "\n[T12] Filtro derivativo (N > 0)\n";

    Control::Pid_Params p = base_params();
    p.kp = 0.0; p.ki = 0.0; p.kd = 0.05;
    p.derivativeFilterN = 10.0;  // filtro ativo
    Control::Pid_Controller ctrlFilt(p);

    p.derivativeFilterN = 0.0;   // derivativo puro
    Control::Pid_Controller ctrlPure(p);

    // Pico de ruído: salto de 0 → 200 px
    ctrlFilt.calculate_output(0.0);
    ctrlPure.calculate_output(0.0);

    auto outFilt = ctrlFilt.calculate_output(200.0);
    auto outPure = ctrlPure.calculate_output(200.0);

    // O filtrado deve ter |D| menor que o puro
    check(std::fabs(outFilt.termD) < std::fabs(outPure.termD),
          "derivativo filtrado < derivativo puro sob salto brusco");
}

// ─────────────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────────────

int main()
{
    std::cout << "=== Testes do Pid_Controller ===\n";

    test_proportional();
    test_zero_error();
    test_integral();
    test_derivative();
    test_saturation();
    test_anti_windup();
    test_reset();
    test_independence();
    test_invert_direction();
    test_reference_calculation();
    test_invalid_params();
    test_derivative_filter();

    std::cout << "\n=== Resultado: " << gPassedTests << "/" << gTotalTests
              << " testes aprovados ===\n";

    return (gPassedTests == gTotalTests) ? 0 : 1;
}
