/**
 * @file Pid_Controller.hpp
 * @brief Interface do controlador PID digital para um único eixo.
 *
 * Implementa um PID discreto com:
 *   - Polarização (bias) em milissegundos
 *   - Anti-windup por clamping do integrador
 *   - Filtro derivativo de primeira ordem (coeficiente N configurável)
 *   - Inversão de sentido configurável
 *   - Tratamento de medições inválidas (não calcula saída sem dado válido)
 *   - Reinicialização completa do estado interno
 *
 * A saída é uma largura de pulso absoluta em milissegundos [ms], não um
 * incremento de ângulo. O valor de repouso (servo parado, alvo centrado) é
 * controlado pelo parâmetro biasMs.
 *
 * Equações discretas utilizadas (período de amostragem Ts):
 *
 *   Termo proporcional:
 *     P(k) = Kp * e(k)
 *
 *   Termo integral (método de Euler à frente, com anti-windup):
 *     I(k) = I(k-1) + Ki * Ts * e(k)
 *     I(k) = clamp(I(k), -integralLimit, +integralLimit)
 *
 *   Termo derivativo (filtro de primeira ordem com coeficiente N):
 *     D(k) = (Kd/Ts) * (e(k) - e(k-1)) / (1 + N * Kd/Ts)
 *     — filtrado para atenuar ruído nas coordenadas de pixel.
 *
 *   Saída bruta:
 *     u(k) = biasMs + P(k) + I(k) + D(k)
 *
 *   Saída saturada:
 *     output(k) = clamp(u(k), outMinMs, outMaxMs)
 *
 *   Anti-windup: se output(k) ≠ u(k) (saturação ativa), o integrador é
 *   retido no valor anterior (back-calculation implícito via clamping do
 *   acumulador).
 */

#pragma once

#include <chrono>
#include <optional>

namespace Control {

// ─────────────────────────────────────────────────────────────────────────────
// Parâmetros do controlador
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Conjunto de parâmetros de um controlador PID de eixo único.
 */
struct Pid_Params {
    double kp = 0.0;         ///< Ganho proporcional [ms/px]
    double ki = 0.0;         ///< Ganho integral     [ms/(px·s)]
    double kd = 0.0;         ///< Ganho derivativo   [ms·s/px]

    double biasMs    = 1.5;  ///< Polarização (largura de pulso de repouso) [ms]
    double outMinMs  = 1.0;  ///< Limite inferior da saída [ms]
    double outMaxMs  = 2.0;  ///< Limite superior da saída [ms]

    /// Limite do acumulador integral (anti-windup por clamping) [ms].
    double integralLimitMs = 0.4;

    /// Coeficiente do filtro derivativo. Valores típicos: 5–20.
    /// N → ∞ equivale a derivativo puro (sem filtragem).
    double derivativeFilterN = 10.0;

    /// Período de amostragem [s]. Deve corresponder ao período real do loop.
    double samplePeriodS = 0.05;

    /// Se true, o sinal do erro é invertido antes do cálculo (inversão de eixo).
    bool invertDirection = false;
};

// ─────────────────────────────────────────────────────────────────────────────
// Estado interno (exposto para inspeção/depuração)
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Estado interno do controlador em um dado instante.
 */
struct Pid_State {
    double integralAccum  = 0.0;  ///< Acumulador integral [ms]
    double prevError      = 0.0;  ///< Erro no instante anterior [px]
    double prevDerivative = 0.0;  ///< Derivativo filtrado anterior [ms]
    bool   firstSample    = true; ///< true enquanto não houver amostra anterior
};

// ─────────────────────────────────────────────────────────────────────────────
// Resultado de um ciclo de cálculo
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Resultado detalhado de calculate_output(), útil para depuração.
 */
struct Pid_Output {
    double pulseWidthMs = 1.5; ///< Saída final (saturada) [ms]
    double rawOutput    = 1.5; ///< Saída antes da saturação [ms]
    double termP        = 0.0; ///< Contribuição proporcional [ms]
    double termI        = 0.0; ///< Contribuição integral [ms]
    double termD        = 0.0; ///< Contribuição derivativa [ms]
    double error        = 0.0; ///< Erro utilizado neste ciclo [px]
    bool   saturated    = false; ///< true se a saída foi limitada
};

// ─────────────────────────────────────────────────────────────────────────────
// Controlador PID
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Controlador PID digital para um único eixo (pan ou tilt).
 *
 * Cada eixo deverá ter sua própria instância. O estado interno (integrador,
 * derivativo, amostra anterior) é completamente independente.
 *
 * Uso típico:
 * @code
 *   Pid_Params params;
 *   params.kp = 0.001; params.ki = 0.0001; params.kd = 0.0005;
 *   Pid_Controller ctrl(params);
 *
 *   // A cada ciclo:
 *   double error = xRef - xMeasured;
 *   Pid_Output out = ctrl.calculate_output(error);
 *   servo.set_pulse(out.pulseWidthMs);
 * @endcode
 */
class Pid_Controller {
public:
    explicit Pid_Controller(Pid_Params params);

    /**
     * @brief Calcula a saída do controlador para um dado erro.
     *
     * @param error Erro de posição atual [px]: ref - medida.
     *              Se invertDirection = true, o sinal é invertido internamente.
     * @return Pid_Output com largura de pulso e termos individuais.
     */
    Pid_Output calculate_output(double error);

    /**
     * @brief Reinicia o estado interno do controlador.
     *
     * Zera integrador, erro anterior e derivativo. O próximo ciclo
     * será tratado como a primeira amostra.
     */
    void reset();

    /**
     * @brief Atualiza os parâmetros do controlador sem reiniciar o estado.
     *
     * Permite ajuste em tempo de execução (auto-tuning, ganho programável).
     */
    void set_params(const Pid_Params& params);

    /**
     * @brief Retorna os parâmetros atuais (const).
     */
    const Pid_Params& get_params() const;

    /**
     * @brief Retorna o estado interno atual (para inspeção/depuração).
     */
    const Pid_State& get_state() const;

private:
    Pid_Params mParams;
    Pid_State  mState;

    /// Aplica clamping simétrico ao acumulador integral.
    static double clamp_integral(double value, double limit);

    /// Aplica clamping ao resultado final.
    static double clamp_output(double value, double minVal, double maxVal);
};

} // namespace Control
