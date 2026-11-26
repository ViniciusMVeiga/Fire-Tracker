/**
 * @file Pid_Controller.cpp
 * @brief Implementação do controlador PID digital.
 *
 * Ver Pid_Controller.hpp para a descrição completa das equações e requisitos.
 */

#include "Pid_Controller.hpp"

#include <algorithm>  // std::clamp
#include <stdexcept>
#include <cmath>      // std::isfinite

namespace Control {

// ─────────────────────────────────────────────────────────────────────────────
// Construtor
// ─────────────────────────────────────────────────────────────────────────────

Pid_Controller::Pid_Controller(Pid_Params params)
    : mParams(std::move(params))
{
    // Validações básicas de parâmetros
    if (mParams.samplePeriodS <= 0.0) {
        throw std::invalid_argument(
            "Pid_Controller: samplePeriodS deve ser positivo.");
    }
    if (mParams.outMinMs >= mParams.outMaxMs) {
        throw std::invalid_argument(
            "Pid_Controller: outMinMs deve ser menor que outMaxMs.");
    }
    if (mParams.integralLimitMs < 0.0) {
        throw std::invalid_argument(
            "Pid_Controller: integralLimitMs deve ser não-negativo.");
    }
    if (mParams.derivativeFilterN < 0.0) {
        throw std::invalid_argument(
            "Pid_Controller: derivativeFilterN deve ser não-negativo.");
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Cálculo da saída (ciclo principal)
// ─────────────────────────────────────────────────────────────────────────────

Pid_Output Pid_Controller::calculate_output(double error)
{
    Pid_Output out;

    // Inversão de sentido configurável (ex.: eixo Y da câmera cresce para baixo)
    if (mParams.invertDirection) {
        error = -error;
    }
    out.error = error;

    const double Ts = mParams.samplePeriodS;
    const double Kp = mParams.kp;
    const double Ki = mParams.ki;
    const double Kd = mParams.kd;
    const double N  = mParams.derivativeFilterN;

    // ── Termo Proporcional ───────────────────────────────────────────────────
    const double termP = Kp * error;
    out.termP = termP;

    // ── Termo Integral (Euler à frente) ──────────────────────────────────────
    // Acumula antes do clamping para verificar saturação
    mState.integralAccum += Ki * Ts * error;

    // Anti-windup: clamping do acumulador integral
    mState.integralAccum = clamp_integral(
        mState.integralAccum, mParams.integralLimitMs);

    const double termI = mState.integralAccum;
    out.termI = termI;

    // ── Termo Derivativo (filtro de primeira ordem) ──────────────────────────
    //
    // Para evitar impacto de ruído nas coordenadas de pixel, usamos um filtro
    // de primeira ordem (derivativo com polo em s = -N/Ts equivalente discreto):
    //
    //   D(k) = [ Kd*(e(k) - e(k-1))/Ts + N*D(k-1) ] / (1 + N)
    //
    // Em vez da derivada pura: D(k) = Kd * (e(k) - e(k-1)) / Ts
    //
    // Na primeira amostra (firstSample = true) não há histórico, então D = 0.
    double termD = 0.0;

    if (!mState.firstSample && N > 0.0) {
        const double dRaw = Kd * (error - mState.prevError) / Ts;
        termD = (dRaw + N * mState.prevDerivative) / (1.0 + N);
    } else if (!mState.firstSample && N == 0.0) {
        // N = 0 → derivativo puro (sem filtro)
        termD = Kd * (error - mState.prevError) / Ts;
    }

    out.termD = termD;

    // ── Saída bruta ──────────────────────────────────────────────────────────
    const double rawOutput = mParams.biasMs + termP + termI + termD;
    out.rawOutput = rawOutput;

    // ── Saturação da saída ───────────────────────────────────────────────────
    const double saturatedOutput = clamp_output(
        rawOutput, mParams.outMinMs, mParams.outMaxMs);

    out.pulseWidthMs = saturatedOutput;
    out.saturated    = (saturatedOutput != rawOutput);

    // ── Anti-windup: back-calculation se saturado ────────────────────────────
    // Se a saída foi saturada, reverte a última adição ao integrador para evitar
    // que ele continue crescendo enquanto a saída está limitada.
    if (out.saturated) {
        // Desfaz a contribuição adicionada neste ciclo e reaaplica o clamp
        mState.integralAccum -= Ki * Ts * error;
        mState.integralAccum  = clamp_integral(
            mState.integralAccum, mParams.integralLimitMs);
    }

    // ── Atualiza estado interno ──────────────────────────────────────────────
    mState.prevError      = error;
    mState.prevDerivative = termD;
    mState.firstSample    = false;

    return out;
}

// ─────────────────────────────────────────────────────────────────────────────
// Reinicialização
// ─────────────────────────────────────────────────────────────────────────────

void Pid_Controller::reset()
{
    mState.integralAccum  = 0.0;
    mState.prevError      = 0.0;
    mState.prevDerivative = 0.0;
    mState.firstSample    = true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Accessores
// ─────────────────────────────────────────────────────────────────────────────

void Pid_Controller::set_params(const Pid_Params& params)
{
    if (params.samplePeriodS <= 0.0) {
        throw std::invalid_argument("set_params: samplePeriodS deve ser positivo.");
    }
    mParams = params;
}

const Pid_Params& Pid_Controller::get_params() const
{
    return mParams;
}

const Pid_State& Pid_Controller::get_state() const
{
    return mState;
}

// ─────────────────────────────────────────────────────────────────────────────
// Funções auxiliares estáticas
// ─────────────────────────────────────────────────────────────────────────────

double Pid_Controller::clamp_integral(double value, double limit)
{
    if (limit <= 0.0) {
        return value;  // sem limitação se limit = 0 (desabilitado)
    }
    return std::clamp(value, -limit, limit);
}

double Pid_Controller::clamp_output(double value, double minVal, double maxVal)
{
    return std::clamp(value, minVal, maxVal);
}

} // namespace Control
