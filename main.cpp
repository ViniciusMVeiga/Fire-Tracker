/**
 * @file main.cpp
 * @brief Programa principal do sistema de rastreamento pan-tilt.
 *
 * Integra e coordena os quatro módulos:
 *   1. Image_Processing::Image_Processor  — captura e detecção do alvo
 *   2. Control::Pid_Controller (eixo X)   — controlador PID pan
 *   3. Control::Pid_Controller (eixo Y)   — controlador PID tilt
 *   4. Actuators::Servo_Driver            — acionamento dos servos via pigpio
 *
 * Fluxo do loop de controle (período Ts = Config::SAMPLE_PERIOD_S):
 *   1. Captura frame e executa pipeline de detecção
 *   2. Verifica validade e atualidade da medição
 *   3. Calcula erros: error_x = x_ref - x, error_y = y_ref - y
 *   4. Calcula saídas dos PIDs
 *   5. Envia larguras de pulso aos servos
 *   6. Aguarda o restante do período (temporização por deadline absoluto)
 *
 * Temporização:
 *   Utiliza deadline absoluto (next_cycle = last_cycle + Ts) em vez de
 *   sleep(Ts) após o processamento. Isso evita acúmulo de atrasos ao longo
 *   do tempo. Se o processamento exceder Ts, o atraso é detectado e
 *   registrado, mas o próximo ciclo começa imediatamente (sem salto duplo).
 *
 * Encerramento:
 *   Tratado por sinal SIGINT (Ctrl+C) e SIGTERM. Os servos são levados à
 *   posição segura antes de encerrar.
 */

#include "Config/config.hpp"
#include "Image_Processing/Image_Processor.hpp"
#include "Control/Pid_Controller.hpp"
#include "Actuators/Servo_Driver.hpp"
#include "Actuators/Laser_Driver.hpp"

#include <iostream>
#include <csignal>
#include <atomic>
#include <chrono>
#include <thread>
#include <stdexcept>
#include <cmath>

// ─────────────────────────────────────────────────────────────────────────────
// Tratamento de sinais de encerramento
// ─────────────────────────────────────────────────────────────────────────────

/// Flag atômica verificada no loop principal. Setada por SIGINT/SIGTERM.
static std::atomic<bool> gRunning{true};

static void signal_handler(int signal)
{
    std::cout << "\n[main] Sinal " << signal
              << " recebido. Encerrando o sistema...\n";
    gRunning.store(false);
}

// ─────────────────────────────────────────────────────────────────────────────
// Função auxiliar: constrói parâmetros a partir de Config::*
// ─────────────────────────────────────────────────────────────────────────────

static Image_Processing::Detection_Params build_image_params()
{
    Image_Processing::Detection_Params p;

    p.imageWidth      = Config::IMAGE_WIDTH;
    p.imageHeight     = Config::IMAGE_HEIGHT;
    p.cameraFps       = Config::CAMERA_FPS;
    p.blurKernelSize  = Config::BLUR_KERNEL_SIZE;

    p.hsv1Low  = {static_cast<double>(Config::HSV1_H_LOW),
                  static_cast<double>(Config::HSV1_S_LOW),
                  static_cast<double>(Config::HSV1_V_LOW)};
    p.hsv1High = {static_cast<double>(Config::HSV1_H_HIGH),
                  static_cast<double>(Config::HSV1_S_HIGH),
                  static_cast<double>(Config::HSV1_V_HIGH)};
    p.hsv2Low  = {static_cast<double>(Config::HSV2_H_LOW),
                  static_cast<double>(Config::HSV2_S_LOW),
                  static_cast<double>(Config::HSV2_V_LOW)};
    p.hsv2High = {static_cast<double>(Config::HSV2_H_HIGH),
                  static_cast<double>(Config::HSV2_S_HIGH),
                  static_cast<double>(Config::HSV2_V_HIGH)};

    p.morphOpenSize    = Config::MORPH_OPEN_SIZE;
    p.morphCloseSize   = Config::MORPH_CLOSE_SIZE;
    p.minContourArea   = Config::MIN_CONTOUR_AREA;
    p.gstreamerTemplate = Config::GSTREAMER_PIPELINE_TEMPLATE;
    p.useMockCamera    = Config::HARDWARE_MOCK;

    return p;
}

static Control::Pid_Params build_pid_x_params()
{
    Control::Pid_Params p;
    p.kp               = Config::PID_X_KP;
    p.ki               = Config::PID_X_KI;
    p.kd               = Config::PID_X_KD;
    p.biasMs           = Config::PID_X_BIAS_MS;
    p.outMinMs         = Config::PID_X_OUT_MIN_MS;
    p.outMaxMs         = Config::PID_X_OUT_MAX_MS;
    p.integralLimitMs  = Config::PID_X_INT_LIMIT_MS;
    p.samplePeriodS    = Config::SAMPLE_PERIOD_S;
    p.invertDirection  = Config::PID_X_INVERT;
    return p;
}

static Control::Pid_Params build_pid_y_params()
{
    Control::Pid_Params p;
    p.kp               = Config::PID_Y_KP;
    p.ki               = Config::PID_Y_KI;
    p.kd               = Config::PID_Y_KD;
    p.biasMs           = Config::PID_Y_BIAS_MS;
    p.outMinMs         = Config::PID_Y_OUT_MIN_MS;
    p.outMaxMs         = Config::PID_Y_OUT_MAX_MS;
    p.integralLimitMs  = Config::PID_Y_INT_LIMIT_MS;
    p.samplePeriodS    = Config::SAMPLE_PERIOD_S;
    p.invertDirection  = Config::PID_Y_INVERT;
    return p;
}

static Actuators::Servo_Params build_servo_params()
{
    Actuators::Servo_Params p;
    p.panGpio    = Config::SERVO_PAN_GPIO;
    p.tiltGpio   = Config::SERVO_TILT_GPIO;
    p.pwmFreqHz  = Config::SERVO_PWM_FREQ_HZ;
    p.safePulseMs = Config::SERVO_SAFE_PULSE_MS;
    p.absMinMs   = Config::SERVO_ABS_MIN_MS;
    p.absMaxMs   = Config::SERVO_ABS_MAX_MS;
    return p;
}

// ─────────────────────────────────────────────────────────────────────────────
// main()
// ─────────────────────────────────────────────────────────────────────────────

int main()
{
    // ── Registra tratadores de sinal ─────────────────────────────────────────
    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);

    std::cout << "=== Sistema Pan-Tilt PID ===\n"
              << "Período de amostragem: " << Config::SAMPLE_PERIOD_S * 1000.0
              << " ms\n"
              << "Resolução configurada: " << Config::IMAGE_WIDTH
              << "x" << Config::IMAGE_HEIGHT << "\n"
              << "Modo: " << (Config::HARDWARE_MOCK ? "MOCK (sem hardware)" : "REAL")
              << "\n\n";

    // ── 1. Inicializa o driver de servos ─────────────────────────────────────
    Actuators::Servo_Driver servoDriver(build_servo_params());

    if (!servoDriver.initialize()) {
        std::cerr << "[main] Falha na inicialização dos servos. Abortando.\n";
        return 1;
    }

    // ── 1b. Inicializa o driver do laser ─────────────────────────────────────
    // ATENÇÃO: o Laser_Driver usa gpioSetMode/gpioWrite e pressupõe que o
    // pigpio já foi inicializado pelo Servo_Driver acima.
    Actuators::Laser_Driver laserDriver(Config::LASER_GPIO);

    if (!laserDriver.initialize()) {
        std::cerr << "[main] Falha na inicialização do laser. Abortando.\n";
        servoDriver.shutdown();
        return 1;
    }

    // ── 2. Inicializa o processamento de imagem ──────────────────────────────
    Image_Processing::Image_Processor imageProc(build_image_params());

    if (!imageProc.initialize()) {
        std::cerr << "[main] Falha na inicialização da câmera. Abortando.\n";
        laserDriver.shutdown();
        servoDriver.shutdown();
        return 1;
    }

    // ── 3. Obtém dimensões efetivas e define referências ─────────────────────
    const cv::Size frameSize = imageProc.get_frame_size();

    if (frameSize.width == 0 || frameSize.height == 0) {
        std::cerr << "[main] Dimensões de frame inválidas. Abortando.\n";
        imageProc.shutdown();
        laserDriver.shutdown();
        servoDriver.shutdown();
        return 1;
    }

    // Referência = centro da imagem (nas dimensões efetivas do frame)
    const double xRef = frameSize.width  / 2.0;
    const double yRef = frameSize.height / 2.0;

    std::cout << "[main] Frame efetivo: " << frameSize.width
              << "x" << frameSize.height << "\n"
              << "[main] Referência: x_ref = " << xRef
              << ", y_ref = " << yRef << "\n\n";

    // ── 4. Inicializa os controladores PID ───────────────────────────────────
    Control::Pid_Controller pidX(build_pid_x_params());
    Control::Pid_Controller pidY(build_pid_y_params());

    // ── 5. Loop de controle ──────────────────────────────────────────────────
    using Clock    = std::chrono::steady_clock;
    using Duration = std::chrono::duration<double>;

    const Duration samplePeriod{Config::SAMPLE_PERIOD_S};
    const Duration maxMeasAge{Config::MAX_MEASUREMENT_AGE_S};

    auto nextCycle    = Clock::now() + samplePeriod;
    long cycleCount   = 0L;
    long missCount    = 0L;  ///< Ciclos com processamento mais lento que Ts

    std::cout << "[main] Loop de controle iniciado. CTRL+C para encerrar.\n";

    while (gRunning.load()) {
        const auto cycleStart = Clock::now();
        ++cycleCount;

        // ── 5a. Captura e processa frame ─────────────────────────────────────
        const Image_Processing::Target_Measurement meas = imageProc.process_frame();

        // ── 5b. Valida a medição ─────────────────────────────────────────────
        const auto now    = Clock::now();
        const Duration age = now - meas.timestamp;

        bool validMeas = meas.valid && (age <= maxMeasAge);

        if (!meas.valid) {
            // Alvo não detectado: mantém servos na posição atual (sem novo comando)
            if (cycleCount % 30 == 0) {
                std::cout << "[main] Ciclo " << cycleCount
                          << ": alvo não detectado.\n";
            }
            // Desliga o laser enquanto o alvo não é detectado
            laserDriver.turn_off();
            // Reinicia os integradores para evitar windup acumulado sem medição
            pidX.reset();
            pidY.reset();
        } else if (age > maxMeasAge) {
            std::cerr << "[main] Ciclo " << cycleCount
                      << ": medição desatualizada (" << age.count() * 1000.0
                      << " ms). Ignorada.\n";
            validMeas = false;
        }

        // ── 5c. Calcula e envia comandos somente com medição válida ──────────
        if (validMeas) {
            // Liga o laser ao detectar o alvo
            laserDriver.turn_on();

            // Erros de posição [px]
            const double errorX = xRef - static_cast<double>(meas.x);
            const double errorY = yRef - static_cast<double>(meas.y);

            // ── 5d. Executa os PIDs ──────────────────────────────────────────
            const Control::Pid_Output outX = pidX.calculate_output(errorX);
            const Control::Pid_Output outY = pidY.calculate_output(errorY);

            // ── 5e. Aciona os servos ─────────────────────────────────────────
            const bool servoOk = servoDriver.set_both_pulses(
                outX.pulseWidthMs, outY.pulseWidthMs);

            if (!servoOk) {
                std::cerr << "[main] Ciclo " << cycleCount
                          << ": falha no acionamento dos servos.\n";
            }

            // Log periódico (a cada 20 ciclos ≈ 1 s a 20 Hz)
            if (cycleCount % 20 == 0) {
                std::cout << "[main] Ciclo " << cycleCount
                          << " | Alvo:(" << meas.x << "," << meas.y << ")"
                          << " | Erro X=" << errorX << " Y=" << errorY
                          << " | Pan=" << outX.pulseWidthMs << " ms"
                          << " | Tilt=" << outY.pulseWidthMs << " ms"
                          << (outX.saturated ? " [PAN SAT]" : "")
                          << (outY.saturated ? " [TILT SAT]" : "")
                          << "\n";
            }
        }

        // ── 5f. Verifica atraso do ciclo ─────────────────────────────────────
        const auto cycleDuration = Clock::now() - cycleStart;
        if (cycleDuration > samplePeriod) {
            ++missCount;
            if (missCount % 10 == 1) {
                std::cerr << "[main] AVISO: processamento excedeu Ts em "
                          << std::chrono::duration_cast<
                                 std::chrono::milliseconds>(
                                 cycleDuration - samplePeriod)
                                 .count()
                          << " ms (misses acumulados: " << missCount << ").\n";
            }
            // Avança o deadline para o próximo ciclo sem acumular atrasos
            nextCycle = Clock::now() + samplePeriod;
        } else {
            // ── 5g. Aguarda o restante do período ─────────────────────────────
            std::this_thread::sleep_until(nextCycle);
            nextCycle += samplePeriod;
        }
    }

    // ── 6. Encerramento seguro ───────────────────────────────────────────────
    std::cout << "\n[main] Encerrando. Ciclos totais: " << cycleCount
              << " | Atrasos: " << missCount << "\n";

    imageProc.shutdown();
    laserDriver.shutdown();   // garante laser desligado antes de encerrar pigpio
    servoDriver.shutdown();   // aplica pulso seguro e encerra pigpio

    std::cout << "[main] Sistema encerrado.\n";
    return 0;
}
