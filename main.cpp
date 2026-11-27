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
 * Fluxo do loop de controle (período Ts = Config::SAMPLE_PERIOD_S, ajustável
 * em tempo de execução pela janela de configurações quando o sistema está parado):
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
#include "Image_Processing/Camera_Display.hpp"
#include "Image_Processing/Pid_Params_Editor.hpp"
#include "Image_Processing/Error_Log.hpp"
#include "Control/Pid_Controller.hpp"
#include "Actuators/Servo_Driver.hpp"
#include "Actuators/Laser_Driver.hpp"

#include <algorithm>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <ctime>
#include <csignal>
#include <cstdlib>
#include <cctype>
#include <atomic>
#include <chrono>
#include <optional>
#include <thread>
#include <stdexcept>
#include <cmath>

#include <sys/stat.h>  // mkdir da pasta logs/
#ifdef _WIN32
#  include <direct.h>  // _mkdir no Windows (apenas para validação local)
#endif

#include <opencv2/highgui.hpp>

// ─────────────────────────────────────────────────────────────────────────────
// Tratamento de sinais de encerramento
// ─────────────────────────────────────────────────────────────────────────────

/// Flag atômica verificada no loop principal. Setada por SIGINT/SIGTERM.
static std::atomic<bool> gRunning{true};

/// Estado do painel de controle (Start/Stop + parâmetros PID editáveis).
static Image_Processing::Control_Panel_State gCtrl;

/// Ponteiros para os PIDs, usados para aplicar os parâmetros editados na GUI.
static Control::Pid_Controller* gPidX = nullptr;
static Control::Pid_Controller* gPidY = nullptr;

/// Janela de configurações (campos de texto + botão de salvamento).
static Image_Processing::Pid_Params_Editor gPidEditor;

/// Log de erro da sessão atual (exportado em CSV sob demanda).
static Image_Processing::Error_Log gErrorLog;

/// Período de amostragem efetivo do loop [s]. Atualizado ao iniciar o controle.
static double gSamplePeriodS = Config::SAMPLE_PERIOD_S;

/// Dimensões efetivas do frame (preenchidas após abrir a câmera).
static int gFrameWidth = 0;
static int gFrameHeight = 0;

/// Mira: deslocamento editado e referência efetivamente aplicada [px].
/// (0,0) de deslocamento = centro da imagem; X+ à direita, Y+ para baixo.
static double gAimOffsetX = Config::AIM_OFFSET_X_PX;
static double gAimOffsetY = Config::AIM_OFFSET_Y_PX;
static double gRefX = 0.0;
static double gRefY = 0.0;

/// Marcações da sessão de log.
using Clock = std::chrono::steady_clock;
static Clock::time_point gSessionStart;
static long gSessionSample = 0L;

static void signal_handler(int signal)
{
    std::cout << "\n[main] Sinal " << signal
              << " recebido. Encerrando o sistema...\n";
    gRunning.store(false);
}

// ─────────────────────────────────────────────────────────────────────────────
// Janela de configurações — campos de texto e salvamento do CSV
// ─────────────────────────────────────────────────────────────────────────────

static const char* kParamsWin = Image_Processing::Pid_Params_Editor::window_name();

/// Inicializa gCtrl com os parâmetros atuais definidos em Config.
static void init_ctrl_from_config()
{
    gCtrl.kpX = Config::PID_X_KP;
    gCtrl.kiX = Config::PID_X_KI;
    gCtrl.kdX = Config::PID_X_KD;
    gCtrl.kpY = Config::PID_Y_KP;
    gCtrl.kiY = Config::PID_Y_KI;
    gCtrl.kdY = Config::PID_Y_KD;
    gCtrl.samplePeriodMs = Config::SAMPLE_PERIOD_S * 1000.0;
    gCtrl.aimOffsetX = Config::AIM_OFFSET_X_PX;
    gCtrl.aimOffsetY = Config::AIM_OFFSET_Y_PX;
    gCtrl.running = false;
}

/// Callback de mouse da janela de configurações.
static void on_params_mouse(int event, int x, int y, int /*flags*/,
                            void* /*userdata*/)
{
    gPidEditor.handle_mouse(event, x, y);
}

/// Cria a janela de configurações com os valores iniciais de Config.
static void init_params_editor(int frameWidth, int frameHeight)
{
    Image_Processing::Pid_Editor_Values values;
    values.kpX            = Config::PID_X_KP;
    values.kiX            = Config::PID_X_KI;
    values.kdX            = Config::PID_X_KD;
    values.kpY            = Config::PID_Y_KP;
    values.kiY            = Config::PID_Y_KI;
    values.kdY            = Config::PID_Y_KD;
    values.samplePeriodMs = Config::SAMPLE_PERIOD_S * 1000.0;
    values.aimOffsetX     = Config::AIM_OFFSET_X_PX;
    values.aimOffsetY     = Config::AIM_OFFSET_Y_PX;

    gPidEditor.set_frame_size(frameWidth, frameHeight);
    gPidEditor.set_values(values);
    cv::namedWindow(kParamsWin, cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback(kParamsWin, on_params_mouse, nullptr);
}

/// Copia os valores confirmados no editor para gCtrl.
static void apply_editor_to_ctrl()
{
    const Image_Processing::Pid_Editor_Values values = gPidEditor.values();
    gCtrl.kpX            = values.kpX;
    gCtrl.kiX            = values.kiX;
    gCtrl.kdX            = values.kdX;
    gCtrl.kpY            = values.kpY;
    gCtrl.kiY            = values.kiY;
    gCtrl.kdY            = values.kdY;
    gCtrl.samplePeriodMs = values.samplePeriodMs;

    // A mira é sempre refletida na pré-visualização; só vira referência do
    // controle quando o sistema é iniciado (ver set_running).
    gAimOffsetX = values.aimOffsetX;
    gAimOffsetY = values.aimOffsetY;
    gCtrl.aimOffsetX = values.aimOffsetX;
    gCtrl.aimOffsetY = values.aimOffsetY;
}

/// Reflete no dashboard a mira já confirmada no editor (também quando parado).
static void sync_aim_preview()
{
    const Image_Processing::Pid_Editor_Values values = gPidEditor.values();
    gAimOffsetX = values.aimOffsetX;
    gAimOffsetY = values.aimOffsetY;
    gCtrl.aimOffsetX = gAimOffsetX;
    gCtrl.aimOffsetY = gAimOffsetY;
}

/// Copia ganhos e período de amostragem de gCtrl para os PIDs.
/// Preserva bias e limites de saída; reinicia o estado interno.
static void apply_params_to_pids()
{
    const double ts = gCtrl.samplePeriodMs / 1000.0;

    if (gPidX != nullptr) {
        Control::Pid_Params p = gPidX->get_params();
        p.kp = gCtrl.kpX;
        p.ki = gCtrl.kiX;
        p.kd = gCtrl.kdX;
        p.samplePeriodS = ts;
        gPidX->set_params(p);
        gPidX->reset();
    }
    if (gPidY != nullptr) {
        Control::Pid_Params p = gPidY->get_params();
        p.kp = gCtrl.kpY;
        p.ki = gCtrl.kiY;
        p.kd = gCtrl.kdY;
        p.samplePeriodS = ts;
        gPidY->set_params(p);
        gPidY->reset();
    }
}

/// Liga/desliga o controle (botão do dashboard ou tecla S).
static void set_running(bool run, const char* source)
{
    if (run == gCtrl.running) {
        return;
    }

    if (run) {
        // Ao iniciar, os valores editados na janela de configurações passam a valer.
        apply_editor_to_ctrl();
        apply_params_to_pids();
        gSamplePeriodS = gCtrl.samplePeriodMs / 1000.0;

        // Aplica a mira em relação ao centro do frame e inicia nova sessão de log.
        gRefX = static_cast<double>(gFrameWidth) / 2.0 + gAimOffsetX;
        gRefY = static_cast<double>(gFrameHeight) / 2.0 + gAimOffsetY;
        gErrorLog.clear();
        gSessionSample = 0L;
        gSessionStart = Clock::now();
    } else {
        // Ao parar, zera integradores/derivativo para não retomar com estado velho.
        if (gPidX != nullptr) gPidX->reset();
        if (gPidY != nullptr) gPidY->reset();
    }

    gCtrl.running = run;
    gPidEditor.set_enabled(!run);

    std::cout << "[main] Sistema " << (run ? "INICIADO" : "PARADO")
              << " (" << source << "). Ganhos: "
              << "KpX=" << gCtrl.kpX << " KiX=" << gCtrl.kiX
              << " KdX=" << gCtrl.kdX
              << " | KpY=" << gCtrl.kpY << " KiY=" << gCtrl.kiY
              << " KdY=" << gCtrl.kdY
              << " | Ts=" << gCtrl.samplePeriodMs << " ms"
              << " | Mira=(" << gAimOffsetX << "," << gAimOffsetY << ") px\n";
}

/// Data/hora atual formatada para compor o nome do arquivo CSV.
static std::string make_timestamp()
{
    const std::time_t now = std::chrono::system_clock::to_time_t(
        std::chrono::system_clock::now());
    std::ostringstream oss;
    oss << std::put_time(std::localtime(&now), "%Y-%m-%d_%H-%M-%S");
    return oss.str();
}

/// Grava a sessão atual em logs/pid_AAAA-MM-DD_HH-MM-SS.csv.
static void save_csv_now()
{
    const std::string stamp = make_timestamp();
    const std::string dir   = "logs";
#ifdef _WIN32
    ::_mkdir(dir.c_str());  // ignorado se a pasta já existir
#else
    ::mkdir(dir.c_str(), 0755);  // ignorado se a pasta já existir
#endif
    const std::string path = dir + "/pid_" + stamp + ".csv";

    Image_Processing::Error_Log_Metadata meta;
    meta.timestamp      = stamp;
    meta.frameWidth     = gFrameWidth;
    meta.frameHeight    = gFrameHeight;
    meta.kpX = gCtrl.kpX; meta.kiX = gCtrl.kiX; meta.kdX = gCtrl.kdX;
    meta.kpY = gCtrl.kpY; meta.kiY = gCtrl.kiY; meta.kdY = gCtrl.kdY;
    meta.samplePeriodMs = gCtrl.samplePeriodMs;
    // Usa a mira efetivamente aplicada no último start (não a edição atual).
    meta.aimOffsetX     = gRefX - static_cast<double>(gFrameWidth) / 2.0;
    meta.aimOffsetY     = gRefY - static_cast<double>(gFrameHeight) / 2.0;
    meta.invertX        = Config::PID_X_INVERT;
    meta.invertY        = Config::PID_Y_INVERT;

    if (gErrorLog.save_csv(path, meta)) {
        std::cout << "[main] CSV salvo: " << path
                  << " (" << gErrorLog.size() << " amostras)\n";
        gPidEditor.show_status("CSV salvo: logs/pid_" + stamp + ".csv", true);
    } else {
        std::cerr << "[main] Falha ao salvar CSV em " << path << "\n";
        gPidEditor.show_status("Falha ao salvar CSV", false);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Mouse callback: detecta clique no botão Start/Stop do dashboard
// ─────────────────────────────────────────────────────────────────────────────

static void on_mouse(int event, int x, int y, int /*flags*/, void* /*userdata*/)
{
    if (event != cv::EVENT_LBUTTONDOWN) {
        return;
    }
    if (gCtrl.buttonRect.contains(cv::Point(x, y))) {
        set_running(!gCtrl.running, "botão");
    }
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
    p.rotate180         = Config::CAMERA_ROTATE_180;
    p.useMockCamera     = Config::HARDWARE_MOCK;

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
    p.panGpio         = Config::SERVO_PAN_GPIO;
    p.tiltGpio        = Config::SERVO_TILT_GPIO;
    p.pwmFreqHz       = Config::SERVO_PWM_FREQ_HZ;
    p.panSafePulseMs  = Config::SERVO_PAN_SAFE_PULSE_MS;
    p.tiltSafePulseMs = Config::SERVO_TILT_SAFE_PULSE_MS;
    p.absMinMs        = Config::SERVO_ABS_MIN_MS;
    p.absMaxMs        = Config::SERVO_ABS_MAX_MS;
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

    // Referência inicial = centro do frame; a janela de configurações pode
    // deslocá-la antes de iniciar (ver set_running).
    gFrameWidth  = frameSize.width;
    gFrameHeight = frameSize.height;
    gRefX = static_cast<double>(gFrameWidth) / 2.0 + gAimOffsetX;
    gRefY = static_cast<double>(gFrameHeight) / 2.0 + gAimOffsetY;
    gSessionStart = Clock::now();

    std::cout << "[main] Frame efetivo: " << gFrameWidth
              << "x" << gFrameHeight << "\n"
              << "[main] Referência inicial: x_ref = " << gRefX
              << ", y_ref = " << gRefY
              << " (mira = " << gAimOffsetX << "," << gAimOffsetY << " px)\n\n";

    // ── 4. Inicializa os controladores PID e visualização ────────────────────
    Control::Pid_Controller pidX(build_pid_x_params());
    Control::Pid_Controller pidY(build_pid_y_params());
    gPidX = &pidX;
    gPidY = &pidY;

    const bool guiEnabled = (std::getenv("DISPLAY") != nullptr ||
                             std::getenv("WAYLAND_DISPLAY") != nullptr);
    Image_Processing::Pid_Error_Plot errorPlot(gRefX, gRefY, 160);
    Image_Processing::Dashboard_Renderer dashboard;

    // Estado inicial do painel: parâmetros vindos de Config (não de valores fixos).
    init_ctrl_from_config();

    if (guiEnabled) {
        cv::namedWindow("Pan-Tilt PID", cv::WINDOW_AUTOSIZE);
        cv::setMouseCallback("Pan-Tilt PID", on_mouse, nullptr);
        init_params_editor(gFrameWidth, gFrameHeight);
        std::cout << "[main] Interface gráfica ativa. Use o botão ou a tecla S "
                     "para iniciar/parar. Edite ganhos, Ts e mira na janela "
                     "de configurações com o sistema parado.\n";
    } else {
        // Sem display não há como acionar o botão: mantém o comportamento
        // anterior e inicia o controle imediatamente.
        gCtrl.running = true;
        std::cout << "[main] Aviso: DISPLAY/WAYLAND_DISPLAY ausente; "
                     "janelas visuais desativadas (controle iniciado).\n"
                  << "[main] Dica: se estiver usando sudo no Wayland, rode "
                     "'sudo -E ./pantilt' para preservar o ambiente gráfico.\n";
    }

    // ── 5. Loop de controle ──────────────────────────────────────────────────
    using Duration = std::chrono::duration<double>;

    long cycleCount   = 0L;
    long missCount    = 0L;  ///< Ciclos com processamento mais lento que Ts

    std::cout << "[main] Loop principal iniciado. CTRL+C para encerrar.\n";
    if (!gCtrl.running) {
        std::cout << "[main] Controle PARADO. Use o botão INICIAR ou a tecla S.\n";
    }

    while (gRunning.load()) {
        const auto cycleStart = Clock::now();
        ++cycleCount;

        // Período e validade da medição acompanham o Ts editado pelo usuário.
        const Duration samplePeriod{gSamplePeriodS};
        const Duration maxMeasAge{
            std::max(Config::MAX_MEASUREMENT_AGE_S, 1.5 * gSamplePeriodS)};
        const auto nextCycle = cycleStart + samplePeriod;

        // ── 5a. Captura e processa frame ─────────────────────────────────────
        const Image_Processing::Target_Measurement meas = imageProc.process_frame();

        // ── 5b. Valida a medição ─────────────────────────────────────────────
        const auto now    = Clock::now();
        const Duration age = now - meas.timestamp;

        bool validMeas = meas.valid && (age <= maxMeasAge);

        std::optional<double> plotErrorX;
        std::optional<double> plotErrorY;
        double errorX = 0.0;
        double errorY = 0.0;
        double panMs  = servoDriver.get_pan_pulse_ms();
        double tiltMs = servoDriver.get_tilt_pulse_ms();
        bool panSat   = false;
        bool tiltSat  = false;

        if (!gCtrl.running) {
            // Sistema parado: mantém os servos na posição atual, desliga o
            // laser e reflete no dashboard os valores editados no editor.
            apply_editor_to_ctrl();
            laserDriver.turn_off();
            validMeas = false;
        } else if (!meas.valid) {
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
        if (gCtrl.running && validMeas) {
            // Liga o laser ao detectar o alvo
            laserDriver.turn_on();

            // Erros de posição [px] em relação à mira configurada
            errorX = gRefX - static_cast<double>(meas.x);
            errorY = gRefY - static_cast<double>(meas.y);

            // ── 5d. Executa os PIDs ──────────────────────────────────────────
            const Control::Pid_Output outX = pidX.calculate_output(errorX);
            const Control::Pid_Output outY = pidY.calculate_output(errorY);

            plotErrorX = errorX;
            plotErrorY = errorY;
            panMs      = outX.pulseWidthMs;
            tiltMs     = outY.pulseWidthMs;
            panSat     = outX.saturated;
            tiltSat    = outY.saturated;

            // ── 5e. Aciona os servos ─────────────────────────────────────────
            const bool servoOk = servoDriver.set_both_pulses(
                outX.pulseWidthMs, outY.pulseWidthMs);

            if (!servoOk) {
                std::cerr << "[main] Ciclo " << cycleCount
                          << ": falha no acionamento dos servos.\n";
            }

            // Log periódico (a cada 20 ciclos)
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

        // ── 5e1. Registra a amostra no log da sessão (inclusive falhas) ───────
        if (gCtrl.running) {
            Image_Processing::Error_Sample sample;
            sample.sample  = ++gSessionSample;
            sample.timeS   = Duration(Clock::now() - gSessionStart).count();
            sample.valid   = validMeas;
            sample.refX    = gRefX;
            sample.refY    = gRefY;
            sample.panMs   = panMs;
            sample.tiltMs  = tiltMs;
            sample.panSat  = panSat;
            sample.tiltSat = tiltSat;

            if (validMeas) {
                sample.errorX  = errorX;
                sample.errorY  = errorY;
                sample.targetX = meas.x;
                sample.targetY = meas.y;
            }
            gErrorLog.push(sample);

            if (gErrorLog.full() && cycleCount % 100 == 0) {
                std::cerr << "[main] AVISO: limite de "
                          << Image_Processing::Error_Log::max_samples()
                          << " amostras do CSV atingido. Salve o arquivo.\n";
            }
        }

        // ── 5e2. Atualiza janela única (câmera, gráfico e painel de controle)
        if (guiEnabled) {
            sync_aim_preview();

            const cv::Point aimPoint(
                static_cast<int>(static_cast<double>(gFrameWidth) / 2.0 +
                                 gAimOffsetX),
                static_cast<int>(static_cast<double>(gFrameHeight) / 2.0 +
                                 gAimOffsetY));

            // Escala do gráfico acompanha a mira (maior erro possível no eixo).
            errorPlot.set_max_abs(
                std::max(gRefX, static_cast<double>(gFrameWidth) - gRefX),
                std::max(gRefY, static_cast<double>(gFrameHeight) - gRefY));
            errorPlot.push_sample(plotErrorX, plotErrorY);

            const int divider = std::max(1, Config::GUI_UPDATE_DIVIDER);
            if (cycleCount % divider == 0) {
                const cv::Mat& lastFrame = imageProc.get_last_frame();
                if (!lastFrame.empty()) {
                    const std::string statusLine =
                        "Pan: " + Image_Processing::format_decimal(panMs) +
                        " ms | Tilt: " + Image_Processing::format_decimal(tiltMs) +
                        " ms | Laser: " +
                        ((gCtrl.running && validMeas) ? "ON" : "OFF");

                    const cv::Mat& dashImg = dashboard.render(
                        lastFrame,
                        imageProc.get_last_mask(),
                        meas,
                        aimPoint,
                        statusLine,
                        errorPlot,
                        panMs,
                        tiltMs,
                        panSat,
                        tiltSat,
                        validMeas,
                        gCtrl);

                    cv::imshow("Pan-Tilt PID", dashImg);
                }
            }

            // Processa eventos da GUI em todo ciclo (resposta mais rápida).
            const int rawKey = cv::waitKey(1);
            if (rawKey >= 0) {
                // Enquanto um campo está ativo, as teclas pertencem ao editor.
                if (!gPidEditor.handle_key(rawKey)) {
                    const int key = std::tolower(
                        static_cast<unsigned char>(rawKey & 0xFF));
                    if (key == 27 || key == 'q') {
                        gRunning.store(false);
                    } else if (key == 's') {
                        set_running(!gCtrl.running, "tecla S");
                    }
                }
            }

            // Pedido de salvamento vindo do botão da janela de configurações.
            if (gPidEditor.consume_save_request()) {
                save_csv_now();
            }

            // Atualiza a janela de configurações (campos e status desenhados).
            cv::imshow(kParamsWin, gPidEditor.render());
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
            // O deadline é recalculado no início de cada ciclo, então um Ts
            // alterado ou um atraso não acumulam erro de temporização.
        } else {
            // ── 5g. Aguarda o restante do período ─────────────────────────────
            std::this_thread::sleep_until(nextCycle);
        }
    }

    // ── 6. Encerramento seguro ───────────────────────────────────────────────
    std::cout << "\n[main] Encerrando. Ciclos totais: " << cycleCount
              << " | Atrasos: " << missCount << "\n";

    imageProc.shutdown();
    laserDriver.shutdown();   // garante laser desligado antes de encerrar pigpio
    servoDriver.shutdown();   // aplica pulso seguro e encerra pigpio
    if (guiEnabled) {
        cv::destroyAllWindows();
    }

    std::cout << "[main] Sistema encerrado.\n";
    return 0;
}
