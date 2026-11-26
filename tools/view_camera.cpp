/**
 * @file view_camera.cpp
 * @brief Visualizador em tempo real da câmera com calibração interativa dos servos.
 *
 * Exibe duas janelas:
 *   1. "Alvo Detectado" - Imagem da câmera, mira central, detecção e HUD dos servos.
 *   2. "Mascara HSV"    - Visão binária do filtro de fogo.
 *
 * Controles na janela:
 *   - W / S       : ajusta Tilt (eixo Y) + / -
 *   - A / D       : ajusta Pan  (eixo X) - / +
 *   - 1 / 2 / 3   : seleciona passo (0.01 / 0.05 / 0.10 ms)
 *   - C           : centraliza nos valores de bias do Config
 *   - P           : imprime os pulsos atuais no terminal
 *   - Q ou ESC    : encerra o programa (retorna servos à posição segura)
 */

#include "Config/config.hpp"
#include "Image_Processing/Image_Processor.hpp"
#include "Image_Processing/Camera_Display.hpp"
#include "Actuators/Servo_Driver.hpp"

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cmath>
#include <csignal>
#include <iostream>
#include <string>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

namespace {

std::atomic<bool> gRunning{true};

void signal_handler(int /*signal*/)
{
    gRunning.store(false);
}

double round_ms(double value)
{
    return std::round(value * 1000.0) / 1000.0;
}

double clamp_pulse_ms(double value)
{
    return std::clamp(
        round_ms(value),
        Config::SERVO_ABS_MIN_MS,
        Config::SERVO_ABS_MAX_MS);
}

Image_Processing::Detection_Params build_image_params()
{
    Image_Processing::Detection_Params params;
    params.imageWidth        = Config::IMAGE_WIDTH;
    params.imageHeight       = Config::IMAGE_HEIGHT;
    params.cameraFps         = Config::CAMERA_FPS;
    params.blurKernelSize    = Config::BLUR_KERNEL_SIZE;
    params.hsv1Low           = {static_cast<double>(Config::HSV1_H_LOW),
                                static_cast<double>(Config::HSV1_S_LOW),
                                static_cast<double>(Config::HSV1_V_LOW)};
    params.hsv1High          = {static_cast<double>(Config::HSV1_H_HIGH),
                                static_cast<double>(Config::HSV1_S_HIGH),
                                static_cast<double>(Config::HSV1_V_HIGH)};
    params.hsv2Low           = {static_cast<double>(Config::HSV2_H_LOW),
                                static_cast<double>(Config::HSV2_S_LOW),
                                static_cast<double>(Config::HSV2_V_LOW)};
    params.hsv2High          = {static_cast<double>(Config::HSV2_H_HIGH),
                                static_cast<double>(Config::HSV2_S_HIGH),
                                static_cast<double>(Config::HSV2_V_HIGH)};
    params.morphOpenSize     = Config::MORPH_OPEN_SIZE;
    params.morphCloseSize    = Config::MORPH_CLOSE_SIZE;
    params.minContourArea    = Config::MIN_CONTOUR_AREA;
    params.gstreamerTemplate = Config::GSTREAMER_PIPELINE_TEMPLATE;
    params.rotate180         = Config::CAMERA_ROTATE_180;
#ifdef HARDWARE_MOCK
    params.useMockCamera     = true;
#else
    params.useMockCamera     = Config::HARDWARE_MOCK;
#endif
    return params;
}

Actuators::Servo_Params build_servo_params()
{
    Actuators::Servo_Params params;
    params.panGpio         = Config::SERVO_PAN_GPIO;
    params.tiltGpio        = Config::SERVO_TILT_GPIO;
    params.pwmFreqHz       = Config::SERVO_PWM_FREQ_HZ;
    params.panSafePulseMs  = Config::SERVO_PAN_SAFE_PULSE_MS;
    params.tiltSafePulseMs = Config::SERVO_TILT_SAFE_PULSE_MS;
    params.absMinMs        = Config::SERVO_ABS_MIN_MS;
    params.absMaxMs        = Config::SERVO_ABS_MAX_MS;
    return params;
}

void print_positions(double panMs, double tiltMs, double stepMs)
{
    std::cout << "[view_camera] Pan (X): "
              << Image_Processing::format_decimal(panMs)
              << " ms | Tilt (Y): "
              << Image_Processing::format_decimal(tiltMs)
              << " ms | Passo: "
              << Image_Processing::format_decimal(stepMs, 2) << " ms\n";
}

} // namespace

int main()
{
    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);

    std::cout << "=== Visualizador de Camera e Calibracao Interativa ===\n"
              << "Controles (com foco na janela da camera):\n"
              << "  W / S     : Tilt (Y) + / -\n"
              << "  A / D     : Pan  (X) - / +\n"
              << "  1 / 2 / 3 : Passo 0.01 / 0.05 / 0.10 ms\n"
              << "  C         : Centralizar (PID_X_BIAS_MS / PID_Y_BIAS_MS)\n"
              << "  P         : Imprimir pulsos atuais no terminal\n"
              << "  Q / ESC   : Sair\n\n";

    Actuators::Servo_Driver servoDriver(build_servo_params());
    const bool servosReady = servoDriver.initialize();

    double panMs  = clamp_pulse_ms(Config::PID_X_BIAS_MS);
    double tiltMs = clamp_pulse_ms(Config::PID_Y_BIAS_MS);
    double stepMs = 0.05;

    if (servosReady) {
        servoDriver.set_both_pulses(panMs, tiltMs);
        print_positions(panMs, tiltMs, stepMs);
    } else {
        std::cerr << "[view_camera] Aviso: servos indisponiveis "
                     "(execute com 'sudo ./view_camera' para movimentar o suporte).\n"
                     "[view_camera] Continuando apenas com a visualizacao da camera.\n\n";
    }

    Image_Processing::Image_Processor proc(build_image_params());
    if (!proc.initialize()) {
        std::cerr << "[view_camera] Erro ao inicializar a camera.\n";
        servoDriver.shutdown();
        return 1;
    }

    const cv::Size size = proc.get_frame_size();
    const cv::Point centerRef(size.width / 2, size.height / 2);

    cv::namedWindow("Alvo Detectado", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Mascara HSV", cv::WINDOW_AUTOSIZE);

    while (gRunning.load()) {
        auto meas = proc.process_frame();
        cv::Mat display = proc.get_last_frame().clone();
        cv::Mat mask    = proc.get_last_mask();

        if (display.empty()) {
            continue;
        }

        const std::string servoText =
            "Pan (X): " + Image_Processing::format_decimal(panMs) +
            " ms | Tilt (Y): " + Image_Processing::format_decimal(tiltMs) +
            " ms | Passo: " + Image_Processing::format_decimal(stepMs, 2) + " ms";

        Image_Processing::draw_tracking_overlay(
            display,
            meas,
            centerRef,
            servoText,
            "WASD: mover | 1/2/3: passo | C: centro | P: imprimir | Q: sair",
            servosReady ? "" : "Servos inativos (execute com sudo)");

        cv::imshow("Alvo Detectado", display);
        if (!mask.empty()) {
            cv::imshow("Mascara HSV", mask);
        }

        const int rawKey = cv::waitKey(1);
        if (rawKey < 0) {
            continue;
        }

        if (rawKey == 27) { // ESC
            break;
        }

        const char key = static_cast<char>(
            std::tolower(static_cast<unsigned char>(rawKey & 0xFF)));

        if (key == 'q') {
            break;
        }

        bool moved = false;

        switch (key) {
            case 'a':
                panMs = clamp_pulse_ms(panMs - stepMs);
                moved = true;
                break;
            case 'd':
                panMs = clamp_pulse_ms(panMs + stepMs);
                moved = true;
                break;
            case 'w':
                tiltMs = clamp_pulse_ms(tiltMs + stepMs);
                moved = true;
                break;
            case 's':
                tiltMs = clamp_pulse_ms(tiltMs - stepMs);
                moved = true;
                break;
            case '1':
                stepMs = 0.01;
                print_positions(panMs, tiltMs, stepMs);
                break;
            case '2':
                stepMs = 0.05;
                print_positions(panMs, tiltMs, stepMs);
                break;
            case '3':
                stepMs = 0.10;
                print_positions(panMs, tiltMs, stepMs);
                break;
            case 'c':
                panMs  = clamp_pulse_ms(Config::PID_X_BIAS_MS);
                tiltMs = clamp_pulse_ms(Config::PID_Y_BIAS_MS);
                moved  = true;
                break;
            case 'p':
                print_positions(panMs, tiltMs, stepMs);
                break;
            default:
                break;
        }

        if (moved) {
            if (servosReady) {
                servoDriver.set_both_pulses(panMs, tiltMs);
            }
            print_positions(panMs, tiltMs, stepMs);
        }
    }

    proc.shutdown();
    servoDriver.shutdown();
    cv::destroyAllWindows();
    return 0;
}
