/**
 * @file view_camera.cpp
 * @brief Utilitário visual para visualização em tempo real da câmera e da detecção de fogo.
 *
 * Exibe duas janelas:
 *   1. "Alvo Detectado" - Imagem da câmera com a mira central e o centróide detectado.
 *   2. "Mascara HSV"    - Visão binária do filtro de fogo.
 *
 * Pressione 'q' ou 'ESC' na janela para sair.
 */

#include "Config/config.hpp"
#include "Image_Processing/Image_Processor.hpp"

#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

int main()
{
    std::cout << "=== Visualizador de Camera e Rastreamento ===\n"
              << "Pressione 'q' ou 'ESC' na janela para encerrar.\n\n";

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
    params.useMockCamera     = false;

    Image_Processing::Image_Processor proc(params);

    if (!proc.initialize()) {
        std::cerr << "Erro ao inicializar a camera.\n";
        return 1;
    }

    const cv::Size size = proc.get_frame_size();
    const cv::Point centerRef(size.width / 2, size.height / 2);

    cv::namedWindow("Alvo Detectado", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Mascara HSV", cv::WINDOW_AUTOSIZE);

    while (true) {
        auto meas = proc.process_frame();
        cv::Mat display = proc.get_last_frame().clone();
        cv::Mat mask    = proc.get_last_mask();

        if (display.empty()) {
            continue;
        }

        // Desenha a mira de centro (referência x_ref, y_ref em verde)
        cv::drawMarker(display, centerRef, cv::Scalar(0, 255, 0), cv::MARKER_CROSS, 30, 2);

        // Se detectou alvo, desenha circulo e coordenadas
        if (meas.valid) {
            cv::Point targetPt(meas.x, meas.y);
            cv::circle(display, targetPt, 12, cv::Scalar(0, 0, 255), 2);
            cv::circle(display, targetPt, 3, cv::Scalar(0, 0, 255), -1);

            std::string text = "Alvo: (" + std::to_string(meas.x) + ", " + std::to_string(meas.y) + ")";
            cv::putText(display, text, cv::Point(meas.x + 15, meas.y - 10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
        } else {
            cv::putText(display, "Alvo nao detectado", cv::Point(20, 30),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
        }

        cv::imshow("Alvo Detectado", display);
        if (!mask.empty()) {
            cv::imshow("Mascara HSV", mask);
        }

        char c = static_cast<char>(cv::waitKey(1));
        if (c == 'q' || c == 'Q' || c == 27) { // 27 = ESC
            break;
        }
    }

    proc.shutdown();
    cv::destroyAllWindows();
    return 0;
}
