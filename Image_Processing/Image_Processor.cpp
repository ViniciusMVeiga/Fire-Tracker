/**
 * @file Image_Processor.cpp
 * @brief Implementação do módulo de processamento de imagem.
 *
 * Pipeline preservado do código original (detectar_fogo.cpp):
 *   1. Captura de frame via VideoCapture
 *   2. Suavização Gaussiana (GaussianBlur)
 *   3. Conversão BGR → HSV (cvtColor)
 *   4. Filtragem HSV: faixa 1 (laranja) + faixa 2 (vermelho wrap-around)
 *      combinadas por OR binário (bitwise_or)
 *   5. Morfologia: abertura (MORPH_OPEN) + fechamento (MORPH_CLOSE)
 *   6. Detecção de contornos (findContours, RETR_EXTERNAL)
 *   7. Seleção do maior contorno com área > minContourArea
 *   8. Centróide por momentos de imagem (moments → m10/m00, m01/m00)
 *
 * Alterações em relação ao original:
 *   - Entrada substituída de imread(arquivo) para VideoCapture (câmera).
 *   - Pipeline encapsulado em métodos privados step_*().
 *   - Timestamp adicionado ao resultado para detecção de medições antigas.
 *   - Verificação das dimensões efetivas do frame.
 *   - Suporte a modo simulado (useMockCamera) para testes sem RPi.
 */

#include "Image_Processor.hpp"

#include <cstdio>       // snprintf
#include <stdexcept>
#include <iostream>

#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

namespace Image_Processing {

// ─────────────────────────────────────────────────────────────────────────────
// Construtor / Destrutor
// ─────────────────────────────────────────────────────────────────────────────

Image_Processor::Image_Processor(Detection_Params params)
    : mParams(std::move(params))
{}

Image_Processor::~Image_Processor()
{
    shutdown();
}

// ─────────────────────────────────────────────────────────────────────────────
// Inicialização
// ─────────────────────────────────────────────────────────────────────────────

bool Image_Processor::initialize()
{
    if (mInitialized) {
        std::cerr << "[Image_Processor] Já inicializado.\n";
        return true;
    }

    if (mParams.useMockCamera) {
        // Modo simulado: tenta abrir /dev/video0 (webcam USB) ou índice 0.
        std::cout << "[Image_Processor] Modo mock: abrindo /dev/video0\n";
        mCapture.open(0, cv::CAP_V4L2);

        if (!mCapture.isOpened()) {
            // Em ambiente sem qualquer câmera (CI, testes puros) não é possível
            // abrir captura. Retornamos false para que o chamador trate.
            std::cerr << "[Image_Processor] Não foi possível abrir câmera mock.\n";
            return false;
        }

        // Configura resolução desejada
        mCapture.set(cv::CAP_PROP_FRAME_WIDTH,  mParams.imageWidth);
        mCapture.set(cv::CAP_PROP_FRAME_HEIGHT, mParams.imageHeight);
        mCapture.set(cv::CAP_PROP_FPS,          mParams.cameraFps);
    } else {
        // Modo real: captura via V4L2 com backend libcamera.
        // No Raspberry Pi OS Trixie o OpenCV instalado via apt não inclui
        // suporte a GStreamer; usamos CAP_V4L2 diretamente, que funciona com
        // a camada de compatibilidade libcamera-v4l2 (/dev/video0).
        std::cout << "[Image_Processor] Abrindo camera via V4L2 (/dev/video0)...\n";

        mCapture.open(0, cv::CAP_V4L2);

        if (!mCapture.isOpened()) {
            std::cerr << "[Image_Processor] Falha ao abrir /dev/video0 via V4L2.\n"
                      << "  Verifique se a camera esta habilitada (raspi-config)\n"
                      << "  e se /dev/video0 existe (ls /dev/video*).\n";
            return false;
        }

        // Solicita a resolução configurada (o driver pode ajustar para o
        // modo suportado mais próximo — a resolução efetiva é lida logo abaixo)
        mCapture.set(cv::CAP_PROP_FRAME_WIDTH,  mParams.imageWidth);
        mCapture.set(cv::CAP_PROP_FRAME_HEIGHT, mParams.imageHeight);
        mCapture.set(cv::CAP_PROP_FPS,          mParams.cameraFps);
    }

    // Verifica dimensões efetivas
    const int effW = static_cast<int>(mCapture.get(cv::CAP_PROP_FRAME_WIDTH));
    const int effH = static_cast<int>(mCapture.get(cv::CAP_PROP_FRAME_HEIGHT));

    if (effW != mParams.imageWidth || effH != mParams.imageHeight) {
        std::cerr << "[Image_Processor] AVISO: Resolução solicitada ("
                  << mParams.imageWidth << "x" << mParams.imageHeight
                  << ") difere da efetiva (" << effW << "x" << effH << ").\n"
                  << "  As coordenadas serão expressas na resolução efetiva.\n";
        // Atualiza params para refletir a resolução real
        mParams.imageWidth  = effW;
        mParams.imageHeight = effH;
    }

    std::cout << "[Image_Processor] Câmera aberta: "
              << mParams.imageWidth << "x" << mParams.imageHeight << " @ "
              << mParams.cameraFps << " fps\n";

    mInitialized = true;
    return true;
}

void Image_Processor::shutdown()
{
    if (mCapture.isOpened()) {
        mCapture.release();
    }
    mInitialized = false;
}

// ─────────────────────────────────────────────────────────────────────────────
// Consultas de estado
// ─────────────────────────────────────────────────────────────────────────────

bool Image_Processor::is_open() const
{
    return mCapture.isOpened();
}

cv::Size Image_Processor::get_frame_size() const
{
    if (!mInitialized) {
        return {0, 0};
    }
    return {mParams.imageWidth, mParams.imageHeight};
}

const cv::Mat& Image_Processor::get_last_frame() const
{
    return mLastFrame;
}

const cv::Mat& Image_Processor::get_last_mask() const
{
    return mLastMask;
}

// ─────────────────────────────────────────────────────────────────────────────
// Pipeline principal
// ─────────────────────────────────────────────────────────────────────────────

Target_Measurement Image_Processor::process_frame()
{
    Target_Measurement result;
    result.valid     = false;
    result.timestamp = std::chrono::steady_clock::now();

    if (!mInitialized || !mCapture.isOpened()) {
        std::cerr << "[Image_Processor] process_frame: câmera não inicializada.\n";
        return result;
    }

    // 1. Captura frame
    cv::Mat frame;
    if (!mCapture.read(frame) || frame.empty()) {
        std::cerr << "[Image_Processor] Falha ao capturar frame.\n";
        return result;
    }

    mLastFrame = frame.clone();

    const int frameW = frame.cols;
    const int frameH = frame.rows;

    result.frameWidth  = frameW;
    result.frameHeight = frameH;

    // 2. Suavização Gaussiana  (etapa preservada do código original)
    const cv::Mat blurred = step_blur(frame);

    // 3. Filtragem HSV  (etapas 3 + 4 preservadas)
    const cv::Mat mask = step_hsv_filter(blurred);

    // 4. Morfologia  (etapa 5 preservada)
    const cv::Mat morphed = step_morphology(mask);
    mLastMask = morphed.clone();

    // 5. Encontra alvo  (etapas 6 + 7 + 8 preservadas)
    result = step_find_target(morphed, frameW, frameH);
    result.timestamp = std::chrono::steady_clock::now();

    return result;
}

// ─────────────────────────────────────────────────────────────────────────────
// Etapas internas do pipeline
// ─────────────────────────────────────────────────────────────────────────────

/**
 * Etapa 2: Suavização Gaussiana.
 * Idêntica ao original: GaussianBlur com kernel NxN.
 */
cv::Mat Image_Processor::step_blur(const cv::Mat& frame) const
{
    cv::Mat blurred;
    const int k = mParams.blurKernelSize;
    cv::GaussianBlur(frame, blurred, cv::Size(k, k), 0);
    return blurred;
}

/**
 * Etapas 3 + 4: Conversão para HSV e filtragem de cor.
 * Idêntica ao original: duas faixas HSV combinadas por OR binário.
 */
cv::Mat Image_Processor::step_hsv_filter(const cv::Mat& blurred) const
{
    cv::Mat hsv;
    cv::cvtColor(blurred, hsv, cv::COLOR_BGR2HSV);

    cv::Mat mask1, mask2, mask;
    cv::inRange(hsv, mParams.hsv1Low, mParams.hsv1High, mask1);
    cv::inRange(hsv, mParams.hsv2Low, mParams.hsv2High, mask2);
    cv::bitwise_or(mask1, mask2, mask);

    return mask;
}

/**
 * Etapa 5: Operações morfológicas.
 * Idêntica ao original: abertura (remove ruído) seguida de fechamento (preenche).
 */
cv::Mat Image_Processor::step_morphology(const cv::Mat& mask) const
{
    cv::Mat result = mask.clone();

    const cv::Mat elemOpen = cv::getStructuringElement(
        cv::MORPH_ELLIPSE,
        cv::Size(mParams.morphOpenSize, mParams.morphOpenSize));

    const cv::Mat elemClose = cv::getStructuringElement(
        cv::MORPH_ELLIPSE,
        cv::Size(mParams.morphCloseSize, mParams.morphCloseSize));

    cv::morphologyEx(result, result, cv::MORPH_OPEN,  elemOpen);
    cv::morphologyEx(result, result, cv::MORPH_CLOSE, elemClose);

    return result;
}

/**
 * Etapas 6 + 7 + 8: Contornos, seleção e centróide.
 * Lógica preservada do original. Retorna medição inválida se não detectar.
 */
Target_Measurement Image_Processor::step_find_target(
    const cv::Mat& mask, int frameW, int frameH) const
{
    Target_Measurement result;
    result.frameWidth  = frameW;
    result.frameHeight = frameH;

    // Etapa 6: encontra contornos externos
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask.clone(), contours,
                     cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // Etapa 7: seleciona o maior contorno
    int    bestIndex = -1;
    double bestArea  = 0.0;

    for (int i = 0; i < static_cast<int>(contours.size()); ++i) {
        const double area = cv::contourArea(contours[i]);
        if (area > bestArea) {
            bestArea  = area;
            bestIndex = i;
        }
    }

    if (bestIndex == -1 || bestArea < mParams.minContourArea) {
        // Nenhuma região significativa de fogo detectada
        result.valid = false;
        return result;
    }

    // Etapa 8: calcula centróide por momentos de imagem
    const cv::Moments m = cv::moments(contours[bestIndex]);

    if (m.m00 == 0.0) {
        result.valid = false;
        return result;
    }

    result.valid = true;
    result.x     = static_cast<int>(m.m10 / m.m00);
    result.y     = static_cast<int>(m.m01 / m.m00);
    result.area  = bestArea;

    return result;
}

// ─────────────────────────────────────────────────────────────────────────────
// Utilitários privados
// ─────────────────────────────────────────────────────────────────────────────

std::string Image_Processor::build_pipeline() const
{
    char buf[512];
    std::snprintf(buf, sizeof(buf),
                  mParams.gstreamerTemplate.c_str(),
                  mParams.imageWidth,
                  mParams.imageHeight,
                  static_cast<int>(mParams.cameraFps));
    return std::string(buf);
}

} // namespace Image_Processing
