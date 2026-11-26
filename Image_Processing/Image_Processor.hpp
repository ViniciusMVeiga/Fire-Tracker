/**
 * @file Image_Processor.hpp
 * @brief Interface do módulo de processamento de imagem.
 *
 * Encapsula a aquisição de imagens da Raspberry Pi Camera (via OpenCV +
 * GStreamer/libcamera) e o pipeline de detecção de fogo por filtragem HSV.
 *
 * Pipeline (preservado do código original detectar_fogo.cpp):
 *   1. Captura de frame via VideoCapture (GStreamer → libcamera)
 *   2. Suavização Gaussiana
 *   3. Conversão BGR → HSV
 *   4. Filtragem por faixas HSV do fogo (laranja + vermelho wrap-around)
 *   5. Operações morfológicas (abertura + fechamento)
 *   6. Detecção de contornos externos
 *   7. Seleção do maior contorno (área mínima configurável)
 *   8. Cálculo do centróide por momentos de imagem
 *
 * Alterações em relação ao código original:
 *   - Fonte de entrada substituída de arquivo JPG para Raspberry Pi Camera
 *     (VideoCapture com pipeline GStreamer + libcamera), preservando todas
 *     as etapas 2–8 inalteradas.
 *   - Pipeline encapsulado em classe reutilizável com interface limpa.
 *   - Suporte a modo simulado (HARDWARE_MOCK) para testes sem câmera.
 *   - Resultado exposto via struct Target_Measurement com timestamp,
 *     coordenadas e flag de validade — sem retornar silenciosamente
 *     coordenadas antigas.
 *   - Resolução configurável por parâmetro, confrontada com as dimensões
 *     efetivas do frame capturado.
 */

#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <string>
#include <optional>
#include <thread>
#include <sys/types.h>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>

namespace Image_Processing {

// ─────────────────────────────────────────────────────────────────────────────
// Estruturas de dados
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Parâmetros do pipeline de detecção.
 *
 * Todos os campos têm valores padrão derivados de Config::*, podendo ser
 * sobrescritos sem recompilar — basta construir com valores diferentes.
 */
struct Detection_Params {
    // Dimensões configuradas (serão confrontadas com as do frame real)
    int imageWidth  = 640;
    int imageHeight = 480;
    double cameraFps = 30.0;

    // Suavização
    int blurKernelSize = 5;     ///< Tamanho do kernel Gaussiano (ímpar)

    // Faixas HSV — fogo laranja/vermelho
    cv::Scalar hsv1Low  = {  0, 100, 100};
    cv::Scalar hsv1High = { 25, 255, 255};
    cv::Scalar hsv2Low  = {170, 100, 100};
    cv::Scalar hsv2High = {179, 255, 255};

    // Morfologia
    int morphOpenSize  = 3;
    int morphCloseSize = 5;

    // Filtragem de contornos
    double minContourArea = 100.0;   ///< Área mínima [px²]

    // Pipeline GStreamer (template com %d para width, height, fps)
    std::string gstreamerTemplate =
        "libcamerasrc ! "
        "video/x-raw,width=%d,height=%d,framerate=%d/1,format=BGR ! "
        "appsink drop=true max-buffers=1";

    /// Se true, rotaciona a imagem em 180° (câmera de cabeça para baixo).
    bool rotate180 = false;

    /// Se true, usa webcam /dev/video0 em vez do pipeline GStreamer.
    /// Útil para desenvolvimento sem câmera RPi.
    bool useMockCamera = false;
};

/**
 * @brief Resultado de uma iteração do pipeline de detecção.
 */
struct Target_Measurement {
    bool valid = false;   ///< true somente quando o alvo foi detectado

    int x = 0;            ///< Coordenada X do centróide [px]
    int y = 0;            ///< Coordenada Y do centróide [px]
    double area = 0.0;    ///< Área do maior contorno detectado [px²]

    /// Instante em que a medição foi produzida.
    std::chrono::steady_clock::time_point timestamp{};

    /// Dimensões efetivas do frame (podem diferir das configuradas).
    int frameWidth  = 0;
    int frameHeight = 0;
};

// ─────────────────────────────────────────────────────────────────────────────
// Classe principal
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Encapsula aquisição de imagem e pipeline de detecção de fogo.
 *
 * Uso típico:
 * @code
 *   Detection_Params params;
 *   Image_Processor proc(params);
 *   if (!proc.initialize()) { ... }
 *
 *   Target_Measurement m = proc.process_frame();
 *   if (m.valid) { use(m.x, m.y); }
 * @endcode
 */
class Image_Processor {
public:
    explicit Image_Processor(Detection_Params params = {});
    ~Image_Processor();

    // Não copiável (VideoCapture não é copiável)
    Image_Processor(const Image_Processor&) = delete;
    Image_Processor& operator=(const Image_Processor&) = delete;

    /**
     * @brief Abre a câmera e valida as dimensões do frame.
     * @return true em caso de sucesso.
     */
    bool initialize();

    /**
     * @brief Libera recursos da câmera.
     */
    void shutdown();

    /**
     * @brief Captura um frame e executa o pipeline de detecção.
     *
     * Sempre produz uma medição com timestamp atual. Se o alvo não for
     * detectado, retorna medição inválida (valid = false).
     *
     * @return Target_Measurement com resultado da detecção.
     */
    Target_Measurement process_frame();

    /**
     * @brief Retorna as dimensões efetivas dos frames capturados.
     *
     * Disponível somente após initialize(). Retorna {0,0} antes disso.
     */
    cv::Size get_frame_size() const;

    /**
     * @brief Indica se a câmera foi aberta com sucesso.
     */
    bool is_open() const;

    /**
     * @brief Expõe o frame bruto mais recente para debug ou exibição.
     * @return Referência constante ao último frame. Vazio se nenhum foi capturado.
     */
    const cv::Mat& get_last_frame() const;

    /**
     * @brief Expõe a máscara binária mais recente para debug.
     */
    const cv::Mat& get_last_mask() const;

private:
    Detection_Params  mParams;
    cv::VideoCapture  mCapture;

    cv::Mat mLastFrame;   ///< Último frame bruto (BGR)
    cv::Mat mLastMask;    ///< Última máscara binária

    bool mInitialized = false;

    // Captura nativa Raspberry Pi via pipe (rpicam-vid / libcamera-vid)
    bool                    mUsingRpicamPipe = false;
    pid_t                   mCameraPid       = -1;
    int                     mPipeFd          = -1;
    std::thread             mReaderThread;
    std::atomic<bool>       mStopping{false};
    std::mutex              mFrameMutex;
    std::condition_variable mFrameCv;
    cv::Mat                 mPendingFrame;
    bool                    mHasNewFrame     = false;
    bool                    mReaderEnded     = false;
    std::chrono::steady_clock::time_point mPendingTimestamp{};

    bool start_rpicam_pipe();
    void stop_rpicam_pipe();
    void reader_loop();

    /// Monta a string do pipeline GStreamer com as dimensões configuradas.
    std::string build_pipeline() const;

    /// Etapas internas do pipeline (preservadas do código original).
    cv::Mat step_blur(const cv::Mat& frame) const;
    cv::Mat step_hsv_filter(const cv::Mat& blurred) const;
    cv::Mat step_morphology(const cv::Mat& mask) const;
    Target_Measurement step_find_target(const cv::Mat& mask,
                                        int frameW, int frameH) const;
};

} // namespace Image_Processing
