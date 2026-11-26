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

#include <cerrno>
#include <csignal>
#include <cstdio>       // snprintf
#include <cstdlib>      // getenv
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <vector>

#include <sys/prctl.h>
#include <sys/wait.h>
#include <unistd.h>

#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

namespace Image_Processing {

namespace {

std::string find_executable_in_path(const char* name)
{
    const char* pathEnv = std::getenv("PATH");
    const std::string searchPath = pathEnv ? pathEnv : "/usr/local/bin:/usr/bin:/bin";

    std::stringstream ss(searchPath);
    std::string dir;
    while (std::getline(ss, dir, ':')) {
        if (dir.empty()) {
            continue;
        }
        const std::string candidate = dir + "/" + name;
        if (::access(candidate.c_str(), X_OK) == 0) {
            return candidate;
        }
    }
    return {};
}

std::string find_rpicam_executable()
{
    std::string exe = find_executable_in_path("rpicam-vid");
    if (!exe.empty()) {
        return exe;
    }
    return find_executable_in_path("libcamera-vid");
}

bool read_exact(int fd, unsigned char* dst, std::size_t totalBytes)
{
    std::size_t offset = 0;
    while (offset < totalBytes) {
        const ssize_t n = ::read(fd, dst + offset, totalBytes - offset);
        if (n > 0) {
            offset += static_cast<std::size_t>(n);
        } else if (n == 0) {
            return false; // EOF
        } else if (errno != EINTR) {
            return false;
        }
    }
    return true;
}

} // namespace

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
        // Modo real (Raspberry Pi 4 + libcamera):
        // Usa rpicam-vid (ou libcamera-vid) em pipe YUV420 diretamente,
        // evitando falhas de V4L2 (/dev/video0 unicam) e ausência de GStreamer
        // no OpenCV empacotado pelo apt.
        if (start_rpicam_pipe()) {
            std::cout << "[Image_Processor] Câmera aberta via rpicam-vid/libcamera-vid: "
                      << mParams.imageWidth << "x" << mParams.imageHeight << " @ "
                      << mParams.cameraFps << " fps\n";
            mInitialized = true;
            return true;
        }

        // Fallback: tenta GStreamer e por fim V4L2
        std::cout << "[Image_Processor] Tentando fallback via GStreamer / V4L2...\n";
        mCapture.open(build_pipeline(), cv::CAP_GSTREAMER);

        if (!mCapture.isOpened()) {
            mCapture.open(0, cv::CAP_V4L2);
        }

        if (!mCapture.isOpened()) {
            std::cerr << "[Image_Processor] Falha ao abrir câmera (rpicam-vid, GStreamer e V4L2).\n";
            return false;
        }

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
    stop_rpicam_pipe();
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
    if (mUsingRpicamPipe) {
        return mInitialized && !mReaderEnded;
    }
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

    if (!mInitialized || !is_open()) {
        std::cerr << "[Image_Processor] process_frame: câmera não inicializada.\n";
        return result;
    }

    // 1. Captura frame
    cv::Mat frame;
    auto captureTimestamp = std::chrono::steady_clock::now();

    if (mUsingRpicamPipe) {
        std::unique_lock<std::mutex> lock(mFrameMutex);
        const bool gotFrame = mFrameCv.wait_for(
            lock, std::chrono::milliseconds(1000),
            [this]() {
                return mHasNewFrame || mReaderEnded || mStopping.load();
            });

        if (!gotFrame || !mHasNewFrame || mPendingFrame.empty()) {
            std::cerr << "[Image_Processor] Falha ao capturar frame (pipe rpicam-vid).\n";
            return result;
        }

        frame            = std::move(mPendingFrame);
        captureTimestamp = mPendingTimestamp;
        mHasNewFrame     = false;
    } else {
        if (!mCapture.read(frame) || frame.empty()) {
            std::cerr << "[Image_Processor] Falha ao capturar frame.\n";
            return result;
        }
        if (mParams.rotate180) {
            cv::rotate(frame, frame, cv::ROTATE_180);
        }
        captureTimestamp = std::chrono::steady_clock::now();
    }

    mLastFrame = frame;

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
    mLastMask = morphed;

    // 5. Encontra alvo  (etapas 6 + 7 + 8 preservadas)
    result = step_find_target(morphed, frameW, frameH);
    result.timestamp = captureTimestamp;

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
    cv::Mat opened, closed;

    const cv::Mat elemOpen = cv::getStructuringElement(
        cv::MORPH_ELLIPSE,
        cv::Size(mParams.morphOpenSize, mParams.morphOpenSize));

    const cv::Mat elemClose = cv::getStructuringElement(
        cv::MORPH_ELLIPSE,
        cv::Size(mParams.morphCloseSize, mParams.morphCloseSize));

    cv::morphologyEx(mask,   opened, cv::MORPH_OPEN,  elemOpen);
    cv::morphologyEx(opened, closed, cv::MORPH_CLOSE, elemClose);

    return closed;
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

    // Etapa 6: encontra contornos externos (OpenCV 4 não altera a matriz fonte)
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours,
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

bool Image_Processor::start_rpicam_pipe()
{
    if (mParams.imageWidth <= 0 || mParams.imageHeight <= 0 ||
        (mParams.imageWidth % 2) != 0 || (mParams.imageHeight % 2) != 0) {
        std::cerr << "[Image_Processor] Dimensões inválidas para YUV420: "
                  << mParams.imageWidth << "x" << mParams.imageHeight << "\n";
        return false;
    }

    const std::string exePath = find_rpicam_executable();
    if (exePath.empty()) {
        std::cerr << "[Image_Processor] rpicam-vid/libcamera-vid não encontrado no PATH.\n";
        return false;
    }

    int pipefd[2] = {-1, -1};
    if (::pipe(pipefd) != 0) {
        std::cerr << "[Image_Processor] Falha ao criar pipe para câmera.\n";
        return false;
    }

    const pid_t parentPid = ::getpid();
    const pid_t pid = ::fork();
    if (pid < 0) {
        std::cerr << "[Image_Processor] Falha no fork() para iniciar " << exePath << ".\n";
        ::close(pipefd[0]);
        ::close(pipefd[1]);
        return false;
    }

    if (pid == 0) {
        ::prctl(PR_SET_PDEATHSIG, SIGTERM);
        if (::getppid() != parentPid) {
            ::_exit(127);
        }

        ::dup2(pipefd[1], STDOUT_FILENO);
        ::close(pipefd[0]);
        ::close(pipefd[1]);

        std::vector<std::string> args = {
            exePath,
            "--nopreview",
            "-t", "0",
            "--width", std::to_string(mParams.imageWidth),
            "--height", std::to_string(mParams.imageHeight),
            "--framerate", std::to_string(static_cast<int>(mParams.cameraFps)),
            "--codec", "yuv420",
            "--flush",
            "-o", "-"
        };
        if (mParams.rotate180) {
            args.push_back("--rotation");
            args.push_back("180");
        }
        std::vector<char*> argv;
        argv.reserve(args.size() + 1);
        for (auto& arg : args) {
            argv.push_back(arg.data());
        }
        argv.push_back(nullptr);

        ::execv(exePath.c_str(), argv.data());
        ::_exit(127);
    }

    ::close(pipefd[1]);

    {
        std::lock_guard<std::mutex> lock(mFrameMutex);
        mPendingFrame.release();
        mHasNewFrame = false;
        mReaderEnded = false;
    }

    mStopping.store(false);
    mCameraPid       = pid;
    mPipeFd          = pipefd[0];
    mUsingRpicamPipe = true;

    std::cout << "[Image_Processor] Iniciando captura via " << exePath << "...\n";
    mReaderThread = std::thread(&Image_Processor::reader_loop, this);

    // Aguarda o primeiro frame para garantir que a câmera está realmente entregando imagens
    std::unique_lock<std::mutex> lock(mFrameMutex);
    const bool ready = mFrameCv.wait_for(
        lock, std::chrono::seconds(5),
        [this]() { return mHasNewFrame || mReaderEnded; });

    if (!ready || !mHasNewFrame) {
        lock.unlock();
        std::cerr << "[Image_Processor] Timeout ou falha ao receber primeiro frame de "
                  << exePath << ".\n";
        stop_rpicam_pipe();
        return false;
    }

    return true;
}

void Image_Processor::stop_rpicam_pipe()
{
    mStopping.store(true);
    mFrameCv.notify_all();

    if (mCameraPid > 0) {
        ::kill(mCameraPid, SIGTERM);
        int status = 0;
        bool reaped = false;
        for (int i = 0; i < 50; ++i) {
            const pid_t res = ::waitpid(mCameraPid, &status, WNOHANG);
            if (res == mCameraPid || res < 0) {
                reaped = true;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        if (!reaped) {
            ::kill(mCameraPid, SIGKILL);
            ::waitpid(mCameraPid, &status, 0);
        }
        mCameraPid = -1;
    }

    if (mReaderThread.joinable()) {
        mReaderThread.join();
    }

    if (mPipeFd >= 0) {
        ::close(mPipeFd);
        mPipeFd = -1;
    }

    mUsingRpicamPipe = false;
}

void Image_Processor::reader_loop()
{
    const std::size_t frameBytes =
        static_cast<std::size_t>(mParams.imageWidth) *
        static_cast<std::size_t>(mParams.imageHeight) * 3u / 2u;

    std::vector<unsigned char> buffer(frameBytes);

    while (!mStopping.load()) {
        if (!read_exact(mPipeFd, buffer.data(), frameBytes)) {
            break;
        }

        cv::Mat yuv(mParams.imageHeight + mParams.imageHeight / 2,
                    mParams.imageWidth, CV_8UC1, buffer.data());
        cv::Mat bgr;
        cv::cvtColor(yuv, bgr, cv::COLOR_YUV2BGR_I420);

        {
            std::lock_guard<std::mutex> lock(mFrameMutex);
            mPendingFrame     = std::move(bgr);
            mPendingTimestamp = std::chrono::steady_clock::now();
            mHasNewFrame      = true;
        }
        mFrameCv.notify_one();
    }

    {
        std::lock_guard<std::mutex> lock(mFrameMutex);
        mReaderEnded = true;
    }
    mFrameCv.notify_all();
}

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
