/**
 * @file config.hpp
 * @brief Parâmetros centralizados de configuração do sistema pan-tilt.
 *
 * Todos os parâmetros ajustáveis são definidos aqui. Altere este arquivo
 * (ou o arquivo config.json em tempo de execução) sem recompilar o restante
 * do código.
 *
 * Convenção de nomes adotada no projeto:
 *   - Variáveis/atributos : camelCase  (ex.: imageWidth, errorX)
 *   - Classes/structs      : Palavra_Palavra  (ex.: Pid_Controller)
 *   - Métodos/funções      : palavra_palavra  (ex.: calculate_output)
 *   - Constantes           : SCREAMING_SNAKE_CASE
 */

#pragma once

#include <string>

namespace Config {

// ─────────────────────────────────────────────────────────────────────────────
// CÂMERA E IMAGEM
// ─────────────────────────────────────────────────────────────────────────────

/// Largura de captura em pixels (deve ser suportada pela câmera).
inline constexpr int IMAGE_WIDTH  = 640;

/// Altura de captura em pixels.
inline constexpr int IMAGE_HEIGHT = 480;

/// Taxa de quadros desejada (fps). Impacta a latência do loop de controle.
inline constexpr double CAMERA_FPS = 30.0;

/// Pipeline GStreamer para captura via libcamera (RPi OS Bookworm).
/// Parâmetros width/height/framerate são substituídos em tempo de execução
/// com base em IMAGE_WIDTH, IMAGE_HEIGHT e CAMERA_FPS.
inline const std::string GSTREAMER_PIPELINE_TEMPLATE =
    "libcamerasrc ! "
    "video/x-raw,width=%d,height=%d,framerate=%d/1,format=BGR ! "
    "appsink drop=true max-buffers=1";

// ─────────────────────────────────────────────────────────────────────────────
// PROCESSAMENTO DE IMAGEM — FILTRO HSV
// ─────────────────────────────────────────────────────────────────────────────

/// Suavização Gaussiana: tamanho do kernel (deve ser ímpar).
inline constexpr int BLUR_KERNEL_SIZE = 5;

// Faixa HSV primária (laranja-vermelho): H ∈ [0, 25]
inline constexpr int HSV1_H_LOW  =   0;
inline constexpr int HSV1_H_HIGH =  25;
inline constexpr int HSV1_S_LOW  = 100;
inline constexpr int HSV1_S_HIGH = 255;
inline constexpr int HSV1_V_LOW  = 100;
inline constexpr int HSV1_V_HIGH = 255;

// Faixa HSV secundária (vermelho-magenta wrap-around): H ∈ [170, 179]
inline constexpr int HSV2_H_LOW  = 170;
inline constexpr int HSV2_H_HIGH = 179;
inline constexpr int HSV2_S_LOW  = 100;
inline constexpr int HSV2_S_HIGH = 255;
inline constexpr int HSV2_V_LOW  = 100;
inline constexpr int HSV2_V_HIGH = 255;

/// Elemento morfológico de abertura (remove ruído pequeno).
inline constexpr int MORPH_OPEN_SIZE  = 3;

/// Elemento morfológico de fechamento (preenche buracos).
inline constexpr int MORPH_CLOSE_SIZE = 5;

/// Área mínima (px²) de um contorno para ser considerado fogo.
inline constexpr double MIN_CONTOUR_AREA = 100.0;

/// Tempo máximo (segundos) que uma medição pode ter antes de ser considerada
/// desatualizada pelo programa principal.
inline constexpr double MAX_MEASUREMENT_AGE_S = 0.2;

// ─────────────────────────────────────────────────────────────────────────────
// LOOP DE CONTROLE
// ─────────────────────────────────────────────────────────────────────────────

/// Período de amostragem do loop de controle em segundos (Ts).
/// Deve ser maior que o tempo típico de processamento de imagem.
/// Valor inicial: 50 ms → 20 Hz.
inline constexpr double SAMPLE_PERIOD_S = 0.05;

// ─────────────────────────────────────────────────────────────────────────────
// CONTROLADOR PID — EIXO X (PAN)
// ─────────────────────────────────────────────────────────────────────────────
/// ATENÇÃO: Os ganhos abaixo são PROVISÓRIOS para fins de compilação e teste
/// funcional. Devem ser calibrados experimentalmente antes da operação real.

inline constexpr double PID_X_KP   = 0.001;   ///< Ganho proporcional [ms/px]
inline constexpr double PID_X_KI   = 0.0001;  ///< Ganho integral     [ms/(px·s)]
inline constexpr double PID_X_KD   = 0.0005;  ///< Ganho derivativo   [ms·s/px]

/// Largura de pulso de referência (polarização) para o eixo pan [ms].
/// Valor inicial: 1.5 ms (posição neutra aproximada). Validar com o servo real.
inline constexpr double PID_X_BIAS_MS  = 1.5;

/// Limites da saída em ms (pulso mínimo e máximo para o servo pan).
inline constexpr double PID_X_OUT_MIN_MS = 1.0;
inline constexpr double PID_X_OUT_MAX_MS = 2.0;

/// Limite do acumulador integral (anti-windup por clamping).
inline constexpr double PID_X_INT_LIMIT_MS = 0.4;

/// Inversão de sentido do eixo X: true → multiplica o erro por -1.
inline constexpr bool PID_X_INVERT = false;

// ─────────────────────────────────────────────────────────────────────────────
// CONTROLADOR PID — EIXO Y (TILT)
// ─────────────────────────────────────────────────────────────────────────────
/// ATENÇÃO: Ganhos PROVISÓRIOS — calibrar experimentalmente.

inline constexpr double PID_Y_KP   = 0.001;
inline constexpr double PID_Y_KI   = 0.0001;
inline constexpr double PID_Y_KD   = 0.0005;

/// Largura de pulso de referência para o eixo tilt [ms].
inline constexpr double PID_Y_BIAS_MS  = 1.5;

/// Limites da saída em ms.
inline constexpr double PID_Y_OUT_MIN_MS = 1.0;
inline constexpr double PID_Y_OUT_MAX_MS = 2.0;

/// Limite do acumulador integral (anti-windup por clamping).
inline constexpr double PID_Y_INT_LIMIT_MS = 0.4;

/// Inversão de sentido do eixo Y: true → multiplica o erro por -1.
/// Em câmeras, o eixo Y cresce para baixo; pode ser necessário inverter.
inline constexpr bool PID_Y_INVERT = true;

// ─────────────────────────────────────────────────────────────────────────────
// ATUADORES — SERVOMOTORES MG90S via pigpio
// ─────────────────────────────────────────────────────────────────────────────

/// Pino GPIO BCM para o servo pan (eixo X).
/// GPIO 12 possui suporte a PWM de hardware no RPi 4.
inline constexpr unsigned int SERVO_PAN_GPIO  = 12;

/// Pino GPIO BCM para o servo tilt (eixo Y).
/// GPIO 13 possui suporte a PWM de hardware no RPi 4.
inline constexpr unsigned int SERVO_TILT_GPIO = 13;

/// Frequência do sinal PWM para servos RC [Hz].
/// 50 Hz é o padrão para servos analógicos como o MG90S (período = 20 ms).
inline constexpr unsigned int SERVO_PWM_FREQ_HZ = 50;

/// Largura de pulso segura aplicada ao encerrar o sistema [ms].
/// Posição neutra (≈ 90°). Validar com a montagem física.
inline constexpr double SERVO_SAFE_PULSE_MS = 1.5;

/// Largura de pulso mínima absoluta permitida pelo hardware [ms].
/// Protege o servo contra comandos fora de especificação.
inline constexpr double SERVO_ABS_MIN_MS = 0.8;

/// Largura de pulso máxima absoluta permitida pelo hardware [ms].
inline constexpr double SERVO_ABS_MAX_MS = 2.2;

// ─────────────────────────────────────────────────────────────────────────────
// MODO DE EXECUÇÃO
// ─────────────────────────────────────────────────────────────────────────────

/// Se true, executa sem acessar hardware real (câmera e GPIOs simulados).
/// Útil para testes no computador de desenvolvimento.
inline constexpr bool HARDWARE_MOCK = false;

} // namespace Config
