/**
 * @file Servo_Driver.hpp
 * @brief Interface do módulo de acionamento dos servomotores via pigpio.
 *
 * Responsabilidade exclusiva: converter larguras de pulso em ms em sinais
 * PWM para os pinos GPIO do Raspberry Pi 4, utilizando a biblioteca pigpio.
 *
 * pigpio gera PWM por DMA com resolução de 1 µs — suficiente para servos RC
 * como o MG90S (especificação: pulso 0,5–2,4 ms, período 20 ms / 50 Hz).
 *
 * Este módulo NÃO implementa lógica de controle nem calcula erros.
 *
 * Dependência: libpigpio (instalação: sudo apt install libpigpio-dev pigpio)
 * O daemon pigpiod deve estar em execução: sudo pigpiod
 *
 * Compatibilidade: Raspberry Pi 4, Raspberry Pi OS Bookworm (64-bit).
 *
 * IMPORTANTE: pigpio requer execução como root ou via daemon (pigpiod).
 * Use: sudo ./pantilt   ou   sudo pigpiod && ./pantilt
 */

#pragma once

#include <string>

namespace Actuators {

// ─────────────────────────────────────────────────────────────────────────────
// Parâmetros do driver
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Parâmetros de configuração do driver de servos.
 */
struct Servo_Params {
    unsigned int panGpio  = 12;  ///< GPIO BCM do servo pan  (eixo X)
    unsigned int tiltGpio = 13;  ///< GPIO BCM do servo tilt (eixo Y)

    unsigned int pwmFreqHz = 50; ///< Frequência PWM [Hz] — 50 Hz padrão RC

    double safePulseMs = 1.5;    ///< Pulso aplicado na inicialização e encerramento
    double absMinMs    = 0.8;    ///< Limite mínimo absoluto de pulso [ms]
    double absMaxMs    = 2.2;    ///< Limite máximo absoluto de pulso [ms]
};

// ─────────────────────────────────────────────────────────────────────────────
// Driver de servos
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Controla dois servomotores MG90S via pigpio (GPIO do RPi 4).
 *
 * Uso típico:
 * @code
 *   Servo_Params p;
 *   Servo_Driver drv(p);
 *   if (!drv.initialize()) { return -1; }
 *
 *   drv.set_pan_pulse(1.3);   // move pan
 *   drv.set_tilt_pulse(1.7);  // move tilt
 *
 *   drv.shutdown();  // aplica pulso seguro e libera GPIO
 * @endcode
 */
class Servo_Driver {
public:
    explicit Servo_Driver(Servo_Params params = {});
    ~Servo_Driver();

    // Não copiável (recursos de hardware)
    Servo_Driver(const Servo_Driver&) = delete;
    Servo_Driver& operator=(const Servo_Driver&) = delete;

    /**
     * @brief Inicializa pigpio e configura os pinos.
     *
     * Aplica o pulso seguro (safePulseMs) em ambos os servos após a init.
     *
     * @return true em caso de sucesso.
     */
    bool initialize();

    /**
     * @brief Envia pulso ao servo pan e aguarda a próxima chamada.
     *
     * @param pulseMs Largura de pulso [ms], validada antes do envio.
     * @return true se o comando foi aceito e enviado.
     */
    bool set_pan_pulse(double pulseMs);

    /**
     * @brief Envia pulso ao servo tilt.
     *
     * @param pulseMs Largura de pulso [ms].
     * @return true se o comando foi aceito e enviado.
     */
    bool set_tilt_pulse(double pulseMs);

    /**
     * @brief Envia pulsos para ambos os servos em uma única chamada.
     *
     * @return true somente se ambos os comandos foram aceitos.
     */
    bool set_both_pulses(double panMs, double tiltMs);

    /**
     * @brief Aplica o pulso seguro e libera os recursos de hardware.
     */
    void shutdown();

    /**
     * @brief Indica se o driver foi inicializado com sucesso.
     */
    bool is_initialized() const;

    /**
     * @brief Retorna a largura de pulso mais recente enviada ao pan [ms].
     */
    double get_pan_pulse_ms() const;

    /**
     * @brief Retorna a largura de pulso mais recente enviada ao tilt [ms].
     */
    double get_tilt_pulse_ms() const;

private:
    Servo_Params mParams;
    bool   mInitialized  = false;
    double mPanPulseMs   = 0.0;
    double mTiltPulseMs  = 0.0;

    /**
     * @brief Valida e converte ms → µs (unidade do pigpio).
     *
     * @param pulseMs Largura de pulso em ms.
     * @param[out] pulseUs Largura de pulso em µs (para uso no pigpio).
     * @return true se o valor é válido.
     */
    bool validate_and_convert(double pulseMs, unsigned int& pulseUs) const;

    /**
     * @brief Envia um pulso a um pino GPIO via pigpio.
     *
     * @param gpio   Número do pino (BCM).
     * @param pulseUs Largura de pulso em µs.
     * @return true em caso de sucesso.
     */
    bool send_pulse(unsigned int gpio, unsigned int pulseUs);
};

} // namespace Actuators
