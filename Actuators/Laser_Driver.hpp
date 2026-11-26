/**
 * @file Laser_Driver.hpp
 * @brief Interface do driver do módulo laser KY-008 via pigpio.
 *
 * O módulo laser KY-008 é acionado por um sinal digital simples:
 *   - Nível ALTO  (gpioWrite HIGH) → laser ligado.
 *   - Nível BAIXO (gpioWrite LOW)  → laser desligado.
 *
 * Pino utilizado:
 *   GPIO 26 — Pino físico 37 do header de 40 pinos do Raspberry Pi 4.
 *
 * IMPORTANTE: pigpio deve já ter sido inicializado pelo Servo_Driver antes
 * de criar um Laser_Driver. Este módulo chama apenas gpioSetMode() e
 * gpioWrite(), sem chamar gpioInitialise() / gpioTerminate() novamente.
 *
 * Dependência: libpigpio (sudo apt install libpigpio-dev pigpio)
 */

#pragma once

namespace Actuators {

/**
 * @brief Controla o módulo laser KY-008 via saída digital GPIO.
 *
 * Uso típico:
 * @code
 *   Laser_Driver laser(Config::LASER_GPIO);
 *   laser.initialize();   // configura o pino como saída (pigpio já inicializado)
 *
 *   laser.turn_on();      // liga o laser
 *   laser.turn_off();     // desliga o laser
 *
 *   laser.shutdown();     // garante que o laser está desligado ao encerrar
 * @endcode
 */
class Laser_Driver {
public:
    /**
     * @brief Constrói o driver para o pino GPIO informado.
     * @param gpioPin Número BCM do pino (padrão: Config::LASER_GPIO = 26).
     */
    explicit Laser_Driver(unsigned int gpioPin = 26);
    ~Laser_Driver();

    // Não copiável (recurso de hardware)
    Laser_Driver(const Laser_Driver&) = delete;
    Laser_Driver& operator=(const Laser_Driver&) = delete;

    /**
     * @brief Configura o pino como saída digital e garante estado inicial desligado.
     *
     * ATENÇÃO: pigpio deve já estar inicializado (gpioInitialise() chamado pelo
     * Servo_Driver) antes de chamar este método.
     *
     * @return true em caso de sucesso.
     */
    bool initialize();

    /**
     * @brief Liga o laser (nível ALTO no pino GPIO).
     * @return true se o comando foi enviado com sucesso.
     */
    bool turn_on();

    /**
     * @brief Desliga o laser (nível BAIXO no pino GPIO).
     * @return true se o comando foi enviado com sucesso.
     */
    bool turn_off();

    /**
     * @brief Desliga o laser e libera o estado interno.
     */
    void shutdown();

    /**
     * @brief Indica se o driver está inicializado e pronto para uso.
     */
    bool is_initialized() const;

    /**
     * @brief Retorna o estado atual do laser (true = ligado).
     */
    bool is_on() const;

private:
    unsigned int mGpioPin;
    bool mInitialized = false;
    bool mOn          = false;

    /**
     * @brief Envia nível lógico ao pino via pigpio.
     * @param level 1 = ALTO (ligado), 0 = BAIXO (desligado).
     * @return true em caso de sucesso.
     */
    bool write_gpio(unsigned int level);
};

} // namespace Actuators
