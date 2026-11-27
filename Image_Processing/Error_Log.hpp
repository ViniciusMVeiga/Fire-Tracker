/**
 * @file Error_Log.hpp
 * @brief Registro em memória das amostras de erro do PID e exportação em CSV.
 *
 * Cada ciclo do loop de controle produz uma linha (mesmo quando o alvo não foi
 * detectado, para preservar as falhas do gráfico). O arquivo gerado contém
 * metadados da configuração em linhas comentadas, seguidos do cabeçalho e das
 * amostras, permitindo recriar os gráficos de erro offline.
 */

#pragma once

#include <cstddef>
#include <fstream>
#include <iomanip>
#include <string>
#include <vector>

namespace Image_Processing {

/**
 * @brief Uma amostra do loop de controle.
 */
struct Error_Sample {
    long  sample   = 0;      ///< Índice da amostra dentro da sessão
    double timeS   = 0.0;    ///< Tempo desde o início da sessão [s]
    bool   valid   = false;  ///< true se a medição foi válida
    double errorX  = 0.0;    ///< Erro do pan [px]
    double errorY  = 0.0;    ///< Erro do tilt [px]
    int    targetX = 0;      ///< Coordenada X do alvo [px]
    int    targetY = 0;      ///< Coordenada Y do alvo [px]
    double refX    = 0.0;    ///< Referência X (mira) [px]
    double refY    = 0.0;    ///< Referência Y (mira) [px]
    double panMs   = 0.0;    ///< Largura de pulso do pan [ms]
    double tiltMs  = 0.0;    ///< Largura de pulso do tilt [ms]
    bool   panSat  = false;  ///< Saída do pan saturada
    bool   tiltSat = false;  ///< Saída do tilt saturada
};

/**
 * @brief Configuração da sessão, gravada como metadados no CSV.
 */
struct Error_Log_Metadata {
    std::string timestamp;        ///< Data/hora da gravação (nome do arquivo)
    int    frameWidth  = 0;
    int    frameHeight = 0;
    double kpX = 0.0, kiX = 0.0, kdX = 0.0;
    double kpY = 0.0, kiY = 0.0, kdY = 0.0;
    double samplePeriodMs = 0.0;
    double aimOffsetX = 0.0;
    double aimOffsetY = 0.0;
    bool   invertX = false;
    bool   invertY = false;
};

/**
 * @brief Acumulador de amostras com exportação para CSV.
 */
class Error_Log {
public:
    /// Número máximo de amostras em memória (evita crescimento sem limite).
    static std::size_t max_samples()
    {
        return 500000UL;
    }

    void clear()
    {
        mSamples.clear();
    }

    bool empty() const
    {
        return mSamples.empty();
    }

    std::size_t size() const
    {
        return mSamples.size();
    }

    bool full() const
    {
        return mSamples.size() >= max_samples();
    }

    void push(const Error_Sample& sample)
    {
        if (mSamples.size() < max_samples()) {
            mSamples.push_back(sample);
        }
    }

    /**
     * @brief Grava todas as amostras acumuladas em um arquivo CSV.
     * @return true se o arquivo foi escrito com sucesso.
     */
    bool save_csv(const std::string& path,
                  const Error_Log_Metadata& meta) const
    {
        std::ofstream out(path.c_str());
        if (!out) {
            return false;
        }

        out << "# pan tilt pid - log de erro\n"
            << "# timestamp=" << meta.timestamp << '\n'
            << "# frame=" << meta.frameWidth << "x" << meta.frameHeight << '\n'
            << "# Ts_ms=" << std::setprecision(6) << meta.samplePeriodMs << '\n'
            << "# Kp_X=" << meta.kpX << " Ki_X=" << meta.kiX
            << " Kd_X=" << meta.kdX << '\n'
            << "# Kp_Y=" << meta.kpY << " Ki_Y=" << meta.kiY
            << " Kd_Y=" << meta.kdY << '\n'
            << "# aim_offset_x_px=" << meta.aimOffsetX
            << " aim_offset_y_px=" << meta.aimOffsetY << '\n'
            << "# invert_x=" << (meta.invertX ? 1 : 0)
            << " invert_y=" << (meta.invertY ? 1 : 0) << '\n'
            << "# samples=" << mSamples.size() << '\n'
            << "sample,time_s,valid,error_x_px,error_y_px,target_x_px,"
               "target_y_px,ref_x_px,ref_y_px,pan_ms,tilt_ms,pan_sat,tilt_sat\n";

        for (const Error_Sample& s : mSamples) {
            out << s.sample << ','
                << std::setprecision(6) << s.timeS << ','
                << (s.valid ? 1 : 0) << ',';

            if (s.valid) {
                out << std::setprecision(6) << s.errorX << ','
                    << std::setprecision(6) << s.errorY << ','
                    << s.targetX << ',' << s.targetY << ',';
            } else {
                // Quatro colunas vazias: sem erro e sem alvo neste ciclo.
                out << ",,,,";
            }

            out << std::setprecision(6) << s.refX << ','
                << std::setprecision(6) << s.refY << ','
                << std::setprecision(6) << s.panMs << ','
                << std::setprecision(6) << s.tiltMs << ','
                << (s.panSat ? 1 : 0) << ','
                << (s.tiltSat ? 1 : 0) << '\n';
        }

        return out.good();
    }

private:
    std::vector<Error_Sample> mSamples;
};

} // namespace Image_Processing
