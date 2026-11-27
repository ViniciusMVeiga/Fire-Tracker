/**
 * @file Pid_Params_Editor.hpp
 * @brief Janela de configurações com campos de texto desenhados em HighGUI:
 *        ganhos PID, período de amostragem, posição da mira e salvamento de CSV.
 *
 * O OpenCV HighGUI não oferece um widget nativo de entrada de texto (apenas
 * trackbars/botões). Por isso esta classe desenha os campos em um cv::Mat e
 * recebe as teclas através de cv::waitKey(), que é chamado pelo loop principal.
 *
 * Interação:
 *   - Clique em um campo para selecioná-lo;
 *   - Digite o valor diretamente (ex.: 0.001 para um ganho, 200.0 para Ts em ms);
 *   - Enter confirma, Esc cancela e restaura o valor anterior;
 *   - Backspace apaga o último caractere;
 *   - Clique em "Salvar CSV" para pedir o arquivo à aplicação principal.
 *
 * Os ganhos são exibidos/inseridos em valor real, sem qualquer fator de
 * escala (antes era necessário digitar 100 para representar 0.001 = 100e-5).
 * A mira é um deslocamento em pixels em relação ao centro da imagem:
 * X positivo à direita, Y positivo para baixo; (0,0) = centro.
 */

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <optional>
#include <sstream>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>  // EVENT_LBUTTONDOWN, waitKey
#include <opencv2/imgproc.hpp>

namespace Image_Processing {

// ─────────────────────────────────────────────────────────────────────────────
// Valores editados
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Conjunto de parâmetros ajustáveis pela janela de configurações.
 */
struct Pid_Editor_Values {
    double kpX = 0.0;          ///< Ganho proporcional do pan  [ms/px]
    double kiX = 0.0;          ///< Ganho integral do pan      [ms/(px·s)]
    double kdX = 0.0;          ///< Ganho derivativo do pan    [ms·s/px]
    double kpY = 0.0;          ///< Ganho proporcional do tilt [ms/px]
    double kiY = 0.0;          ///< Ganho integral do tilt     [ms/(px·s)]
    double kdY = 0.0;          ///< Ganho derivativo do tilt   [ms·s/px]
    double samplePeriodMs = 200.0; ///< Período de amostragem [ms]
    double aimOffsetX = 0.0;   ///< Deslocamento da mira em X [px], +direita
    double aimOffsetY = 0.0;   ///< Deslocamento da mira em Y [px], +baixo
};

// ─────────────────────────────────────────────────────────────────────────────
// Editor
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Janela de configurações: 9 campos de texto e um botão de salvamento.
 */
class Pid_Params_Editor {
public:
    static constexpr int kWindowWidth  = 440;
    static constexpr int kWindowHeight = 430;

    static const char* window_name()
    {
        return "Configuracoes (editar quando parado)";
    }

    Pid_Params_Editor()
    {
        // Layout: 2 colunas de ganhos, Ts, mira (X/Y) e botão Salvar CSV.
        const int colX[2] = {14, 226};
        const int rowY[3] = {70, 118, 166};
        for (int i = 0; i < 3; ++i) {
            mRects[i]     = cv::Rect(colX[0], rowY[i], 200, 38);
            mRects[i + 3] = cv::Rect(colX[1], rowY[i], 200, 38);
        }
        mRects[TsMs]   = cv::Rect(14, 214, 412, 38);
        mRects[MiraX]  = cv::Rect(colX[0], 262, 200, 38);
        mRects[MiraY]  = cv::Rect(colX[1], 262, 200, 38);
        mSaveButton    = cv::Rect(14, 312, 412, 38);

        set_values(Pid_Editor_Values{});
    }

    /// Informa ao editor o tamanho efetivo do frame para validar a mira.
    void set_frame_size(int width, int height)
    {
        mFrameWidth  = width;
        mFrameHeight = height;
    }

    /// Carrega os valores iniciais (normalmente vindos de Config).
    void set_values(const Pid_Editor_Values& values)
    {
        mValues[0] = values.kpX;
        mValues[1] = values.kiX;
        mValues[2] = values.kdX;
        mValues[3] = values.kpY;
        mValues[4] = values.kiY;
        mValues[5] = values.kdY;
        mValues[6] = values.samplePeriodMs;
        mValues[7] = values.aimOffsetX;
        mValues[8] = values.aimOffsetY;

        for (int i = 0; i < FieldCount; ++i) {
            mText[i] = format_value(i, mValues[i]);
        }
        mActive.reset();
        set_status("Clique em um campo para editar", Status::Neutral);
    }

    /// Valores atualmente confirmados no editor.
    Pid_Editor_Values values() const
    {
        Pid_Editor_Values v;
        v.kpX            = mValues[0];
        v.kiX            = mValues[1];
        v.kdX            = mValues[2];
        v.kpY            = mValues[3];
        v.kiY            = mValues[4];
        v.kdY            = mValues[5];
        v.samplePeriodMs = mValues[6];
        v.aimOffsetX     = mValues[7];
        v.aimOffsetY     = mValues[8];
        return v;
    }

    /// Habilita/desabilita a edição (desabilita = sistema rodando).
    void set_enabled(bool enabled)
    {
        if (!enabled && mActive.has_value()) {
            // Cancela edição em andamento ao iniciar o controle.
            mText[*mActive] = format_value(*mActive, mValues[*mActive]);
            mActive.reset();
        }
        mEnabled = enabled;
    }

    /// Retorna true uma vez quando o usuário clicou em "Salvar CSV".
    bool consume_save_request()
    {
        if (!mSaveRequested) {
            return false;
        }
        mSaveRequested = false;
        return true;
    }

    /// Mostra na janela o resultado de uma operação (ex.: gravação do CSV).
    void show_status(const std::string& text, bool ok)
    {
        set_status(text, ok ? Status::Ok : Status::Error);
    }

    /**
     * @brief Processa uma tecla recebida do cv::waitKey().
     * @return true se a tecla foi consumida pelo campo ativo.
     */
    bool handle_key(int key)
    {
        if (!mActive.has_value()) {
            return false;
        }

        const int field = *mActive;

        if (key == 27) {  // ESC
            mText[field] = format_value(field, mValues[field]);
            mActive.reset();
            set_status("Edicao cancelada", Status::Neutral);
            return true;
        }

        if (key == 13 || key == 10) {  // Enter
            double parsed = 0.0;
            if (!parse_value(mText[field], parsed) || !is_valid(field, parsed)) {
                set_status("Valor invalido - use Enter apos corrigir", Status::Error);
                return true;
            }
            mValues[field] = parsed;
            mText[field] = format_value(field, parsed);
            mActive.reset();
            set_status("Confirmado: " + std::string(field_defs()[field].label) +
                       " = " + mText[field], Status::Ok);
            return true;
        }

        if (key == 8 || key == 127) {  // Backspace / Delete
            if (!mText[field].empty()) {
                mText[field].pop_back();
            }
            return true;
        }

        const int ch = key & 0xFF;
        const bool isDigit = (ch >= static_cast<int>('0') && ch <= static_cast<int>('9'));
        const bool isNumericChar = isDigit || ch == static_cast<int>('.') ||
                                   ch == static_cast<int>(',') ||
                                   ch == static_cast<int>('-') ||
                                   ch == static_cast<int>('+');

        if (isNumericChar && mText[field].size() < 24) {
            // Vírgula é aceita como separador decimal para conveniência.
            mText[field].push_back(ch == static_cast<int>(',') ? '.' :
                                   static_cast<char>(ch));
            set_status("Digite o valor e pressione Enter", Status::Neutral);
        }
        // Qualquer tecla enquanto um campo está ativo é consumida, para que
        // 'S'/'Q' não acionem o controle principal durante a edição.
        return true;
    }

    /// Callback de mouse: seleciona o campo clicado.
    void handle_mouse(int event, int x, int y)
    {
        if (event != cv::EVENT_LBUTTONDOWN) {
            return;
        }

        const cv::Point pt(x, y);

        // O botão de CSV funciona com o sistema parado ou rodando.
        if (mSaveButton.contains(pt)) {
            mSaveRequested = true;
            set_status("Gerando CSV da sessao...", Status::Neutral);
            return;
        }

        if (!mEnabled) {
            set_status("Pare o sistema para editar", Status::Error);
            return;
        }

        for (int i = 0; i < FieldCount; ++i) {
            if (mRects[i].contains(pt)) {
                mActive = i;
                // Campo selecionado: a próxima digitação substitui o valor atual.
                mText[i].clear();
                set_status("Editando " + std::string(field_defs()[i].label) +
                           " - Enter confirma, Esc cancela", Status::Neutral);
                return;
            }
        }
        mActive.reset();
    }

    /// Desenha e devolve a imagem da janela.
    const cv::Mat& render()
    {
        if (mCanvas.rows != kWindowHeight || mCanvas.cols != kWindowWidth ||
            mCanvas.type() != CV_8UC3) {
            mCanvas.create(kWindowHeight, kWindowWidth, CV_8UC3);
        }
        mCanvas.setTo(cv::Scalar(28, 30, 38));

        const cv::Scalar titleColor(225, 230, 245);
        const cv::Scalar hintColor(150, 158, 178);

        cv::putText(mCanvas, "Configuracoes", cv::Point(14, 28),
                    cv::FONT_HERSHEY_SIMPLEX, 0.72, titleColor, 2, cv::LINE_8);
        cv::putText(mCanvas,
                    mEnabled ? "Clique no campo, digite o valor e pressione Enter"
                             : "Sistema rodando - pare para editar",
                    cv::Point(14, 50), cv::FONT_HERSHEY_SIMPLEX, 0.42,
                    mEnabled ? hintColor : cv::Scalar(120, 180, 255), 1, cv::LINE_8);

        for (int i = 0; i < FieldCount; ++i) {
            draw_field(i);
        }

        // Botão de salvamento do CSV.
        cv::rectangle(mCanvas, mSaveButton, cv::Scalar(38, 96, 150), cv::FILLED);
        cv::rectangle(mCanvas, mSaveButton, cv::Scalar(120, 190, 245), 1);
        cv::putText(mCanvas, "Salvar CSV (logs/)",
                    cv::Point(mSaveButton.x + 96, mSaveButton.y + 25),
                    cv::FONT_HERSHEY_SIMPLEX, 0.55,
                    cv::Scalar(240, 246, 255), 1, cv::LINE_8);

        const cv::Scalar statusColor =
            mStatus == Status::Ok    ? cv::Scalar(90, 220, 110) :
            mStatus == Status::Error ? cv::Scalar(90, 120, 255) :
                                       cv::Scalar(170, 178, 195);
        cv::putText(mCanvas, mStatusText, cv::Point(14, 374),
                    cv::FONT_HERSHEY_SIMPLEX, 0.40, statusColor, 1, cv::LINE_8);

        cv::putText(mCanvas,
                    "Ganhos em valor real | Mira: 0,0 = centro (X+ dir., Y+ baixo)",
                    cv::Point(14, 402), cv::FONT_HERSHEY_SIMPLEX, 0.34,
                    hintColor, 1, cv::LINE_8);

        return mCanvas;
    }

private:
    enum FieldId {
        KpX = 0, KiX, KdX, KpY, KiY, KdY, TsMs, MiraX, MiraY,
        FieldCount
    };

    enum class Status { Neutral, Ok, Error };

    struct FieldDef {
        const char* label;
        const char* unit;
    };

    /// Definições dos campos (estático local evita dependência de inline variables).
    static const std::array<FieldDef, FieldCount>& field_defs()
    {
        static const std::array<FieldDef, FieldCount> defs{{
            {"Kp_X", "ms/px"},
            {"Ki_X", "ms/(px.s)"},
            {"Kd_X", "ms.s/px"},
            {"Kp_Y", "ms/px"},
            {"Ki_Y", "ms/(px.s)"},
            {"Kd_Y", "ms.s/px"},
            {"Ts",   "milissegundos (período do loop)"},
            {"Mira_X", "offset px (+direita)"},
            {"Mira_Y", "offset px (+baixo)"}
        }};
        return defs;
    }

    double mValues[FieldCount] = {};
    std::string mText[FieldCount];
    cv::Rect mRects[FieldCount];
    cv::Rect mSaveButton;
    std::optional<int> mActive;
    bool mEnabled = true;
    bool mSaveRequested = false;
    int mFrameWidth = 0;
    int mFrameHeight = 0;

    cv::Mat mCanvas;
    Status mStatus = Status::Neutral;
    std::string mStatusText;

    void set_status(const std::string& text, Status status)
    {
        mStatusText = text;
        mStatus = status;
    }

    static std::string format_value(int field, double value)
    {
        std::ostringstream oss;
        if (field == TsMs || field == MiraX || field == MiraY) {
            oss << std::fixed << std::setprecision(1) << value;
        } else {
            // Valor real, sem fator de escala: 0.001 continua sendo 0.001.
            oss << std::setprecision(6) << value;
        }
        return oss.str();
    }

    static bool parse_value(const std::string& text, double& out)
    {
        if (text.empty()) {
            return false;
        }
        try {
            std::size_t consumed = 0;
            const double value = std::stod(text, &consumed);
            if (consumed != text.size() || !std::isfinite(value)) {
                return false;
            }
            out = value;
            return true;
        } catch (...) {
            return false;
        }
    }

    bool is_valid(int field, double value) const
    {
        if (field == TsMs) {
            // Faixa segura: 20 ms (50 Hz) a 2000 ms (0,5 Hz).
            return value >= 20.0 && value <= 2000.0;
        }
        if (field == MiraX || field == MiraY) {
            if (!std::isfinite(value)) {
                return false;
            }
            // A mira deve permanecer dentro da imagem (0,0 = centro).
            if (mFrameWidth > 0 && mFrameHeight > 0) {
                const double limitX = static_cast<double>(mFrameWidth) / 2.0;
                const double limitY = static_cast<double>(mFrameHeight) / 2.0;
                return std::fabs(value) <= (field == MiraX ? limitX : limitY);
            }
            return true;
        }
        return std::isfinite(value);
    }

    void draw_field(int field)
    {
        const cv::Rect& rect = mRects[field];
        const bool active = mActive.has_value() && *mActive == field;
        const bool editable = mEnabled;

        const cv::Scalar bgColor = active
            ? cv::Scalar(40, 54, 68)
            : cv::Scalar(36, 38, 48);
        const cv::Scalar borderColor = active
            ? cv::Scalar(255, 190, 70)
            : cv::Scalar(84, 90, 108);
        const cv::Scalar labelColor = !editable
            ? cv::Scalar(120, 125, 140)
            : cv::Scalar(205, 212, 230);
        const cv::Scalar valueColor = !editable
            ? cv::Scalar(130, 136, 150)
            : (active ? cv::Scalar(120, 220, 255) : cv::Scalar(235, 240, 250));

        cv::rectangle(mCanvas, rect, bgColor, cv::FILLED);
        cv::rectangle(mCanvas, rect, borderColor, active ? 2 : 1);

        const FieldDef& def = field_defs()[field];
        cv::putText(mCanvas, def.label,
                    cv::Point(rect.x + 8, rect.y + 16),
                    cv::FONT_HERSHEY_SIMPLEX, 0.44, labelColor, 1, cv::LINE_8);

        cv::putText(mCanvas, def.unit,
                    cv::Point(rect.x + 8, rect.y + 33),
                    cv::FONT_HERSHEY_SIMPLEX, 0.32,
                    cv::Scalar(130, 138, 158), 1, cv::LINE_8);

        // Texto do valor alinhado à direita; quando ativo, mostra um cursor.
        std::string shown = mText[field];
        if (active) {
            shown += "|";
        }
        const double scale = 0.55;
        int baseline = 0;
        const cv::Size textSize = cv::getTextSize(
            shown, cv::FONT_HERSHEY_SIMPLEX, scale, 1, &baseline);
        const int textX = std::max(rect.x + 4,
                                   rect.x + rect.width - 8 - textSize.width);
        cv::putText(mCanvas, shown,
                    cv::Point(textX, rect.y + 27),
                    cv::FONT_HERSHEY_SIMPLEX, scale, valueColor, 1, cv::LINE_8);
    }
};

} // namespace Image_Processing
