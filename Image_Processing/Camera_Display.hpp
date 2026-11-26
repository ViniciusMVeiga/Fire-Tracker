/**
 * @file Camera_Display.hpp
 * @brief Utilitários visuais compartilhados para visualização da câmera e gráfico de erro PID.
 */

#pragma once

#include "Image_Processing/Image_Processor.hpp"

#include <algorithm>
#include <cstddef>
#include <deque>
#include <iomanip>
#include <optional>
#include <sstream>
#include <string>
#include <functional>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace Image_Processing {

// ─────────────────────────────────────────────────────────────────────────────
// Painel de controle: estado Start/Stop e ganhos PID editáveis
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Estado do painel de controle exibido no dashboard.
 *
 * Contém flag de execução e os seis ganhos dos dois PIDs.
 * Quando running == false, os ganhos podem ser editados via trackbars.
 */
struct Control_Panel_State {
    bool   running = false;  ///< true → sistema em execução; false → parado/editável

    // Ganhos PID eixo X (Pan)
    double kpX = 0.001;
    double kiX = 0.0001;
    double kdX = 0.0005;

    // Ganhos PID eixo Y (Tilt)
    double kpY = 0.001;
    double kiY = 0.0001;
    double kdY = 0.0005;

    /// Região (em coordenadas do canvas do dashboard) do botão Start/Stop.
    /// Preenchida pelo Dashboard_Renderer durante render() — usada pelo mouse callback.
    cv::Rect buttonRect;
};


inline std::string format_decimal(double value, int precision = 3)
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision) << value;
    return oss.str();
}

inline void draw_hud_text(cv::Mat& image,
                          const std::string& text,
                          const cv::Point& origin,
                          double scale,
                          const cv::Scalar& color)
{
    cv::putText(image, text, origin, cv::FONT_HERSHEY_SIMPLEX, scale,
                cv::Scalar(0, 0, 0), 3, cv::LINE_8);
    cv::putText(image, text, origin, cv::FONT_HERSHEY_SIMPLEX, scale,
                color, 1, cv::LINE_8);
}

inline void draw_tracking_overlay(cv::Mat& display,
                                  const Target_Measurement& meas,
                                  const cv::Point& centerRef,
                                  const std::string& statusLine,
                                  const std::string& footerLine = {},
                                  const std::string& warningLine = {})
{
    cv::drawMarker(display, centerRef, cv::Scalar(0, 255, 0),
                   cv::MARKER_CROSS, 30, 2);

    if (meas.valid) {
        const cv::Point targetPt(meas.x, meas.y);
        cv::circle(display, targetPt, 12, cv::Scalar(0, 0, 255), 2);
        cv::circle(display, targetPt, 3, cv::Scalar(0, 0, 255), -1);

        const std::string targetText =
            "Alvo: (" + std::to_string(meas.x) + ", " + std::to_string(meas.y) + ")";
        draw_hud_text(display, targetText,
                      cv::Point(meas.x + 15, meas.y - 10),
                      0.5, cv::Scalar(0, 0, 255));
    } else {
        draw_hud_text(display, "Alvo nao detectado",
                      cv::Point(15, 28), 0.6, cv::Scalar(0, 180, 255));
    }

    if (!warningLine.empty()) {
        draw_hud_text(display, warningLine,
                      cv::Point(15, 54), 0.55, cv::Scalar(0, 0, 255));
    }

    const int statusY = footerLine.empty() ? (display.rows - 14) : (display.rows - 34);
    draw_hud_text(display, statusLine,
                  cv::Point(15, statusY), 0.52, cv::Scalar(255, 255, 255));

    if (!footerLine.empty()) {
        draw_hud_text(display, footerLine,
                      cv::Point(15, display.rows - 12),
                      0.45, cv::Scalar(200, 255, 200));
    }
}

/**
 * @brief Renderizador em tempo real do histórico de erro dos dois eixos PID (X e Y).
 */
class Pid_Error_Plot {
public:
    Pid_Error_Plot(double maxAbsErrorX,
                   double maxAbsErrorY,
                   std::size_t maxSamples = 160)
        : mMaxAbsErrorX(std::max(1.0, maxAbsErrorX))
        , mMaxAbsErrorY(std::max(1.0, maxAbsErrorY))
        , mMaxSamples(std::max<std::size_t>(10, maxSamples))
    {}

    void push_sample(std::optional<double> errorX, std::optional<double> errorY)
    {
        append_sample(mHistoryX, errorX);
        append_sample(mHistoryY, errorY);
    }

    void render_into(cv::Mat& target,
                     double panMs,
                     double tiltMs,
                     bool panSat,
                     bool tiltSat,
                     bool validMeas) const
    {
        target.setTo(cv::Scalar(24, 26, 32));

        const std::string infoX = build_info_text(
            mHistoryX.empty() ? std::nullopt : mHistoryX.back(),
            panMs, panSat, validMeas);
        const std::string infoY = build_info_text(
            mHistoryY.empty() ? std::nullopt : mHistoryY.back(),
            tiltMs, tiltSat, validMeas);

        const int marginL = 52;
        const int marginR = 12;
        const int plotW   = std::max(40, target.cols - marginL - marginR);
        const int subH    = std::max(40, (target.rows - 68) / 2);

        const cv::Rect rectX(marginL, 26, plotW, subH);
        const cv::Rect rectY(marginL, 26 + subH + 30, plotW, subH);

        draw_subplot(target, rectX, mHistoryX, mMaxAbsErrorX,
                     "Erro X (Pan)", infoX, cv::Scalar(255, 200, 60));
        draw_subplot(target, rectY, mHistoryY, mMaxAbsErrorY,
                     "Erro Y (Tilt)", infoY, cv::Scalar(80, 180, 255));
    }

private:
    double mMaxAbsErrorX;
    double mMaxAbsErrorY;
    std::size_t mMaxSamples;
    std::deque<std::optional<double>> mHistoryX;
    std::deque<std::optional<double>> mHistoryY;

    void append_sample(std::deque<std::optional<double>>& history,
                       std::optional<double> value) const
    {
        history.push_back(value);
        if (history.size() > mMaxSamples) {
            history.pop_front();
        }
    }

    static std::string build_info_text(std::optional<double> lastError,
                                       double pulseMs,
                                       bool saturated,
                                       bool validMeas)
    {
        std::string errStr = "SEM ALVO";
        if (validMeas && lastError.has_value()) {
            const std::string sign = (*lastError >= 0.0) ? "+" : "";
            errStr = sign + format_decimal(*lastError, 1) + " px";
        }
        return errStr + " | " + format_decimal(pulseMs) +
               " ms" + (saturated ? " [SAT]" : "");
    }

    int error_to_y(double err, double maxAbsError, const cv::Rect& rect) const
    {
        const double clamped = std::clamp(err, -maxAbsError, maxAbsError);
        const double norm    = (maxAbsError - clamped) / (2.0 * maxAbsError);
        return rect.y + static_cast<int>(norm * static_cast<double>(rect.height));
    }

    void draw_subplot(cv::Mat& canvas,
                      const cv::Rect& rect,
                      const std::deque<std::optional<double>>& history,
                      double maxAbsError,
                      const std::string& title,
                      const std::string& infoLine,
                      const cv::Scalar& lineColor) const
    {
        cv::rectangle(canvas, rect, cv::Scalar(32, 35, 44), cv::FILLED);
        cv::rectangle(canvas, rect, cv::Scalar(80, 86, 102), 1);

        const int yMid = rect.y + rect.height / 2;
        const int yQ1  = rect.y + rect.height / 4;
        const int yQ3  = rect.y + (3 * rect.height) / 4;

        cv::line(canvas, cv::Point(rect.x, yQ1), cv::Point(rect.x + rect.width, yQ1),
                 cv::Scalar(48, 52, 64), 1);
        cv::line(canvas, cv::Point(rect.x, yMid), cv::Point(rect.x + rect.width, yMid),
                 cv::Scalar(115, 120, 135), 1);
        cv::line(canvas, cv::Point(rect.x, yQ3), cv::Point(rect.x + rect.width, yQ3),
                 cv::Scalar(48, 52, 64), 1);

        const int maxInt = static_cast<int>(maxAbsError);
        cv::putText(canvas, "+" + std::to_string(maxInt),
                    cv::Point(6, rect.y + 12),
                    cv::FONT_HERSHEY_SIMPLEX, 0.36, cv::Scalar(170, 175, 190), 1, cv::LINE_8);
        cv::putText(canvas, "0",
                    cv::Point(30, yMid + 4),
                    cv::FONT_HERSHEY_SIMPLEX, 0.36, cv::Scalar(200, 205, 220), 1, cv::LINE_8);
        cv::putText(canvas, "-" + std::to_string(maxInt),
                    cv::Point(6, rect.y + rect.height - 4),
                    cv::FONT_HERSHEY_SIMPLEX, 0.36, cv::Scalar(170, 175, 190), 1, cv::LINE_8);

        cv::putText(canvas, title, cv::Point(rect.x, rect.y - 8),
                    cv::FONT_HERSHEY_SIMPLEX, 0.46, lineColor, 1, cv::LINE_8);
        cv::putText(canvas, infoLine, cv::Point(rect.x + 122, rect.y - 8),
                    cv::FONT_HERSHEY_SIMPLEX, 0.42, cv::Scalar(225, 230, 240), 1, cv::LINE_8);

        if (history.empty()) {
            return;
        }

        const std::size_t startCol = mMaxSamples - history.size();
        const double denom = static_cast<double>(std::max<std::size_t>(1, mMaxSamples - 1));

        std::optional<cv::Point> prevPt;
        for (std::size_t i = 0; i < history.size(); ++i) {
            if (!history[i].has_value()) {
                prevPt.reset();
                continue;
            }

            const double frac = static_cast<double>(startCol + i) / denom;
            const int px = rect.x + static_cast<int>(frac * static_cast<double>(rect.width));
            const int py = error_to_y(*history[i], maxAbsError, rect);
            const cv::Point currPt(px, py);

            if (prevPt.has_value()) {
                cv::line(canvas, *prevPt, currPt, lineColor, 2, cv::LINE_8);
            } else {
                cv::circle(canvas, currPt, 2, lineColor, cv::FILLED);
            }
            prevPt = currPt;
        }
    }
};

/**
 * @brief Monta painel único combinando câmera, máscara HSV, gráfico de erro PID e painel de controle.
 *
 * Layout do canvas (largura = camW + kPanelWidth):
 *   ┌─────────────────────┬──────────────────────────┐
 *   │  Camera + HUD       │  Gráfico de erro PID      │ ← kPlotHeight px
 *   │  (camW x camH)      │  (kPanelWidth px)         │
 *   ├─────────────────────┤                           │
 *   │  (preenchimento)    ├──────────────────────────┤
 *   │                     │  Painel Start/Stop + Ganhos│ ← kControlH px
 *   └─────────────────────┴──────────────────────────┘
 */
class Dashboard_Renderer {
public:
    Dashboard_Renderer() = default;

    /**
     * @brief Renderiza o dashboard completo.
     *
     * @param ctrlState Estado do painel de controle (running flag + ganhos).
     *                  O campo buttonRect é atualizado com a posição do botão.
     */
    const cv::Mat& render(const cv::Mat& cameraFrame,
                          const cv::Mat& maskFrame,
                          const Target_Measurement& meas,
                          const cv::Point& centerRef,
                          const std::string& statusLine,
                          const Pid_Error_Plot& errorPlot,
                          double panMs,
                          double tiltMs,
                          bool panSat,
                          bool tiltSat,
                          bool validMeas,
                          Control_Panel_State& ctrlState)
    {
        constexpr int kPanelWidth  = 440;
        constexpr int kPlotHeight  = 300;
        constexpr int kControlH    = 180;  ///< altura do painel Start/Stop + ganhos

        const int camW  = cameraFrame.cols;
        const int camH  = cameraFrame.rows;
        const int dashW = camW + kPanelWidth;
        const int dashH = std::max(camH, kPlotHeight + kControlH);

        if (mCanvas.rows != dashH || mCanvas.cols != dashW || mCanvas.type() != CV_8UC3) {
            mCanvas.create(dashH, dashW, CV_8UC3);
        }
        mCanvas.setTo(cv::Scalar(24, 26, 32));

        // ── 1. Câmera + mira + alvo + HUD (esquerda) ─────────────────────────
        cv::Mat camRoi = mCanvas(cv::Rect(0, 0, camW, camH));
        cameraFrame.copyTo(camRoi);

        // Overlay extra: estado do controlador
        const std::string ctrlOverlay = ctrlState.running
            ? "CTRL: RODANDO — pressione S para parar"
            : "CTRL: PARADO — pressione S para iniciar";
        const cv::Scalar ctrlColor = ctrlState.running
            ? cv::Scalar(50, 220, 50)
            : cv::Scalar(50, 180, 255);
        draw_hud_text(camRoi, ctrlOverlay, cv::Point(15, 54), 0.50, ctrlColor);

        draw_tracking_overlay(camRoi, meas, centerRef, statusLine,
                              "Q/ESC=sair  S=iniciar/parar");

        // ── 2. Gráfico de erro PID X e Y (direita superior) ──────────────────
        cv::Mat plotRoi = mCanvas(cv::Rect(camW, 0, kPanelWidth, kPlotHeight));
        errorPlot.render_into(plotRoi, panMs, tiltMs, panSat, tiltSat, validMeas);

        // ── 3. Painel de controle (direita inferior) ──────────────────────────
        const cv::Rect ctrlRegion(camW, kPlotHeight, kPanelWidth, kControlH);
        draw_control_panel(mCanvas, ctrlRegion, ctrlState);

        return mCanvas;
    }

private:
    cv::Mat mCanvas;
    cv::Mat mMaskSmall;

    // ── Desenha o painel Start/Stop + valores dos 6 ganhos ──────────────────
    static void draw_control_panel(cv::Mat& canvas,
                                   const cv::Rect& region,
                                   Control_Panel_State& ctrlState)
    {
        // Fundo do painel
        cv::rectangle(canvas, region, cv::Scalar(30, 32, 40), cv::FILLED);
        cv::rectangle(canvas, region, cv::Scalar(80, 86, 102), 1);

        const int x0 = region.x + 12;
        int y = region.y + 20;

        // ── Título ──────────────────────────────────────────────────────────
        cv::putText(canvas, "Controle PID",
                    cv::Point(x0, y),
                    cv::FONT_HERSHEY_SIMPLEX, 0.52,
                    cv::Scalar(220, 225, 240), 1, cv::LINE_8);
        y += 6;

        // ── Botão Start/Stop ────────────────────────────────────────────────
        const int btnW = 160, btnH = 32;
        const int btnX = region.x + (region.width - btnW) / 2;
        const int btnY = region.y + 30;
        const cv::Rect btnRect(btnX, btnY, btnW, btnH);

        const cv::Scalar btnColor = ctrlState.running
            ? cv::Scalar(40, 40, 180)    // vermelho-azul → PARAR
            : cv::Scalar(30, 140, 30);   // verde → INICIAR
        cv::rectangle(canvas, btnRect, btnColor, cv::FILLED);
        cv::rectangle(canvas, btnRect, cv::Scalar(180, 185, 200), 1);

        const std::string btnLabel = ctrlState.running ? "  PARAR  [S]" : "  INICIAR [S]";
        cv::putText(canvas, btnLabel,
                    cv::Point(btnX + 10, btnY + 22),
                    cv::FONT_HERSHEY_SIMPLEX, 0.56,
                    cv::Scalar(240, 245, 255), 1, cv::LINE_8);

        // Atualiza buttonRect em coordenadas do canvas (para o mouse callback)
        ctrlState.buttonRect = btnRect;

        // ── Ganhos atuais (read-only quando rodando, lembrete quando parado) ─
        y = btnY + btnH + 18;
        const cv::Scalar labelColor(200, 205, 220);
        const cv::Scalar valueColor = ctrlState.running
            ? cv::Scalar(180, 185, 200)
            : cv::Scalar(80, 220, 255);  // destaca em azul quando editável

        const std::string editHint = ctrlState.running
            ? ""
            : "  (use trackbars para editar)";

        // Cabeçalho
        cv::putText(canvas,
                    "Ganhos PID" + editHint,
                    cv::Point(x0, y),
                    cv::FONT_HERSHEY_SIMPLEX, 0.40,
                    ctrlState.running ? labelColor : cv::Scalar(80, 220, 255),
                    1, cv::LINE_8);
        y += 16;

        // Helper lambda: formata um ganho com precisão adequada
        auto fmt = [](double v) -> std::string {
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(5) << v;
            return oss.str();
        };

        // Duas colunas: X (Pan) e Y (Tilt)
        const int col2 = region.x + region.width / 2 + 4;

        struct GainRow { const char* name; double val; };
        const GainRow rowsX[3] = {
            {"Kp_X", ctrlState.kpX},
            {"Ki_X", ctrlState.kiX},
            {"Kd_X", ctrlState.kdX}
        };
        const GainRow rowsY[3] = {
            {"Kp_Y", ctrlState.kpY},
            {"Ki_Y", ctrlState.kiY},
            {"Kd_Y", ctrlState.kdY}
        };

        // Cabeçalhos de coluna
        cv::putText(canvas, "--- Pan (X) ---", cv::Point(x0, y),
                    cv::FONT_HERSHEY_SIMPLEX, 0.38, labelColor, 1, cv::LINE_8);
        cv::putText(canvas, "--- Tilt (Y) ---", cv::Point(col2, y),
                    cv::FONT_HERSHEY_SIMPLEX, 0.38, labelColor, 1, cv::LINE_8);
        y += 14;

        for (int i = 0; i < 3; ++i) {
            const std::string lineX = std::string(rowsX[i].name) + ": " + fmt(rowsX[i].val);
            const std::string lineY = std::string(rowsY[i].name) + ": " + fmt(rowsY[i].val);
            cv::putText(canvas, lineX, cv::Point(x0, y),
                        cv::FONT_HERSHEY_SIMPLEX, 0.40, valueColor, 1, cv::LINE_8);
            cv::putText(canvas, lineY, cv::Point(col2, y),
                        cv::FONT_HERSHEY_SIMPLEX, 0.40, valueColor, 1, cv::LINE_8);
            y += 14;
        }
    }
};

} // namespace Image_Processing
