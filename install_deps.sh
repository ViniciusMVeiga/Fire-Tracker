#!/usr/bin/env bash
# =============================================================================
# install_deps.sh — Dependências de sistema do Fire-Tracker (Pan-Tilt PID)
#
# O projeto é 100% C++: as dependências NÃO são instaláveis via pip. Este
# script é o equivalente ao "requirements.txt" para a stack de compilação.
#
# Uso:
#   ./install_deps.sh            # instala tudo (Raspberry Pi: CSI + GPIO)
#   ./install_deps.sh --mock     # só o necessário para PC (OpenCV + toolchain)
#   ./install_deps.sh --check    # apenas verifica o que já está instalado
#
# Modos:
#   full  → build-essential, pkg-config, OpenCV, pigpio, GStreamer
#   mock  → build-essential, pkg-config, OpenCV  (make mock / make tests)
#
# Referências:
#   - Makefile (OPENCV_CFLAGS, PIGPIO_LIBS)
#   - Config/config.hpp (pipeline GStreamer com libcamerasrc)
# =============================================================================

set -euo pipefail

MODE="full"
CHECK_ONLY=0

usage() {
    sed -n '3,18p' "$0" | sed 's/^# \{0,1\}//'
}

for arg in "$@"; do
    case "$arg" in
        --mock|-m)  MODE="mock" ;;
        --full|-f)  MODE="full" ;;
        --check|-c) CHECK_ONLY=1 ;;
        --help|-h)  usage; exit 0 ;;
        *)
            echo "erro: opção desconhecida: $arg" >&2
            echo "use --help para ver as opções." >&2
            exit 2
            ;;
    esac
done

# ─── Detecção de ambiente ────────────────────────────────────────────────────

is_raspberry_pi() {
    [[ -r /proc/device-tree/model ]] &&
        tr -d '\0' < /proc/device-tree/model | grep -qi 'raspberry pi'
}

if [[ $EUID -eq 0 ]]; then
    SUDO=()
else
    SUDO=(sudo)
fi

# ─── Pacotes por modo ────────────────────────────────────────────────────────

BASE_PKGS=(build-essential pkg-config)
OPENCV_PKGS=(libopencv-dev)
PIGPIO_PKGS=(libpigpio-dev pigpio)
GST_PKGS=(libgstreamer1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-libav)

PKGS=("${BASE_PKGS[@]}" "${OPENCV_PKGS[@]}")

if [[ "$MODE" == "full" ]]; then
    PKGS+=("${PIGPIO_PKGS[@]}" "${GST_PKGS[@]}")

    # O plugin libcamerasrc (usado em Config::GSTREAMER_PIPELINE_TEMPLATE)
    # vem em pacote separado no Raspberry Pi OS Bookworm.
    if is_raspberry_pi && apt-cache show gstreamer1.0-libcamera >/dev/null 2>&1; then
        PKGS+=(gstreamer1.0-libcamera)
    fi
fi

# ─── Verificação ─────────────────────────────────────────────────────────────

FAILURES=0

check_item() {
    local label="$1"
    shift
    if "$@" >/dev/null 2>&1; then
        printf '  [OK]    %s\n' "$label"
    else
        printf '  [FALTA] %s\n' "$label"
        FAILURES=$((FAILURES + 1))
    fi
}

verify() {
    FAILURES=0
    echo "Verificando dependências (modo: $MODE)..."

    check_item "compilador g++"            g++ --version
    check_item "make"                      make --version
    check_item "pkg-config"                pkg-config --version
    check_item "OpenCV 4 (C++ headers)"    pkg-config --exists opencv4

    if [[ "$MODE" == "full" ]]; then
        check_item "pigpio (pigpio.h)"     test -f /usr/include/pigpio.h
        check_item "GStreamer 1.0 (dev)"   pkg-config --exists gstreamer-1.0

        if is_raspberry_pi; then
            check_item "plugin libcamerasrc" gst-inspect-1.0 libcamerasrc
        fi
    fi

    echo
    if [[ $FAILURES -eq 0 ]]; then
        echo "Tudo pronto."
        return 0
    fi
    echo "$FAILURES dependência(s) faltando."
    return 1
}

if [[ $CHECK_ONLY -eq 1 ]]; then
    verify
    exit $?
fi

# ─── Instalação ──────────────────────────────────────────────────────────────

if ! command -v apt-get >/dev/null 2>&1; then
    echo "erro: este script requer apt-get (Debian/Raspberry Pi OS/Ubuntu)." >&2
    exit 1
fi

if [[ "$MODE" == "full" ]] && ! is_raspberry_pi; then
    echo "aviso: este não parece ser um Raspberry Pi."
    echo "       pigpio e a câmera CSI podem não funcionar aqui."
    echo "       Para desenvolver no PC, use: ./install_deps.sh --mock"
    echo
fi

echo "Atualizando lista de pacotes..."
"${SUDO[@]}" apt-get update

# Instala apenas o que existe no repositório, avisando sobre o restante.
AVAILABLE=()
MISSING=()
for pkg in "${PKGS[@]}"; do
    if apt-cache show "$pkg" >/dev/null 2>&1; then
        AVAILABLE+=("$pkg")
    else
        MISSING+=("$pkg")
    fi
done

if [[ ${#AVAILABLE[@]} -gt 0 ]]; then
    echo
    echo "Instalando: ${AVAILABLE[*]}"
    "${SUDO[@]}" apt-get install -y "${AVAILABLE[@]}"
fi

if [[ ${#MISSING[@]} -gt 0 ]]; then
    echo
    echo "aviso: pacotes indisponíveis neste repositório: ${MISSING[*]}" >&2
    case "$MODE" in
        full)
            echo "       No PC, use --mock; no Raspberry Pi OS, verifique se os" >&2
            echo "       repositórios universe/raspi estão habilitados." >&2
            ;;
    esac
fi

echo
if ! verify; then
    echo
    echo "Algumas dependências continuam faltando. Veja as mensagens acima." >&2
    exit 1
fi

echo
echo "Próximos passos:"
case "$MODE" in
    full)
        echo "  make            # compila o programa principal (pantilt)"
        echo "  sudo ./pantilt  # executa com GPIO e câmera reais"
        ;;
    mock)
        echo "  make mock       # compila sem GPIO/câmera do RPi"
        echo "  ./pantilt_mock  # executa em modo simulado"
        ;;
esac
echo "  make tests      # testes automatizados (sem hardware)"
