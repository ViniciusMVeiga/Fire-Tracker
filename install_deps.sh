#!/usr/bin/env bash
# =============================================================================
# install_deps.sh — Dependências de sistema do Fire-Tracker (Pan-Tilt PID)
#
# Compatível com Raspberry Pi OS (Bookworm / Trixie) e PC (Linux/WSL).
#
# O projeto é 100% C++: as dependências de sistema são instaladas via apt e,
# no caso do pigpio em distros mais novas (Debian 13 Trixie), compiladas
# automaticamente a partir do código-fonte oficial.
#
# Uso:
#   ./install_deps.sh            # instala tudo (Raspberry Pi: Câmera + Servos + Laser)
#   ./install_deps.sh --mock     # só o necessário para PC (OpenCV + toolchain de compilação)
#   ./install_deps.sh --check    # apenas verifica o que já está instalado
# =============================================================================

set -euo pipefail

MODE="full"
CHECK_ONLY=0

usage() {
    sed -n '3,15p' "$0" | sed 's/^# \{0,1\}//'
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

find_v4l2_compat() {
    find /usr/libexec /usr/lib -name "v4l2-compat.so" 2>/dev/null | head -n 1 || true
}

if [[ $EUID -eq 0 ]]; then
    SUDO=()
else
    SUDO=(sudo)
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
    echo "=================================================="
    echo "Verificando dependências (modo: $MODE)..."
    echo "=================================================="

    check_item "compilador g++"            g++ --version
    check_item "make"                      make --version
    check_item "pkg-config"                pkg-config --version
    check_item "git"                       git --version
    check_item "OpenCV 4 (C++ headers)"    pkg-config --exists opencv4

    if [[ "$MODE" == "full" ]]; then
        # pigpio.h pode estar em /usr/include ou /usr/local/include
        check_item "pigpio (pigpio.h)"     bash -c "test -f /usr/include/pigpio.h || test -f /usr/local/include/pigpio.h"
        check_item "biblioteca libpigpio"  bash -c "ldconfig -p | grep -q libpigpio || test -f /usr/local/lib/libpigpio.so"

        if is_raspberry_pi; then
            check_item "v4l2-compat.so (libcamera-v4l2)" bash -c "test -n \"\$(find_v4l2_compat)\""
        fi
    fi

    echo
    if [[ $FAILURES -eq 0 ]]; then
        echo "Todas as dependências estão presentes e prontas!"
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
    echo "aviso: este computador não parece ser um Raspberry Pi."
    echo "       GPIOs reais e a câmera CSI requerem hardware da Raspberry Pi."
    echo "       Para desenvolver e testar no PC, use: ./install_deps.sh --mock"
    echo
fi

echo "Atualizando listas do apt..."
"${SUDO[@]}" apt-get update

BASE_PKGS=(build-essential pkg-config git)
OPENCV_PKGS=(libopencv-dev)
HARDWARE_PKGS=()

if [[ "$MODE" == "full" ]]; then
    # Suporte a v4l2-compat para a Raspberry Pi Camera
    if apt-cache show libcamera-v4l2 >/dev/null 2>&1; then
        HARDWARE_PKGS+=(libcamera-v4l2)
    fi

    # Se o pacote pigpio estiver nos repositórios (Debian Bookworm/Bullseye), adiciona
    if apt-cache show libpigpio-dev >/dev/null 2>&1; then
        HARDWARE_PKGS+=(libpigpio-dev pigpio)
    fi
fi

PKGS=("${BASE_PKGS[@]}" "${OPENCV_PKGS[@]}" "${HARDWARE_PKGS[@]}")

echo
echo "Instalando pacotes via apt: ${PKGS[*]}"
"${SUDO[@]}" apt-get install -y "${PKGS[@]}"

# ─── Compilação do pigpio da fonte (se necessário) ───────────────────────────
if [[ "$MODE" == "full" ]]; then
    if ! test -f /usr/include/pigpio.h && ! test -f /usr/local/include/pigpio.h; then
        echo
        echo "libpigpio não encontrada nos repositórios apt (comum no Debian Trixie)."
        echo "Baixando e compilando pigpio a partir do código-fonte oficial..."

        BUILD_DIR=$(mktemp -d)
        git clone --depth 1 https://github.com/joan2937/pigpio.git "$BUILD_DIR/pigpio"
        pushd "$BUILD_DIR/pigpio" >/dev/null
        make -j"$(nproc)"
        "${SUDO[@]}" make install
        "${SUDO[@]}" ldconfig
        popd >/dev/null
        rm -rf "$BUILD_DIR"
        echo "pigpio instalado com sucesso a partir da fonte!"
    fi
fi

echo
if ! verify; then
    echo
    echo "Aviso: Algumas dependências ainda não foram identificadas. Verifique as mensagens acima." >&2
    exit 1
fi

V4L2_PATH=$(find_v4l2_compat)

echo
echo "=================================================="
echo "Instalação concluída com sucesso!"
echo "=================================================="
echo "Próximos passos para compilar e executar:"
case "$MODE" in
    full)
        echo "  1. Compilar o projeto:"
        echo "       make"
        echo
        echo "  2. Compilar ferramentas auxiliares (opcional):"
        echo "       make calibrate_servo"
        echo "       make view_camera"
        echo
        echo "  3. Executar o rastreador de fogo:"
        if [[ -n "$V4L2_PATH" ]]; then
            echo "       sudo env LD_PRELOAD=$V4L2_PATH ./pantilt"
        else
            echo "       sudo ./pantilt"
        fi
        ;;
    mock)
        echo "  make mock       # compila sem GPIO/câmera real"
        echo "  ./pantilt_mock  # executa em modo simulado no PC"
        echo "  make tests      # executa os testes unitários automatizados"
        ;;
esac
echo "=================================================="
