# Makefile para o sistema Pan-Tilt PID
# Alvo: Raspberry Pi 4, Raspberry Pi OS Bookworm (64-bit)
#
# Dependências no RPi:
#   sudo apt update
#   sudo apt install -y \
#       libopencv-dev \
#       libpigpio-dev \
#       pigpio \
#       libgstreamer1.0-dev \
#       gstreamer1.0-plugins-bad \
#       gstreamer1.0-libav
#
# Alvos:
#   make          → compila o programa principal (pantilt)
#   make tests    → compila e executa os testes automatizados
#   make mock     → compila o programa principal em modo mock (sem GPIO/câmera real)
#   make clean    → remove binários

CXX      := g++
CXXFLAGS := -std=c++17 -Wall -Wextra -Wpedantic -O2

# ─── OpenCV ──────────────────────────────────────────────────────────────────
OPENCV_CFLAGS := $(shell pkg-config --cflags opencv4 2>/dev/null || echo "-I/usr/include/opencv4")
OPENCV_LIBS   := $(shell pkg-config --libs   opencv4 2>/dev/null || echo "-lopencv_core -lopencv_imgproc -lopencv_videoio -lopencv_highgui")

# ─── pigpio ──────────────────────────────────────────────────────────────────
PIGPIO_LIBS := -lpigpio -lrt -lpthread

# ─── Fontes do projeto principal ─────────────────────────────────────────────
SRCS := main.cpp \
        Image_Processing/Image_Processor.cpp \
        Control/Pid_Controller.cpp \
        Actuators/Servo_Driver.cpp

TARGET := pantilt

# ─────────────────────────────────────────────────────────────────────────────
# Alvo principal
# ─────────────────────────────────────────────────────────────────────────────
all: $(TARGET)

$(TARGET): $(SRCS)
	$(CXX) $(CXXFLAGS) $(OPENCV_CFLAGS) -I. $^ -o $@ \
		$(OPENCV_LIBS) $(PIGPIO_LIBS)
	@echo "Compilado: $(TARGET)"
	@echo "Execute com: sudo ./$(TARGET)"

# ─────────────────────────────────────────────────────────────────────────────
# Modo mock (sem hardware GPIO nem câmera RPi)
# Compila com -DHARDWARE_MOCK para ativar stubs pigpio e câmera USB.
# ─────────────────────────────────────────────────────────────────────────────
mock: $(SRCS)
	$(CXX) $(CXXFLAGS) $(OPENCV_CFLAGS) -DHARDWARE_MOCK -I. $^ -o $(TARGET)_mock \
		$(OPENCV_LIBS) -lpthread
	@echo "Compilado: $(TARGET)_mock  (modo mock — sem pigpio, sem câmera RPi)"
	@echo "Execute com: ./$(TARGET)_mock"

# ─────────────────────────────────────────────────────────────────────────────
# Testes automatizados (sem hardware)
# ─────────────────────────────────────────────────────────────────────────────
TESTS_DIR := tests

TEST_PID_SRCS   := $(TESTS_DIR)/test_pid.cpp Control/Pid_Controller.cpp
TEST_SERVO_SRCS := $(TESTS_DIR)/test_servo_mock.cpp Actuators/Servo_Driver.cpp

TEST_PID_BIN   := $(TESTS_DIR)/test_pid
TEST_SERVO_BIN := $(TESTS_DIR)/test_servo_mock

tests: $(TEST_PID_BIN) $(TEST_SERVO_BIN)
	@echo ""
	@echo "─── Executando: test_pid ───"
	./$(TEST_PID_BIN)
	@echo ""
	@echo "─── Executando: test_servo_mock ───"
	./$(TEST_SERVO_BIN)

$(TEST_PID_BIN): $(TEST_PID_SRCS)
	$(CXX) $(CXXFLAGS) -DHARDWARE_MOCK -I. $^ -o $@

$(TEST_SERVO_BIN): $(TEST_SERVO_SRCS)
	$(CXX) $(CXXFLAGS) -DHARDWARE_MOCK -I. $^ -o $@

# ─────────────────────────────────────────────────────────────────────────────
# Limpeza
# ─────────────────────────────────────────────────────────────────────────────
clean:
	rm -f $(TARGET) $(TARGET)_mock \
	      $(TEST_PID_BIN) $(TEST_SERVO_BIN)

.PHONY: all mock tests clean
