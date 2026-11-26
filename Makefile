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
#   make                  → compila todos os programas (pantilt, calibrate_servo, view_camera)
#   make pantilt          → compila apenas o programa principal
#   make calibrate_servo  → compila apenas a ferramenta de calibração manual
#   make view_camera      → compila apenas o visualizador com calibração interativa
#   make tests            → compila e executa os testes automatizados
#   make mock             → compila o programa principal em modo mock (sem GPIO/câmera real)
#   make clean            → remove binários

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
        Actuators/Servo_Driver.cpp \
        Actuators/Laser_Driver.cpp

# Headers comuns: incluí-los como dependência garante rebuild quando
# configurações ou a GUI mudarem (evita executar um binário desatualizado).
HDRS := Config/config.hpp \
        Image_Processing/Image_Processor.hpp \
        Image_Processing/Camera_Display.hpp \
        Control/Pid_Controller.hpp \
        Actuators/Servo_Driver.hpp \
        Actuators/Laser_Driver.hpp

TARGET := pantilt

# ─────────────────────────────────────────────────────────────────────────────
# Alvo principal (compila o sistema e as duas ferramentas)
# ─────────────────────────────────────────────────────────────────────────────
all: $(TARGET) calibrate_servo view_camera
	@echo ""
	@echo "=================================================="
	@echo "Compilação concluída! Comandos para executar:"
	@echo "  sudo ./$(TARGET)                          # programa principal"
	@echo "  sudo ./calibrate_servo <gpio> <pulso_ms>  # calibração manual"
	@echo "  sudo ./view_camera                      # câmera + calibração interativa"
	@echo "=================================================="

$(TARGET): $(SRCS) $(HDRS)
	$(CXX) $(CXXFLAGS) $(OPENCV_CFLAGS) -I. $(SRCS) -o $@ \
		$(OPENCV_LIBS) $(PIGPIO_LIBS)
	@echo "Compilado: $(TARGET) (execute com: sudo ./$(TARGET))"

# ─────────────────────────────────────────────────────────────────────────────
# Modo mock (sem hardware GPIO nem câmera RPi)
# Compila com -DHARDWARE_MOCK para ativar stubs pigpio e câmera USB.
# ─────────────────────────────────────────────────────────────────────────────
mock: $(SRCS) $(HDRS)
	$(CXX) $(CXXFLAGS) $(OPENCV_CFLAGS) -DHARDWARE_MOCK -I. $(SRCS) -o $(TARGET)_mock \
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

$(TEST_PID_BIN): $(TEST_PID_SRCS) Control/Pid_Controller.hpp
	$(CXX) $(CXXFLAGS) -DHARDWARE_MOCK -I. $(TEST_PID_SRCS) -o $@

$(TEST_SERVO_BIN): $(TEST_SERVO_SRCS) Actuators/Servo_Driver.hpp
	$(CXX) $(CXXFLAGS) -DHARDWARE_MOCK -I. $(TEST_SERVO_SRCS) -o $@

# ─────────────────────────────────────────────────────────────────────────────
# Ferramentas auxiliares
# ─────────────────────────────────────────────────────────────────────────────
VIEW_CAMERA_SRCS := tools/view_camera.cpp \
                    Image_Processing/Image_Processor.cpp \
                    Actuators/Servo_Driver.cpp

calibrate_servo: tools/calibrate_servo.cpp
	$(CXX) $(CXXFLAGS) -I. $< -o $@ $(PIGPIO_LIBS)
	@echo "Compilado: calibrate_servo (execute com: sudo ./calibrate_servo <gpio> <pulso_ms>)"

view_camera: $(VIEW_CAMERA_SRCS) $(HDRS)
	$(CXX) $(CXXFLAGS) $(OPENCV_CFLAGS) -I. $(VIEW_CAMERA_SRCS) -o $@ \
		$(OPENCV_LIBS) $(PIGPIO_LIBS)
	@echo "Compilado: view_camera (execute com: sudo ./view_camera)"

# ─────────────────────────────────────────────────────────────────────────────
# Limpeza
# ─────────────────────────────────────────────────────────────────────────────
clean:
	rm -f $(TARGET) $(TARGET)_mock calibrate_servo view_camera \
	      $(TEST_PID_BIN) $(TEST_SERVO_BIN)

.PHONY: all mock tests clean calibrate_servo view_camera
