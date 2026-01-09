# Makefile for Ball Collecting Robot Project
# 
# Common development tasks for Python analysis tools and Arduino firmware.

.PHONY: help install install-dev lint format typecheck test clean \
        sim identify arduino-p1 arduino-p2

# Default target
help:
	@echo "Ball Collecting Robot - Development Commands"
	@echo ""
	@echo "Setup:"
	@echo "  make install      Install Python dependencies"
	@echo "  make install-dev  Install with development tools"
	@echo ""
	@echo "Code Quality:"
	@echo "  make lint         Run linter (ruff)"
	@echo "  make format       Format code (ruff)"
	@echo "  make typecheck    Run type checker (mypy)"
	@echo "  make test         Run tests"
	@echo ""
	@echo "Simulation & Analysis:"
	@echo "  make sim          Run kinematic simulation"
	@echo "  make identify     Run motor identification"
	@echo ""
	@echo "Arduino (requires arduino-cli):"
	@echo "  make arduino-p1   Compile Prototype 1 firmware"
	@echo "  make arduino-p2   Compile Prototype 2 firmware"
	@echo ""
	@echo "Maintenance:"
	@echo "  make clean        Remove generated files"

#------------------------------------------------------------------------------
# Python Setup
#------------------------------------------------------------------------------

install:
	pip install -e ".[plot]"

install-dev:
	pip install -e ".[all]"

#------------------------------------------------------------------------------
# Code Quality
#------------------------------------------------------------------------------

lint:
	ruff check modeling/ tools/

format:
	ruff format modeling/ tools/

typecheck:
	mypy modeling/ tools/

test:
	pytest tests/ -v

#------------------------------------------------------------------------------
# Simulation & Analysis
#------------------------------------------------------------------------------

sim:
	python -m modeling.kinematics_sim --kp 1.2 --ki 0.5

identify:
	python -m modeling.motor_identification data/step_response.csv --plot

#------------------------------------------------------------------------------
# Arduino Firmware
#------------------------------------------------------------------------------

ARDUINO_CLI := arduino-cli
BOARD := arduino:avr:uno
PORT := /dev/tty.usbserial*

arduino-p1:
	$(ARDUINO_CLI) compile --fqbn $(BOARD) firmware/prototype1/servo_guidance_pi.ino

arduino-p2:
	$(ARDUINO_CLI) compile --fqbn $(BOARD) firmware/prototype2/motor_speed_pi.ino

upload-p1: arduino-p1
	$(ARDUINO_CLI) upload --fqbn $(BOARD) --port $(PORT) firmware/prototype1/servo_guidance_pi.ino

upload-p2: arduino-p2
	$(ARDUINO_CLI) upload --fqbn $(BOARD) --port $(PORT) firmware/prototype2/motor_speed_pi.ino

#------------------------------------------------------------------------------
# Maintenance
#------------------------------------------------------------------------------

clean:
	find . -type d -name "__pycache__" -exec rm -rf {} + 2>/dev/null || true
	find . -type d -name "*.egg-info" -exec rm -rf {} + 2>/dev/null || true
	find . -type f -name "*.pyc" -delete 2>/dev/null || true
	find . -type d -name ".mypy_cache" -exec rm -rf {} + 2>/dev/null || true
	find . -type d -name ".ruff_cache" -exec rm -rf {} + 2>/dev/null || true
	rm -rf build/ dist/ .pytest_cache/
