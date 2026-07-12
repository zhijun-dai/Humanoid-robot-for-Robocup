@echo off
cd /d "%~dp0.."
.venv\Scripts\python.exe jetson_vision\run_real_car.py
pause
