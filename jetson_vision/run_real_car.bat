@echo off
cd /d "%~dp0.."
call .venv\Scripts\activate.bat
pip install pyserial -q
echo === Real Car Control ===
echo   Speed: %REAL_CAR_SPEED% (default 10 cm/s)
echo   Serial: %SERIAL_PORT% (default COM3, press 's' to toggle)
echo   Press 'q' to quit
echo.
python jetson_vision\run_real_car.py
pause
