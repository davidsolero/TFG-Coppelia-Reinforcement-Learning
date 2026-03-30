@echo off
echo ============================================
echo EJECUTANDO ENTRENAMIENTO EN MODO HEADLESS
echo ============================================

echo.
echo Iniciando CoppeliaSim headless...
start "" "C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu\coppeliaSim.exe" -h "%~dp0..\entornoRobotVigilancia.ttt"

echo Esperando 10 segundos a que CoppeliaSim cargue...
timeout /t 10 /nobreak

echo.
echo Ejecutando entrenamiento...
cd /d "%~dp0.."
python train.py

echo.
echo Cerrando CoppeliaSim...
taskkill /IM coppeliaSim.exe /F >nul 2>&1

echo.
echo ============================================
echo TERMINADO
echo ============================================
pause