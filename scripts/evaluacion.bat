@echo off
echo ============================================
echo EVALUACION DEL MODELO PPO
echo ============================================

echo.
echo Iniciando CoppeliaSim (con interfaz grafica)...
start "" "C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu\coppeliaSim.exe" "%~dp0..\entornoRobotVigilancia.ttt"

echo Esperando 15 segundos a que CoppeliaSim cargue...
timeout /t 15 /nobreak

echo.
echo Ejecutando evaluacion...
cd /d "%~dp0.."
python evaluate.py

echo.
echo Cerrando CoppeliaSim...
taskkill /IM coppeliaSim.exe /F >nul 2>&1

echo.
echo ============================================
echo TERMINADO
echo ============================================
pause