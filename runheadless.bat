@echo off
echo ============================================
echo EJECUTANDO TEST EN MODO HEADLESS
echo ============================================

echo.
echo Iniciando CoppeliaSim headless...
start "" "C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu\coppeliaSim.exe" -h "C:\Users\usuario\Documents\GitHub\TFG-Coppelia-Reinforcement-Learning\entornoRobotVigilancia.ttt"

echo Esperando 10 segundos a que CoppeliaSim cargue...
timeout /t 10 /nobreak

echo.
echo Ejecutando test de episodios...
cd C:\Users\usuario\Documents\GitHub\TFG-Coppelia-Reinforcement-Learning
python test_episodes.py

echo.
echo Cerrando CoppeliaSim...
taskkill /IM coppeliaSim.exe /F >nul 2>&1

echo.
echo ============================================
echo TERMINADO
echo ============================================
pause