@echo off
title DEEP EVALUATION HEADLESS PPO

echo ============================================
echo   DEEP EVALUATION (HEADLESS)
echo ============================================

set COPPELIA_PATH="C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu\coppeliaSim.exe"
set SCENE_PATH="%~dp0..\entornoRobotVigilancia.ttt"
set PROJECT_PATH="%~dp0.."

echo.
echo Iniciando CoppeliaSim en modo headless...

start "" %COPPELIA_PATH% -h %SCENE_PATH%

echo Esperando inicializacion (15s)...
timeout /t 15 /nobreak >nul

echo.
echo ============================================
echo Ejecutando evaluate_deep.py...
echo ============================================

cd /d %PROJECT_PATH%

python evaluate_deep.py > deepevaluation_logsinc.txt

echo.
echo ============================================
echo Evaluacion finalizada
echo ============================================

echo.
echo Cerrando CoppeliaSim...
taskkill /IM coppeliaSim.exe /F >nul 2>&1

echo.
echo Archivos generados:
echo - deepevaluation_results.csv
echo - deepevaluation_log.txt

echo.
pause