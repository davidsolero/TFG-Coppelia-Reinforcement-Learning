@echo off
echo Abriendo TensorBoard...
cd /d "%~dp0.."
start "" "http://localhost:6006"
python -m tensorboard.main --logdir ./experiments/
pause