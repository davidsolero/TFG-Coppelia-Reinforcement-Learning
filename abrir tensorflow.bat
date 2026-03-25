@echo off
echo Abriendo TensorBoard...
cd C:\Users\usuario\Documents\GitHub\TFG-Coppelia-Reinforcement-Learning
start "" "http://localhost:6006"
python -m tensorboard.main --logdir ./logs/
pause