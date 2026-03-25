"""
Entrenamiento del agente RL con PPO (Stable-Baselines3).
Monitorización con TensorBoard.

Uso:
    python train.py

Ver curvas de entrenamiento:
    tensorboard --logdir ./logs/
"""

from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
from robot_env import RobotEnv


# ── Parámetros ────────────────────────────────────────────────────────────────

TOTAL_TIMESTEPS  = 20_000   # Pasos totales de entrenamiento
MAX_STEPS_EP     = 50       # Pasos máximos por episodio (truncation)
LOG_DIR          = "./logs/Cmedia" # Directorio para TensorBoard
MODEL_PATH       = "./models/ppo_robot_Cmedia"  # Dónde guardar el modelo final
TRACE            = False    # True para ver logs paso a paso

# ── Entorno ───────────────────────────────────────────────────────────────────

print("Creando entorno...")
env = RobotEnv(max_steps=MAX_STEPS_EP, trace=TRACE)

# Verificación automática de compatibilidad con Gymnasium/SB3
# Comenta esta línea si ralentiza el arranque
print("Verificando entorno...")
check_env(env, warn=True)

# ── Modelo PPO ────────────────────────────────────────────────────────────────

print("Creando modelo PPO...")
model = PPO(
    policy          = "MlpPolicy",  # Red neuronal densa (Multi-Layer Perceptron)
    env             = env,
    verbose         = 1,            # Imprime progreso cada pocos episodios
    tensorboard_log = LOG_DIR,
)

print(f"\nIniciando entrenamiento ({TOTAL_TIMESTEPS} pasos)...")
print(f"Para ver curvas: tensorboard --logdir {LOG_DIR}\n")

# ── Entrenamiento ─────────────────────────────────────────────────────────────

model.learn(
    total_timesteps    = TOTAL_TIMESTEPS,
    tb_log_name        = "PPO_robot",   # Nombre de la run en TensorBoard
    progress_bar       = True,
)

# ── Guardar modelo ────────────────────────────────────────────────────────────

import os
os.makedirs("./models", exist_ok=True)
model.save(MODEL_PATH)
print(f"\nModelo guardado en: {MODEL_PATH}")

# ── Cierre limpio ─────────────────────────────────────────────────────────────

env.close()
print("Entrenamiento completado.")