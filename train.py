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

TOTAL_TIMESTEPS  = 20_000
MAX_STEPS_EP     = 50
LOG_DIR          = "./logs/exp2_sin_C_en_media/"
MODEL_PATH       = "./models/exp2_sin_C_en_media/ppo_robot"
TRACE            = False

# ── Entorno ───────────────────────────────────────────────────────────────────

print("Creando entorno...")
env = RobotEnv(max_steps=MAX_STEPS_EP, trace=TRACE)

print("Verificando entorno...")
check_env(env, warn=True)

# ── Modelo PPO ────────────────────────────────────────────────────────────────

print("Creando modelo PPO...")
model = PPO(
    policy          = "MlpPolicy",
    env             = env,
    verbose         = 1,
    tensorboard_log = LOG_DIR,
)

print(f"\nIniciando entrenamiento ({TOTAL_TIMESTEPS} pasos)...")
print(f"Para ver curvas: tensorboard --logdir {LOG_DIR}\n")

# ── Entrenamiento ─────────────────────────────────────────────────────────────

model.learn(
    total_timesteps    = TOTAL_TIMESTEPS,
    tb_log_name        = "exp2_sin_C_en_media",
    progress_bar       = True,
)

# ── Guardar modelo ────────────────────────────────────────────────────────────

import os
os.makedirs(f"./models/exp2_sin_C_en_media", exist_ok=True)
model.save(MODEL_PATH)
print(f"\nModelo guardado en: {MODEL_PATH}")

# ── Cierre limpio ─────────────────────────────────────────────────────────────

env.close()
print("Entrenamiento completado.")