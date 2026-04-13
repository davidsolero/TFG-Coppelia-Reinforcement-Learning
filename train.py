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
import os

# ── Configuración de experimento ───────────────────────────────────────────────
EXP_NAME = "exp_008_MediaHabSinCyConPenalFija"  # Cambiar por nombre único de experimento
BASE_DIR = f"./experiments/{EXP_NAME}"
os.makedirs(f"{BASE_DIR}/model", exist_ok=True)
os.makedirs(f"{BASE_DIR}/train_logs", exist_ok=True)

# Si existe un modelo previo, se reanuda el entrenamiento en lugar de empezar de cero.
RESUME_TRAINING = False  # Cambiar a True para continuar desde un modelo guardado (si existe)

# ── Parámetros ────────────────────────────────────────────────────────────────
TOTAL_TIMESTEPS  = 40_960  # Pasos de entrenamiento
MAX_STEPS_EP     = 50       # Pasos máximos por episodio (truncation)
LOG_DIR          = f"{BASE_DIR}/train_logs"  # Directorio para TensorBoard
MODEL_PATH       = f"{BASE_DIR}/model/ppo_robot"  # Dónde guardar el modelo final
TRACE            = False    # True para ver logs paso a paso

# ── Entorno ───────────────────────────────────────────────────────────────────
print("Creando entorno...")
env = RobotEnv(max_steps=MAX_STEPS_EP, trace=TRACE)

# Verificación automática de compatibilidad con Gymnasium/SB3
print("Verificando entorno...")
check_env(env, warn=True)

# ── Modelo PPO ────────────────────────────────────────────────────────────────
model_zip_path = f"{MODEL_PATH}.zip"

if RESUME_TRAINING and os.path.exists(model_zip_path):
    print(f"Cargando modelo existente: {model_zip_path}")
    model = PPO.load(MODEL_PATH, env=env)
    model.tensorboard_log = LOG_DIR
else:
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
    reset_num_timesteps = not RESUME_TRAINING,
)

# ── Guardar modelo ────────────────────────────────────────────────────────────
model.save(MODEL_PATH)
print(f"\nModelo guardado en: {MODEL_PATH}")

# ── Cierre limpio ─────────────────────────────────────────────────────────────
env.close()
print("Entrenamiento completado.")