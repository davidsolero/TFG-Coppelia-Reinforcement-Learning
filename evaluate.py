"""
Evaluación del agente PPO entrenado.
Ejecutar con CoppeliaSim abierto CON interfaz gráfica (no headless).

Uso:
    1. Abre CoppeliaSim normalmente (con ventana)
    2. Carga la escena entornoRobotVigilancia.ttt
    3. Ejecuta: python evaluate.py

Métricas reportadas por episodio:
    - Recompensa total
    - Habitaciones visitadas (Hab1, Hab2, Hab3)
    - Veces que fue a C voluntariamente
    - Si la batería se agotó
    - Pasos realizados
"""

from stable_baselines3 import PPO
from robot_env import RobotEnv
import numpy as np

# ── Parámetros ────────────────────────────────────────────────────────────────

MODEL_PATH = "./models/ppo_robot_Cmedia"
NUM_EPISODES = 10
MAX_STEPS    = 50

# ── Cargar entorno y modelo ───────────────────────────────────────────────────

print("Conectando a CoppeliaSim...")
env = RobotEnv(max_steps=MAX_STEPS, trace=False)

print(f"Cargando modelo: {MODEL_PATH}.zip")
model = PPO.load(MODEL_PATH, env=env)

# ── Evaluación ────────────────────────────────────────────────────────────────

print(f"\n{'='*60}")
print(f"EVALUACIÓN: {NUM_EPISODES} episodios (deterministic=True)")
print(f"{'='*60}\n")

# Métricas globales
all_rewards      = []
all_steps        = []
all_rooms        = []
all_charges      = []
battery_depletions = 0

for ep in range(NUM_EPISODES):
    obs, info = env.reset()
    
    ep_reward    = 0.0
    ep_steps     = 0
    rooms_visited = set()
    charges      = 0
    depleted     = False
    prev_node    = None

    print(f"--- Episodio {ep + 1}/{NUM_EPISODES} ---")

    while True:
        # Acción determinista: el agente elige lo que aprendió, sin ruido
        action, _ = model.predict(obs, deterministic=True)
        obs, reward, terminated, truncated, info = env.step(int(action))

        ep_reward += reward
        ep_steps  += 1

        node = info['node']
        battery = info['battery']

        # Registrar visitas a habitaciones
        if node in ['Hab1', 'Hab2', 'Hab3']:
            rooms_visited.add(node)

        # Registrar visitas voluntarias a C
        if node == 'C' and prev_node != 'C':
            charges += 1

        prev_node = node

        print(f"  Paso {ep_steps:2d}: nodo={node:4s}  batería={battery:5.1f}%  "
              f"reward={reward:+.1f}  acum={ep_reward:.1f}")

        if terminated:
            depleted = True
            battery_depletions += 1
            print(f"  !! BATERÍA AGOTADA")
            break
        if truncated:
            print(f"  -- Episodio completado ({MAX_STEPS} pasos)")
            break

    # Resumen del episodio
    print(f"  Resumen: reward={ep_reward:.1f}  pasos={ep_steps}  "
          f"habitaciones={len(rooms_visited)}/3  "
          f"cargas={charges}  depleted={'SÍ' if depleted else 'NO'}\n")

    all_rewards.append(ep_reward)
    all_steps.append(ep_steps)
    all_rooms.append(len(rooms_visited))
    all_charges.append(charges)

# ── Resumen final ─────────────────────────────────────────────────────────────

print(f"{'='*60}")
print(f"RESUMEN FINAL ({NUM_EPISODES} episodios)")
print(f"{'='*60}")
print(f"Recompensa media:              {np.mean(all_rewards):.2f} ± {np.std(all_rewards):.2f}")
print(f"Recompensa máxima:             {np.max(all_rewards):.2f}")
print(f"Recompensa mínima:             {np.min(all_rewards):.2f}")
print(f"Pasos medios por episodio:     {np.mean(all_steps):.1f}")
print(f"Habitaciones medias visitadas: {np.mean(all_rooms):.1f} / 3")
print(f"Cargas voluntarias medias:     {np.mean(all_charges):.1f}")
print(f"Episodios con batería agotada: {battery_depletions} / {NUM_EPISODES}")
print(f"{'='*60}")

env.close()
print("Evaluación completada.")