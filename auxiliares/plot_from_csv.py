"""
Script auxiliar para reconstruir gráficas desde CSV de evaluaciones de RobotEnv.
No requiere conexión a CoppeliaSim, solo pandas y matplotlib.

Uso:
    python plot_from_csv.py
"""

import pandas as pd
import matplotlib.pyplot as plt
import os

# ── Configuración ─────────────────────────────────────────────────────────────
EXPERIMENT_NAME = "exp_004_MediaHabSinCyConPenal"
CSV_PATH = f"./experiments/{EXPERIMENT_NAME}/deep_evaluation/deepevaluation_results.csv"
PLOTS_DIR = f"./experiments/{EXPERIMENT_NAME}/deep_evaluation/plots"

if not os.path.exists(CSV_PATH):
    raise FileNotFoundError(f"No se encontró el CSV en {CSV_PATH}")

os.makedirs(PLOTS_DIR, exist_ok=True)

# ── Carga de datos ────────────────────────────────────────────────────────────
df = pd.read_csv(CSV_PATH)

# ── Gráficas (idénticas a evaluate_deep.py) ─────────────────────────────────
plt.figure()
plt.hist(df['rooms'], bins=4)
plt.title("Habitaciones visitadas")
plt.xlabel("Número")
plt.ylabel("Frecuencia")
plt.savefig(os.path.join(PLOTS_DIR, "hist_rooms.png"), dpi=150, bbox_inches="tight")

plt.figure()
plt.hist(df['time_in_C'], bins=20)
plt.title("Tiempo en C (%)")
plt.savefig(os.path.join(PLOTS_DIR, "hist_time_in_c.png"), dpi=150, bbox_inches="tight")

plt.figure()
plt.boxplot([df['reward'], df['rooms'], df['time_in_C'], df['min_battery']])
plt.xticks([1, 2, 3, 4], ["Reward", "Rooms", "Time_C", "Battery"])
plt.title("Distribución de métricas")
plt.savefig(os.path.join(PLOTS_DIR, "boxplot_metrics.png"), dpi=150, bbox_inches="tight")

plt.show()

# ── Métricas numéricas rápidas ───────────────────────────────────────────────
print("\nResumen rápido:")
print(f"Reward medio:          {df['reward'].mean():.2f} ± {df['reward'].std():.2f}")
print(f"Habitaciones medias:   {df['rooms'].mean():.2f} / 3")
print(f"Tiempo medio en C:     {df['time_in_C'].mean()*100:.1f}%")
print(f"Batería mínima media:  {df['min_battery'].mean():.1f}%")
print(f"Gráficas guardadas en: {PLOTS_DIR}")