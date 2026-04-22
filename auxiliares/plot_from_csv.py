"""
Script auxiliar para reconstruir graficas desde CSV de evaluaciones de RobotEnv.
No requiere conexion a CoppeliaSim, solo pandas y matplotlib.

Uso:
    python plot_from_csv.py
"""

import os
import re
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# -- Configuracion -------------------------------------------------------------
EXPERIMENT_NAME = "exp_001_MediaSoloHabsRemakeobs"
DEEP_EVAL_DIR = f"./experiments/{EXPERIMENT_NAME}/deep_evaluation"


def find_latest_csv_and_plots_dir(base_dir):
    versioned = []
    pattern = re.compile(r"deepevaluation_resultsv(\d+)\.csv$")
    for name in os.listdir(base_dir):
        match = pattern.match(name)
        if match:
            versioned.append((int(match.group(1)), name))

    if versioned:
        latest_version, latest_csv_name = max(versioned, key=lambda pair: pair[0])
        csv_path = os.path.join(base_dir, latest_csv_name)
        plots_dir = os.path.join(base_dir, f"plotsv{latest_version}")
        return csv_path, plots_dir

    fallback_csv = os.path.join(base_dir, "deepevaluation_results.csv")
    fallback_plots = os.path.join(base_dir, "plots")
    if os.path.exists(fallback_csv):
        return fallback_csv, fallback_plots

    raise FileNotFoundError(
        "No se encontro ningun CSV de deep evaluation. "
        "Esperado: deepevaluation_resultsvN.csv o deepevaluation_results.csv"
    )


if not os.path.isdir(DEEP_EVAL_DIR):
    raise FileNotFoundError(f"No existe el directorio: {DEEP_EVAL_DIR}")

CSV_PATH, PLOTS_DIR = find_latest_csv_and_plots_dir(DEEP_EVAL_DIR)
os.makedirs(PLOTS_DIR, exist_ok=True)

# -- Carga de datos ------------------------------------------------------------
df = pd.read_csv(CSV_PATH)


def col(name, fallback=None):
    if name in df.columns:
        return df[name]
    if fallback is not None and fallback in df.columns:
        return df[fallback]
    raise KeyError(f"No se encontro la columna '{name}' en el CSV")


def has_columns(*names):
    return all(name in df.columns for name in names)


def save_plot_if_missing(filename):
    output_path = os.path.join(PLOTS_DIR, filename)
    if os.path.exists(output_path):
        print(f"Saltando (ya existe): {output_path}")
        return
    plt.savefig(output_path, dpi=150, bbox_inches="tight")


print(f"CSV cargado: {CSV_PATH}")
print(f"Graficas en: {PLOTS_DIR}")

# -- Graficas (alineadas con evaluate_deep.py actual) --------------------------
rooms_series = col("rooms")
time_in_c_series = col("time_in_C")
time_in_c_pct = np.asarray(time_in_c_series, dtype=np.float32) * 100.0

plt.figure()
room_categories = np.array([0, 1, 2, 3], dtype=np.int32)
room_counts_plot = np.bincount(np.clip(np.asarray(rooms_series, dtype=np.int32), 0, 3), minlength=4)
room_perc_plot = room_counts_plot / max(1, len(rooms_series)) * 100.0
bars = plt.bar(room_categories, room_counts_plot, color="#4C78A8", width=0.75)
for idx, bar in enumerate(bars):
    plt.text(
        bar.get_x() + bar.get_width() / 2.0,
        bar.get_height() + 0.05,
        f"{room_counts_plot[idx]}\n({room_perc_plot[idx]:.1f}%)",
        ha="center",
        va="bottom",
        fontsize=8,
    )
plt.title("Habitaciones distintas visitadas por episodio")
plt.xlabel("Numero de habitaciones (0-3)")
plt.ylabel("Frecuencia")
plt.xticks(room_categories)
save_plot_if_missing("hist_rooms.png")

plt.figure()
plt.hist(time_in_c_pct, bins=np.arange(0, 105, 5), color="#54A24B")
plt.title("Tiempo en C por episodio")
plt.xlabel("Tiempo en C (%)")
plt.ylabel("Frecuencia")
plt.xlim(0, 100)
save_plot_if_missing("hist_time_in_c.png")

if "room_std" in df.columns:
    plt.figure()
    plt.hist(df["room_std"], bins=20)
    plt.title("Desbalance entre habitaciones (std)")
    plt.xlabel("Std de visitas")
    plt.ylabel("Frecuencia")
    save_plot_if_missing("hist_room_std.png")

if has_columns("hab1_visits", "hab2_visits", "hab3_visits"):
    plt.figure()
    room_labels = ["Hab1", "Hab2", "Hab3"]
    room_means = [
        float(df["hab1_visits"].mean()),
        float(df["hab2_visits"].mean()),
        float(df["hab3_visits"].mean()),
    ]
    dominant_idx = int(np.argmax(room_means))
    colors = ["#4C78A8", "#F58518", "#54A24B"]
    colors[dominant_idx] = "#E45756"
    bars = plt.bar(room_labels, room_means, color=colors)
    for idx, bar in enumerate(bars):
        plt.text(
            bar.get_x() + bar.get_width() / 2.0,
            bar.get_height() + 0.05,
            f"{room_means[idx]:.2f}",
            ha="center",
            va="bottom",
        )
    plt.title(f"Balance entre habitaciones (mas visitada: {room_labels[dominant_idx]})")
    plt.ylabel("Visitas medias por episodio")
    save_plot_if_missing("balance_habs.png")

if "reward_per_step" in df.columns:
    plt.figure()
    plt.hist(df["reward_per_step"], bins=20)
    plt.title("Reward por paso")
    plt.xlabel("Reward/paso")
    plt.ylabel("Frecuencia")
    save_plot_if_missing("hist_reward_per_step.png")

if "min_battery" in df.columns:
    plt.figure()
    plt.hist(df["min_battery"], bins=20)
    plt.title("Bateria minima")
    plt.xlabel("Bateria minima (%)")
    plt.ylabel("Frecuencia")
    plt.xlim(0, 100)
    save_plot_if_missing("hist_min_battery.png")

if has_columns("reward", "reward_per_step", "min_battery") and "room_std" in df.columns:
    plt.figure()
    plt.boxplot(
        [
            df["reward"],
            df["reward_per_step"],
            time_in_c_pct,
            df["min_battery"],
            df["room_std"],
        ]
    )
    plt.xticks([1, 2, 3, 4, 5], ["Reward", "Reward/step", "Time_C(%)", "Battery(%)", "Room_std"])
    plt.title("Distribucion de metricas")
    plt.ylabel("Valor (unidades mixtas)")
    save_plot_if_missing("boxplot_metrics.png")

if "recharge_battery_mean" in df.columns:
    recharge_battery_values = np.asarray(df["recharge_battery_mean"], dtype=np.float32)
    recharge_battery_values = recharge_battery_values[~np.isnan(recharge_battery_values)]
    if recharge_battery_values.size:
        plt.figure()
        plt.hist(recharge_battery_values, bins=15)
        plt.title("Bateria media al recargar (por episodio)")
        plt.xlabel("Bateria (%)")
        plt.ylabel("Frecuencia")
        save_plot_if_missing("hist_battery_at_recharge.png")

if "first_recharge_step" in df.columns:
    first_recharge_values = np.asarray(df["first_recharge_step"], dtype=np.float32)
    first_recharge_values = first_recharge_values[~np.isnan(first_recharge_values)]
    if first_recharge_values.size:
        plt.figure()
        plt.hist(first_recharge_values, bins=15)
        plt.title("Paso de la primera recarga")
        plt.xlabel("Paso")
        plt.ylabel("Frecuencia")
        save_plot_if_missing("hist_first_recharge_step.png")

if has_columns("recharge_from_hab1", "recharge_from_hab2", "recharge_from_hab3", "recharge_from_other"):
    plt.figure()
    room_labels = ["Hab1", "Hab2", "Hab3", "Otro"]
    recharge_counts = [
        int(df["recharge_from_hab1"].sum()),
        int(df["recharge_from_hab2"].sum()),
        int(df["recharge_from_hab3"].sum()),
        int(df["recharge_from_other"].sum()),
    ]
    total_recharges = sum(recharge_counts)
    recharge_pcts = [100.0 * c / max(1, total_recharges) for c in recharge_counts]
    colors = ["#4C78A8", "#F58518", "#54A24B", "#E45756"]
    bars = plt.bar(room_labels, recharge_pcts, color=colors)
    for idx, bar in enumerate(bars):
        plt.text(
            bar.get_x() + bar.get_width() / 2.0,
            bar.get_height() + 0.5,
            f"{recharge_pcts[idx]:.1f}%\n(n={recharge_counts[idx]})",
            ha="center",
            va="bottom",
            fontsize=8,
        )
    plt.title("Origen de recargas (desde qué habitación)")
    plt.ylabel("Porcentaje (%)")
    plt.ylim(0, max(5, max(recharge_pcts) + 10))
    save_plot_if_missing("recharge_origin_rooms.png")

print(
    "Aviso: no se puede reconstruir 'prob_recharge_by_battery_bin.png' "
    "desde el CSV porque requiere datos por decision (no guardados por episodio)."
)

plt.show()

# -- Metricas numericas rapidas ------------------------------------------------
print("\nResumen rapido:")
print(f"Reward medio:              {df['reward'].mean():.2f} +- {df['reward'].std():.2f}")
print(f"Reward por paso medio:     {df['reward_per_step'].mean():.3f} +- {df['reward_per_step'].std():.3f}")
print(f"Habitaciones medias:       {df['rooms'].mean():.2f} / 3")
if "full_coverage" in df.columns:
    print(f"Full coverage rate:        {df['full_coverage'].mean()*100:.1f}%")
print(f"Tiempo medio en C:         {df['time_in_C'].mean()*100:.1f}%")
if "time_out_C" in df.columns:
    print(f"Tiempo fuera de C medio:   {df['time_out_C'].mean()*100:.1f}%")
print(f"Bateria minima media:      {df['min_battery'].mean():.1f}%")
if "charges" in df.columns:
    print(f"Cargas medias:             {df['charges'].mean():.2f}")
if has_columns("hab1_visits", "hab2_visits", "hab3_visits"):
    print(
        "Hab1/Hab2/Hab3 medias:     "
        f"{df['hab1_visits'].mean():.2f} / {df['hab2_visits'].mean():.2f} / {df['hab3_visits'].mean():.2f}"
    )
if "room_std" in df.columns:
    print(f"Desbalance medio (std):    {df['room_std'].mean():.2f}")
if "room_spread" in df.columns:
    print(f"Desbalance medio (max-min): {df['room_spread'].mean():.2f}")
print(f"Graficas guardadas en:     {PLOTS_DIR}")