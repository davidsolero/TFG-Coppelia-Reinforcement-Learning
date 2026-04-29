"""
Script simple para graficar bateria con parametros fijos (los actuales de Lua).

No lee archivos ni usa argumentos de consola.
"""

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


# Parametros actuales (script pioneer.lua)
DISCHARGE_PLATEAU_DURATION = 20.0
DISCHARGE_FALL_DURATION = 55.0
CHARGE_DURATION = 20.0

# Parametros del grafico
DT = 0.1
RECHARGE_THRESHOLD = 0.0
TOTAL_TIME = 320.0
CHARGE_START_BATTERY = 0.0
CHARGE_PLATEAU_SECONDS = 8.0


def discharge_curve() -> tuple[np.ndarray, np.ndarray]:
    t1 = np.arange(0.0, DISCHARGE_PLATEAU_DURATION + DT, DT)
    b1 = np.full_like(t1, 100.0, dtype=float)

    t2 = np.arange(DT, DISCHARGE_FALL_DURATION + DT, DT)
    b2 = 100.0 * (1.0 - (t2 / DISCHARGE_FALL_DURATION))
    b2 = np.clip(b2, 0.0, 100.0)

    t = np.concatenate([t1, DISCHARGE_PLATEAU_DURATION + t2])
    b = np.concatenate([b1, b2])
    return t, b


def periodic_cycle() -> tuple[np.ndarray, np.ndarray]:
    times = [0.0]
    batteries = [100.0]

    t = 0.0
    state = "plateau"
    state_t = 0.0
    battery = 100.0
    charge_start = 100.0

    while t < TOTAL_TIME:
        t += DT
        state_t += DT

        if state == "plateau":
            battery = 100.0
            if state_t >= DISCHARGE_PLATEAU_DURATION:
                state = "fall"
                state_t = 0.0

        elif state == "fall":
            battery = 100.0 * (1.0 - state_t / DISCHARGE_FALL_DURATION)
            battery = max(0.0, battery)
            if battery <= RECHARGE_THRESHOLD:
                state = "charge"
                state_t = 0.0
                charge_start = battery

        else:  # charge
            charge_time = CHARGE_DURATION * ((100.0 - charge_start) / 100.0)
            if charge_time <= 0.0:
                battery = 100.0
                state = "plateau"
                state_t = 0.0
            else:
                battery = charge_start + (100.0 - charge_start) * min(state_t / charge_time, 1.0)
                if battery >= 100.0:
                    battery = 100.0
                    state = "plateau"
                    state_t = 0.0

        times.append(t)
        batteries.append(battery)

    return np.array(times), np.array(batteries)


def charge_curve(start_battery: float) -> tuple[np.ndarray, np.ndarray]:
    start = float(np.clip(start_battery, 0.0, 100.0))
    charge_time = CHARGE_DURATION * ((100.0 - start) / 100.0)

    t_rise = np.arange(0.0, charge_time + DT, DT)
    if charge_time <= 0.0:
        b_rise = np.full_like(t_rise, 100.0, dtype=float)
    else:
        b_rise = start + (100.0 - start) * np.clip(t_rise / charge_time, 0.0, 1.0)

    # En Lua existe fase charge_plateau: mientras siga cargando, la bateria queda en 100%.
    t_plateau = np.arange(DT, CHARGE_PLATEAU_SECONDS + DT, DT)
    b_plateau = np.full_like(t_plateau, 100.0, dtype=float)

    t = np.concatenate([t_rise, charge_time + t_plateau])
    b = np.concatenate([b_rise, b_plateau])
    return t, b


def main() -> None:
    root = Path(__file__).resolve().parent.parent
    output_dir = root / "redaccion" / "memoria" / "graficos"
    output_dir.mkdir(parents=True, exist_ok=True)

    # 1) Curva de descarga simple
    t_dis, b_dis = discharge_curve()
    plt.figure(figsize=(10, 4.5))
    plt.plot(t_dis, b_dis, color="#1f77b4", linewidth=2.2)
    plt.axvline(DISCHARGE_PLATEAU_DURATION, linestyle="--", color="#555555", linewidth=1.0)
    plt.title("Descarga de bateria (parametros actuales)")
    plt.xlabel("Tiempo (s)")
    plt.ylabel("Bateria (%)")
    plt.ylim(0, 105)
    plt.grid(alpha=0.3)
    plt.tight_layout()
    plt.savefig(output_dir / "battery_discharge_model.png", dpi=160)
    plt.close()

    # 2) Ciclo periodico descarga-recarga
    t_cycle, b_cycle = periodic_cycle()
    plt.figure(figsize=(11, 4.5))
    plt.plot(t_cycle, b_cycle, color="#2ca02c", linewidth=2.0)
    plt.axhline(RECHARGE_THRESHOLD, linestyle="--", color="#d62728", linewidth=1.2)
    plt.title("Ciclo periodico de bateria (umbral de recarga)")
    plt.xlabel("Tiempo (s)")
    plt.ylabel("Bateria (%)")
    plt.ylim(0, 105)
    plt.grid(alpha=0.3)
    plt.tight_layout()
    plt.savefig(output_dir / "battery_periodic_cycle_model.png", dpi=160)
    plt.close()

    # 3) Recarga (subida + meseta)
    t_charge, b_charge = charge_curve(CHARGE_START_BATTERY)
    charge_time = CHARGE_DURATION * ((100.0 - CHARGE_START_BATTERY) / 100.0)
    plt.figure(figsize=(10, 4.5))
    plt.plot(t_charge, b_charge, color="#ff7f0e", linewidth=2.2)
    plt.axvline(charge_time, linestyle="--", color="#555555", linewidth=1.0)
    plt.title("Recarga de bateria (subida + meseta)")
    plt.xlabel("Tiempo (s)")
    plt.ylabel("Bateria (%)")
    plt.ylim(0, 105)
    plt.grid(alpha=0.3)
    plt.tight_layout()
    plt.savefig(output_dir / "battery_charge_model.png", dpi=160)
    plt.close()

    print("Graficas guardadas en:", output_dir)
    print("- battery_discharge_model.png")
    print("- battery_periodic_cycle_model.png")
    print("- battery_charge_model.png")


if __name__ == "__main__":
    main()
