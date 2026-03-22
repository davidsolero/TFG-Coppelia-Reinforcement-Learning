"""
Test de coste real de acciones.
Mide para cada acción: tiempo real, batería gastada y lo que significa
en términos del episodio RL (cada acción = 1 paso, pero no todas cuestan igual).

Uso:
    1. Abre CoppeliaSim normalmente (con ventana) o headless
    2. Ejecuta: python test_coste_acciones.py

Resultados:
    - Tiempo real por acción (segundos)
    - Batería gastada por acción (%)
    - Comparativa ir a habitación vs ir a C (con carga)
"""

from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time


class RobotController:
    def __init__(self):
        self._client = RemoteAPIClient()
        self._sim = self._client.require('sim')
        if self._sim.getSimulationState() == self._sim.simulation_stopped:
            self._sim.startSimulation()
            time.sleep(1)

    def get_state(self):
        battery  = self._sim.getInt32Signal('robot_battery')
        node     = self._sim.getStringSignal('robot_currentNode')
        status   = self._sim.getStringSignal('robot_status')
        depleted = self._sim.getInt32Signal('robot_batteryDepleted') == 1
        return {
            'battery': battery if battery is not None else 100,
            'node':    node    if node    else 'unknown',
            'status':  status  if status  else 'unknown',
            'depleted': depleted
        }

    def send_command(self, cmd):
        self._sim.setStringSignal('robot_command', cmd)

    def wait_for_status(self, targets, timeout=120):
        if isinstance(targets, str):
            targets = [targets]
        start = time.time()
        while time.time() - start < timeout:
            state = self.get_state()
            if state['status'] in targets:
                return state
            time.sleep(0.05)
        return self.get_state()

    def reset(self):
        self.send_command('reset')
        self.wait_for_status(['ready', 'idle'], timeout=15)

    def go_to(self, node):
        """Ejecuta goTo y devuelve (estado_final, tiempo_real_segundos, bateria_gastada)"""
        state_before = self.get_state()
        bat_before   = state_before['battery']

        t_start = time.time()
        self.send_command(f'goTo:{node}')
        state_after = self.wait_for_status(['arrived', 'idle', 'error', 'depleted'], timeout=120)
        t_end = time.time()

        tiempo_real   = t_end - t_start
        bat_after     = state_after['battery']
        bateria_gasta = bat_before - bat_after

        return state_after, tiempo_real, bateria_gasta

    def stop_simulation(self):
        self._sim.stopSimulation()


def medir_accion(robot, destino, origen_label):
    """Ejecuta una acción y devuelve sus métricas."""
    state, tiempo, bateria = robot.go_to(destino)
    return {
        'origen':  origen_label,
        'destino': destino,
        'tiempo':  tiempo,
        'bateria': bateria,
        'status':  state['status'],
        'nodo_final': state['node'],
    }


def imprimir_resultado(r):
    print(f"  {r['origen']:6s} -> {r['destino']:5s} | "
          f"tiempo: {r['tiempo']:5.2f}s | "
          f"batería: -{r['bateria']:4.1f}% | "
          f"status: {r['status']}")


def main():
    robot = RobotController()

    print("\n" + "="*65)
    print("TEST DE COSTE REAL DE ACCIONES")
    print("Cada acción = 1 paso del episodio RL")
    print("="*65)

    resultados = []

    # ── BLOQUE 1: Desde R, ir a cada destino ─────────────────────────────────
    print("\n[1] Desde R -> cada destino")
    print("-"*65)
    for destino in ['Hab1', 'Hab2', 'Hab3', 'C']:
        robot.reset()
        r = medir_accion(robot, destino, 'R')
        imprimir_resultado(r)
        resultados.append(r)

    # ── BLOQUE 2: Desde C, ir a cada habitación ───────────────────────────────
    print("\n[2] Desde C -> cada habitación")
    print("-"*65)
    for destino in ['Hab1', 'Hab2', 'Hab3']:
        robot.reset()
        robot.go_to('C')  # Primero ir a C
        r = medir_accion(robot, destino, 'C')
        imprimir_resultado(r)
        resultados.append(r)

    # ── BLOQUE 3: Ir a C con batería baja ────────────────────────────────────
    print("\n[3] Ir a C con batería baja (desde Hab1, tras varios movimientos)")
    print("-"*65)
    robot.reset()
    # Gastar batería haciendo movimientos
    for _ in range(5):
        robot.go_to('Hab1')
        robot.go_to('Hab2')
        robot.go_to('Hab3')

    state = robot.get_state()
    print(f"  Batería antes de ir a C: {state['battery']}%")
    r = medir_accion(robot, 'C', state['node'])
    imprimir_resultado(r)
    resultados.append(r)

    # ── BLOQUE 4: Ir a C con batería llena (coste de ir sin necesidad) ───────
    print("\n[4] Ir a C con batería llena (coste de ir sin necesidad)")
    print("-"*65)
    robot.reset()
    robot.go_to('Hab1')  # Salir de R
    r = medir_accion(robot, 'C', 'Hab1')
    imprimir_resultado(r)
    resultados.append(r)

    # ── BLOQUE 5: goTo:C desde C repetido (simula el exploit del agente) ─────
    print("\n[5] goTo:C desde C repetido (simula el exploit del agente)")
    print("-"*65)
    robot.reset()
    robot.go_to('C')  # Ir a C primero

    repeticiones = 40
    tiempos_cc = []
    for i in range(repeticiones):
        _, t, _ = robot.go_to('C')
        tiempos_cc.append(t)
        if i < 5:  # Mostrar solo los primeros 5 para no saturar
            print(f"  C -> C (rep {i+1:2d}) | tiempo: {t:.3f}s")

    print(f"  ... ({repeticiones - 5} más no mostradas)")
    print(f"\n  Total {repeticiones} repeticiones:")
    print(f"    Tiempo real total:   {sum(tiempos_cc):.2f}s")
    print(f"    Tiempo medio/acción: {sum(tiempos_cc)/len(tiempos_cc):.3f}s")
    print(f"    Pasos consumidos:    {repeticiones} (de 50 del episodio)")
    print(f"    => El agente puede gastar {repeticiones} pasos en {sum(tiempos_cc):.2f}s reales sin hacer nada útil")

    # ── RESUMEN ───────────────────────────────────────────────────────────────
    print("\n" + "="*65)
    print("RESUMEN COMPARATIVO")
    print("="*65)

    tiempos_hab  = [r['tiempo'] for r in resultados if r['destino'] in ['Hab1', 'Hab2', 'Hab3']]
    tiempos_c    = [r['tiempo'] for r in resultados if r['destino'] == 'C']
    baterias_hab = [r['bateria'] for r in resultados if r['destino'] in ['Hab1', 'Hab2', 'Hab3']]
    baterias_c   = [r['bateria'] for r in resultados if r['destino'] == 'C']

    print(f"\nIr a habitación (media):  {sum(tiempos_hab)/len(tiempos_hab):.2f}s  |  -{sum(baterias_hab)/len(baterias_hab):.1f}% batería")
    print(f"Ir a C           (media):  {sum(tiempos_c)/len(tiempos_c):.2f}s  |  -{sum(baterias_c)/len(baterias_c):.1f}% batería")
    print(f"\nAmbas acciones cuestan 1 paso del episodio RL.")
    print(f"Diferencia de tiempo real: {abs(sum(tiempos_c)/len(tiempos_c) - sum(tiempos_hab)/len(tiempos_hab)):.2f}s por acción")
    print("="*65)

    robot.stop_simulation()
    print("\nTest completado.")


if __name__ == "__main__":
    main()