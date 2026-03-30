"""
Test de múltiples episodios con reset
Simula cómo funcionará el bucle de entrenamiento RL
"""

from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time
import random


class RobotController:
    def __init__(self):
        print("Conectando a CoppeliaSim...")
        self.client = RemoteAPIClient()
        self.sim = self.client.require('sim')
        print("¡Conectado!")
        
        # Iniciar simulación si no está corriendo
        if self.sim.getSimulationState() == self.sim.simulation_stopped:
            print("Iniciando simulación...")
            self.sim.startSimulation()
            time.sleep(1)  # Esperar a que arranque
    
    def get_state(self):
        """Lee el estado actual del robot"""
        battery = self.sim.getInt32Signal('robot_battery')
        current_node = self.sim.getStringSignal('robot_currentNode')
        battery_phase = self.sim.getStringSignal('robot_batteryPhase')
        is_charging = self.sim.getInt32Signal('robot_isCharging') == 1
        status = self.sim.getStringSignal('robot_status')
        battery_depleted = self.sim.getInt32Signal('robot_batteryDepleted') == 1
        
        return {
            'battery': battery if battery is not None else 100,
            'node': current_node if current_node else 'unknown',
            'phase': battery_phase if battery_phase else 'unknown',
            'charging': is_charging,
            'status': status if status else 'unknown',
            'depleted': battery_depleted
        }
    
    def send_command(self, command):
        """Envía un comando al robot"""
        self.sim.setStringSignal('robot_command', command)
    
    def go_to(self, node):
        """Envía al robot a un nodo específico"""
        self.send_command(f'goTo:{node}')
    
    def stop(self, duration=1):
        """Detiene el robot por una duración"""
        self.send_command(f'stop:{duration}')
    
    def call_operator(self):
        """Llama al operario para recarga de emergencia"""
        self.send_command('callOperator')
    
    def reset(self):
        """Reinicia el entorno para un nuevo episodio"""
        self.send_command('reset')
        # Esperar a que el reset se complete
        self.wait_for_status(['ready', 'idle'], timeout=10)
    
    def wait_for_status(self, target_statuses, timeout=60):
        """Espera hasta que el robot alcance uno de los estados objetivo"""
        if isinstance(target_statuses, str):
            target_statuses = [target_statuses]
        
        start_time = time.time()
        while time.time() - start_time < timeout:
            state = self.get_state()
            if state['status'] in target_statuses:
                return state
            time.sleep(0.05)
        
        print(f"Timeout esperando estados: {target_statuses}")
        return self.get_state()
    
    def print_state(self):
        """Imprime el estado actual del robot"""
        state = self.get_state()
        charging = "SÍ" if state['charging'] else "NO"
        depleted = " [DEPLETED!]" if state['depleted'] else ""
        print(f"  Batería: {state['battery']:3d}% | Nodo: {state['node']:4s} | "
              f"Cargando: {charging:2s} | Estado: {state['status']}{depleted}")
    
    def stop_simulation(self):
        """Detiene la simulación"""
        print("Deteniendo simulación...")
        self.sim.stopSimulation()


def run_episode(robot, episode_num, max_actions=10):
    """Ejecuta un episodio completo"""
    print(f"\n{'='*60}")
    print(f"EPISODIO {episode_num}")
    print(f"{'='*60}")
    
    # Reset al inicio del episodio
    print("Reseteando entorno...")
    robot.reset()
    time.sleep(0.2)
    robot.print_state()
    
    # Acciones disponibles
    actions = ['Hab1', 'Hab2', 'Hab3', 'C']
    
    # Contadores del episodio
    actions_taken = 0
    operator_calls = 0
    rooms_visited = []
    
    for step in range(max_actions):
        state = robot.get_state()
        
        # Elegir acción (por ahora aleatoria)
        action = random.choice(actions)
        
        print(f"\n  Paso {step + 1}/{max_actions}: Ir a {action}")
        
        # Si batería agotada, llamar operario
        if state['depleted']:
            print("  !!! Batería agotada - Llamando operario")
            robot.call_operator()
            robot.wait_for_status(['idle'], timeout=60)
            operator_calls += 1
            robot.print_state()
            continue
        
        # Ejecutar acción
        robot.go_to(action)
        final_state = robot.wait_for_status(['arrived', 'idle', 'error', 'depleted'], timeout=120)
        
        actions_taken += 1
        if action in ['Hab1', 'Hab2', 'Hab3']:
            rooms_visited.append(action)
        
        robot.print_state()
        
        # Si se agotó la batería durante el movimiento
        if final_state['depleted']:
            print("  !!! Batería agotada durante movimiento - Llamando operario")
            robot.call_operator()
            robot.wait_for_status(['idle'], timeout=60)
            operator_calls += 1
            robot.print_state()
    
    # Resumen del episodio
    print(f"\n--- Resumen Episodio {episode_num} ---")
    print(f"  Acciones ejecutadas: {actions_taken}")
    print(f"  Llamadas a operario: {operator_calls}")
    print(f"  Habitaciones visitadas: {len(rooms_visited)}")
    
    return {
        'actions': actions_taken,
        'operator_calls': operator_calls,
        'rooms_visited': len(rooms_visited)
    }


def main():
    robot = RobotController()
    
    print("\n" + "="*60)
    print("TEST DE MÚLTIPLES EPISODIOS")
    print("="*60)
    
    num_episodes = 3
    max_actions_per_episode = 8
    
    results = []
    
    try:
        for ep in range(num_episodes):
            result = run_episode(robot, ep + 1, max_actions_per_episode)
            results.append(result)
            time.sleep(0.5)
        
        # Resumen final
        print("\n" + "="*60)
        print("RESUMEN FINAL")
        print("="*60)
        
        total_actions = sum(r['actions'] for r in results)
        total_operator = sum(r['operator_calls'] for r in results)
        total_rooms = sum(r['rooms_visited'] for r in results)
        
        print(f"Episodios completados: {num_episodes}")
        print(f"Total acciones: {total_actions}")
        print(f"Total llamadas operario: {total_operator}")
        print(f"Total habitaciones visitadas: {total_rooms}")
        print(f"Media llamadas operario por episodio: {total_operator/num_episodes:.2f}")
        
    finally:
        # Detener simulación al terminar (importante para headless)
        robot.stop_simulation()


if __name__ == "__main__":
    main()