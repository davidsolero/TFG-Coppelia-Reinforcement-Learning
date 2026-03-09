"""
Control del robot desde Python
Envía comandos y monitoriza el estado
"""

from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time


class RobotController:
    def __init__(self):
        print("Conectando a CoppeliaSim...")
        self.client = RemoteAPIClient()
        self.sim = self.client.require('sim')
        print("¡Conectado!")
    
    def get_state(self):
        """Lee el estado actual del robot"""
        battery = self.sim.getInt32Signal('robot_battery')
        current_node = self.sim.getStringSignal('robot_currentNode')
        battery_phase = self.sim.getStringSignal('robot_batteryPhase')
        is_charging = self.sim.getInt32Signal('robot_isCharging') == 1
        status = self.sim.getStringSignal('robot_status')
        battery_depleted = self.sim.getInt32Signal('robot_batteryDepleted') == 1
        
        return {
            'battery': battery,
            'node': current_node,
            'phase': battery_phase,
            'charging': is_charging,
            'status': status,
            'depleted': battery_depleted
        }
    
    def send_command(self, command):
        """Envía un comando al robot"""
        self.sim.setStringSignal('robot_command', command)
        print(f"Comando enviado: {command}")
    
    def go_to(self, node):
        """Envía al robot a un nodo específico"""
        self.send_command(f'goTo:{node}')
    
    def stop(self, duration=1):
        """Detiene el robot por una duración"""
        self.send_command(f'stop:{duration}')
    
    def call_operator(self):
        """Llama al operario para recarga de emergencia"""
        self.send_command('callOperator')
    
    def wait_for_status(self, target_statuses, timeout=60):
        """Espera hasta que el robot alcance uno de los estados objetivo"""
        if isinstance(target_statuses, str):
            target_statuses = [target_statuses]
        
        start_time = time.time()
        while time.time() - start_time < timeout:
            state = self.get_state()
            if state['status'] in target_statuses:
                return state
            time.sleep(0.1)
        
        print(f"Timeout esperando estados: {target_statuses}")
        return self.get_state()
    
    def print_state(self):
        """Imprime el estado actual del robot"""
        state = self.get_state()
        charging = "SÍ" if state['charging'] else "NO"
        depleted = " [DEPLETED!]" if state['depleted'] else ""
        print(f"Batería: {state['battery']}% | Nodo: {state['node']} | "
              f"Fase: {state['phase']} | Cargando: {charging} | "
              f"Estado: {state['status']}{depleted}")


def main():
    robot = RobotController()
    
    print("\n" + "="*60)
    print("TEST DE CONTROL DEL ROBOT DESDE PYTHON")
    print("="*60)
    
    # Estado inicial
    print("\n--- Estado inicial ---")
    robot.print_state()
    time.sleep(1)
    
    # Secuencia de comandos de prueba
    commands = [
        ('goTo', 'Hab1'),
        ('goTo', 'Hab2'),
        ('goTo', 'C'),      # Ir a cargar
        ('goTo', 'Hab3'),
        ('goTo', 'Hab1'),
    ]
    
    for action, param in commands:
        print(f"\n--- Enviando: {action}:{param} ---")
        
        if action == 'goTo':
            robot.go_to(param)
        elif action == 'stop':
            robot.stop(int(param))
        
        # Esperar a que llegue o falle
        final_state = robot.wait_for_status(['arrived', 'idle', 'error', 'depleted'], timeout=120)
        robot.print_state()
        
        # Si la batería se agotó, llamar al operario
        if final_state['depleted']:
            print("\n!!! BATERÍA AGOTADA - Llamando operario !!!")
            robot.call_operator()
            robot.wait_for_status(['idle'], timeout=60)
            robot.print_state()
        
        time.sleep(0.5)
    
    print("\n" + "="*60)
    print("TEST COMPLETADO")
    print("="*60)


if __name__ == "__main__":
    main()