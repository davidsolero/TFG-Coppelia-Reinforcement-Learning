"""
Test básico de conexión Python-CoppeliaSim
Ejecutar con la escena abierta y simulación corriendo
"""

from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time


def main():
    print("Conectando a CoppeliaSim...")
    client = RemoteAPIClient()
    sim = client.require('sim')
    
    print("¡Conectado!")
    print(f"Versión de CoppeliaSim: {sim.getInt32Param(sim.intparam_program_version)}")
    
    # Leer estado del robot durante 10 segundos
    print("\nLeyendo estado del robot (10 segundos)...")
    print("-" * 50)
    
    start_time = time.time()
    while time.time() - start_time < 10:
        # Leer señales publicadas por Lua
        battery = sim.getInt32Signal('robot_battery')
        current_node = sim.getStringSignal('robot_currentNode')
        battery_phase = sim.getStringSignal('robot_batteryPhase')
        is_charging = sim.getInt32Signal('robot_isCharging')
        
        # Mostrar estado
        charging_str = "SÍ" if is_charging == 1 else "NO"
        print(f"Batería: {battery}% | Nodo: {current_node} | Fase: {battery_phase} | Cargando: {charging_str}")
        
        time.sleep(1)
    
    print("-" * 50)
    print("Test completado.")


if __name__ == "__main__":
    main()