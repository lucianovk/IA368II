#!/usr/bin/env python3
"""
Script rápido para comparar posições reais no CoppeliaSim.

Lê a pose de /myRobot e de um objeto passado na linha de comando
e imprime posições (x,y,z) e distâncias (plana e 3D). Útil para
conferir se o my_robot_yolo_key_node está estimando corretamente.

Uso:
  python3 scripts/check_object_distance.py /nomeDoObjeto
"""

import math
import sys
from coppeliasim_zmqremoteapi_client import RemoteAPIClient


def get_pose(sim, handle):
    # retorna posição (x,y,z) no frame mundial
    pos = sim.getObjectPosition(handle, -1)
    return pos[0], pos[1], pos[2]


def main():
    if len(sys.argv) < 2:
        print("Uso: python3 scripts/check_object_distance.py /nomeDoObjeto")
        sys.exit(1)

    target_alias = sys.argv[1]
    robot_alias = '/myRobot'

    client = RemoteAPIClient()
    sim = client.require('sim')

    try:
        robot_handle = sim.getObject(robot_alias)
    except Exception as exc:
        print(f"Erro ao pegar {robot_alias}: {exc}")
        sys.exit(1)

    try:
        target_handle = sim.getObject(target_alias)
    except Exception as exc:
        print(f"Erro ao pegar {target_alias}: {exc}")
        sys.exit(1)

    rx, ry, rz = get_pose(sim, robot_handle)
    tx, ty, tz = get_pose(sim, target_handle)

    dx = tx - rx
    dy = ty - ry
    dz = tz - rz

    dist_xy = math.hypot(dx, dy)
    dist_3d = math.sqrt(dx * dx + dy * dy + dz * dz)

    print("=== Posicoes no frame mundial ===")
    print(f"{robot_alias}: x={rx:+.3f}, y={ry:+.3f}, z={rz:+.3f}")
    print(f"{target_alias}: x={tx:+.3f}, y={ty:+.3f}, z={tz:+.3f}")
    print()
    print("=== Distancias ===")
    print(f"Plano xy: {dist_xy:.3f} m")
    print(f"3D:       {dist_3d:.3f} m")


if __name__ == "__main__":
    main()
