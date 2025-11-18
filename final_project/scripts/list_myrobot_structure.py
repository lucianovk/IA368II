from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import os
import datetime


def main():
    client = RemoteAPIClient()
    sim = client.require('sim')

    robot_alias = '/myRobot'

    try:
        robot_handle = sim.getObject(robot_alias)
    except Exception as e:
        print(f'ERROR: could not get {robot_alias}: {e}')
        return

    # Pega todos os objetos da subárvore
    obj_handles = sim.getObjectsInTree(robot_handle, sim.handle_all, 0)

    lines = []
    lines.append("=== Dynamic properties dump ===")
    lines.append(f"Timestamp: {datetime.datetime.now().isoformat()}")
    lines.append(f"Robot root: {robot_alias} (handle={robot_handle})")
    lines.append("")
    lines.append("Mass in kg, COM in meters (shape frame), inertia matrix in kg·m² (shape frame).")
    lines.append("")

    for h in obj_handles:
        alias = sim.getObjectAlias(h, 2)
        obj_type = sim.getObjectType(h)

        # Só shapes têm massa/inércia
        if obj_type != sim.object_shape_type:
            continue

        try:
            mass = sim.getShapeMass(h)
            inertia, com_matrix = sim.getShapeInertia(h)
        except Exception as e:
            lines.append(f"# WARNING: could not get mass/inertia for {alias} (handle={h}): {e}")
            lines.append("")
            continue

        # COM vem codificado em com_matrix (12 floats). Em versões recentes:
        # é uma matriz só de translação, então a posição é (com_matrix[3], com_matrix[7], com_matrix[11])
        cx = com_matrix[3]
        cy = com_matrix[7]
        cz = com_matrix[11]

        Ixx, Ixy, Ixz, Iyx, Iyy, Iyz, Izx, Izy, Izz = inertia

        lines.append(f"Shape: {alias}")
        lines.append(f"  Handle: {h}")
        lines.append(f"  Mass: {mass:.6f} kg")
        lines.append(f"  COM (shape frame): "
                     f"[{cx:+.6f}, {cy:+.6f}, {cz:+.6f}]")
        lines.append("  Inertia matrix (shape frame):")
        lines.append(f"    [{Ixx:+.6e}  {Ixy:+.6e}  {Ixz:+.6e}]")
        lines.append(f"    [{Iyx:+.6e}  {Iyy:+.6e}  {Iyz:+.6e}]")
        lines.append(f"    [{Izx:+.6e}  {Izy:+.6e}  {Izz:+.6e}]")
        lines.append("")

    out_filename = "myrobot_dynamics_fixed.txt"
    out_path = os.path.abspath(out_filename)

    with open(out_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))

    print(f"Dynamic properties written to: {out_path}")


if __name__ == "__main__":
    main()
