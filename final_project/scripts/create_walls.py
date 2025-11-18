from coppeliasim_zmqremoteapi_client import RemoteAPIClient

def main():
    client = RemoteAPIClient()
    sim = client.require('sim')

    # ---- Dimensões da sala ----
    room_size = 6.0          # tamanho interno (6 × 6 m)
    wall_thickness = 0.10    # espessura das paredes (10 cm)
    wall_height = 1.0        # altura das paredes (1 m)

    # Sala centrada em (0,0): paredes externas a 3m do centro + metade da espessura
    offset = room_size / 2 + wall_thickness / 2   # 3.05 m
    z_center = wall_height / 2                   # 0.5 m

    door_width = 0.70       # largura das passagens (70 cm)
    door_half = door_width / 2.0

    def create_wall(alias, size, pos):
        """
        Cria uma parede retangular estática, respondable,
        com máscara cheia, e colidível.
        size = [sx, sy, sz]
        pos  = [x, y, z] em relação ao mundo (-1)
        """
        handle = sim.createPrimitiveShape(sim.primitiveshape_cuboid, size, 0)
        sim.setObjectAlias(handle, alias)
        sim.setObjectPosition(handle, pos, -1)

        # Tornar estático e respondable
        sim.setObjectInt32Param(handle, sim.shapeintparam_static, 1)
        sim.setObjectInt32Param(handle, sim.shapeintparam_respondable, 1)

        # Máscara respondable: tudo ligado (local + global)
        # 0xFFFF = 1111111111111111 em binário
        sim.setObjectInt32Param(handle, sim.shapeintparam_respondable_mask, 0xFFFF)

        # Marcar como "collidable" (útil para checkCollision, etc.)
        props = sim.getObjectSpecialProperty(handle)
        props |= sim.objectspecialproperty_collidable
        sim.setObjectSpecialProperty(handle, props)

        return handle

    # ------------------------------------------------------------------
    # PAREDES EXTERNAS (sem portas)
    # ------------------------------------------------------------------

    # Norte (y > 0), comprimento no X = 6
    create_wall(
        '/room/wall_north',
        [room_size, wall_thickness, wall_height],
        [0.0, offset, z_center]
    )

    # Sul (y < 0)
    create_wall(
        '/room/wall_south',
        [room_size, wall_thickness, wall_height],
        [0.0, -offset, z_center]
    )

    # Leste (x > 0), comprimento no Y = 6
    create_wall(
        '/room/wall_east',
        [wall_thickness, room_size, wall_height],
        [offset, 0.0, z_center]
    )

    # Oeste (x < 0)
    create_wall(
        '/room/wall_west',
        [wall_thickness, room_size, wall_height],
        [-offset, 0.0, z_center]
    )

    # ------------------------------------------------------------------
    # PAREDE CENTRAL NORTE–SUL (x = 0) COM PORTA DESCENTRALIZADA
    # ------------------------------------------------------------------
    # Extensão interna em Y: [-3, +3]
    # Porta de 70 cm em torno de y = -2.0 (mais perto da parte sul)
    door_center_y = -2.0

    seg1_y_min = -3.0
    seg1_y_max = door_center_y - door_half   # -2.35
    seg1_len   = seg1_y_max - seg1_y_min     # 0.65
    seg1_y_mid = 0.5 * (seg1_y_min + seg1_y_max)  # -2.675

    seg2_y_min = door_center_y + door_half   # -1.65
    seg2_y_max = 3.0
    seg2_len   = seg2_y_max - seg2_y_min     # 4.65
    seg2_y_mid = 0.5 * (seg2_y_min + seg2_y_max)  # 0.675

    # Segmento 1 (parte de baixo)
    create_wall(
        '/room/wall_divider_center_seg1',
        [wall_thickness, seg1_len, wall_height],
        [0.0, seg1_y_mid, z_center]
    )

    # Segmento 2 (parte de cima)
    create_wall(
        '/room/wall_divider_center_seg2',
        [wall_thickness, seg2_len, wall_height],
        [0.0, seg2_y_mid, z_center]
    )

    # ------------------------------------------------------------------
    # PAREDE INTERNA OESTE LESTE–OESTE (divide A1/A2) COM PORTA
    # ------------------------------------------------------------------
    # Metade oeste: x ∈ [-3, 0]
    # Nova divisão A1/A2: y = 0 → centro da parede em y = 0.0
    west_divider_y = 0.0

    # Porta de 70 cm não centralizada, mais próxima da parede oeste:
    # centro da porta em x = -2.0
    door_center_x_west = -2.0

    w_seg1_x_min = -3.0
    w_seg1_x_max = door_center_x_west - door_half   # -2.35
    w_seg1_len   = w_seg1_x_max - w_seg1_x_min      # 0.65
    w_seg1_x_mid = 0.5 * (w_seg1_x_min + w_seg1_x_max)  # -2.675

    w_seg2_x_min = door_center_x_west + door_half   # -1.65
    w_seg2_x_max = 0.0
    w_seg2_len   = w_seg2_x_max - w_seg2_x_min      # 1.65
    w_seg2_x_mid = 0.5 * (w_seg2_x_min + w_seg2_x_max)  # -0.825

    # Segmento oeste-esquerda
    create_wall(
        '/room/wall_divider_west_seg1',
        [w_seg1_len, wall_thickness, wall_height],
        [w_seg1_x_mid, west_divider_y, z_center]
    )

    # Segmento oeste-direita
    create_wall(
        '/room/wall_divider_west_seg2',
        [w_seg2_len, wall_thickness, wall_height],
        [w_seg2_x_mid, west_divider_y, z_center]
    )

    # ------------------------------------------------------------------
    # PAREDE INTERNA LESTE LESTE–OESTE (divide B1/B2) COM PORTA
    # ------------------------------------------------------------------
    # Metade leste: x ∈ [0, 3]
    # Mantemos a divisão B1/B2 com parede em y ≈ -1 (interno)
    # centro em y = -1.05
    east_divider_y = -1.05

    # Porta não centralizada, mais próxima da parede leste:
    # centro da porta em x = +2.0
    door_center_x_east = 2.0

    e_seg1_x_min = 0.0
    e_seg1_x_max = door_center_x_east - door_half   # 1.65
    e_seg1_len   = e_seg1_x_max - e_seg1_x_min      # 1.65
    e_seg1_x_mid = 0.5 * (e_seg1_x_min + e_seg1_x_max)  # 0.825

    e_seg2_x_min = door_center_x_east + door_half   # 2.35
    e_seg2_x_max = 3.0
    e_seg2_len   = e_seg2_x_max - e_seg2_x_min      # 0.65
    e_seg2_x_mid = 0.5 * (e_seg2_x_min + e_seg2_x_max)  # 2.675

    # Segmento leste-esquerda
    create_wall(
        '/room/wall_divider_east_seg1',
        [e_seg1_len, wall_thickness, wall_height],
        [e_seg1_x_mid, east_divider_y, z_center]
    )

    # Segmento leste-direita
    create_wall(
        '/room/wall_divider_east_seg2',
        [e_seg2_len, wall_thickness, wall_height],
        [e_seg2_x_mid, east_divider_y, z_center]
    )

    print("Sala 6×6 m com A1=9m², paredes internas, passagens de 70 cm e máscaras configuradas criada com sucesso!")

if __name__ == "__main__":
    main()
