#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
import numpy as np
import cv2
import threading
from skimage.feature import peak_local_max

class VoronoiSegmentationNode(Node):
    """
    Segmentação Voronoi + Reparo de Paredes + Fusão de Salas Pequenas.
    
    MELHORIA ATUAL:
    O filtro de área mínima agora FUNDE a sala pequena com o maior vizinho
    em vez de excluí-la. Isso garante cobertura total do mapa.
    """

    def __init__(self):
        super().__init__('voronoi_segmentation_node')

        # --- PARÂMETROS ---
        self.declare_parameter('min_room_area_m2', 3.0) # Salas menores que isso serão fundidas
        self.declare_parameter('wall_fix_kernel_size', 3)
        self.declare_parameter('potential_blur_sigma', 2.0)
        self.declare_parameter('min_room_distance_px', 15)

        # QoS
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )

        # Comms
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self._map_callback, map_qos)
        self.room_pub = self.create_publisher(OccupancyGrid, '/room_map', map_qos)
        
        # Debug
        self.debug_walls_pub = self.create_publisher(OccupancyGrid, '/debug/repaired_walls', map_qos)

        self.timer = self.create_timer(2.0, self._processing_loop)
        self._current_map = None
        self._lock = threading.Lock()
        
        self.get_logger().info('Voronoi Segmentation (Merging Mode) Ready.')

    def _map_callback(self, msg: OccupancyGrid):
        with self._lock:
            self._current_map = msg

    def _processing_loop(self):
        if self._current_map is None: return
        with self._lock: map_msg = self._current_map
        
        try:
            segmented = self._process_segmentation(map_msg)
            if segmented: self.room_pub.publish(segmented)
        except Exception as e:
            self.get_logger().error(f'Processing Error: {e}')

    def _repair_walls(self, grid, height, width):
        """Cria paredes sólidas para evitar vazamento."""
        kernel_size = self.get_parameter('wall_fix_kernel_size').value
        
        # Binarização agressiva
        binary_walls = np.zeros((height, width), dtype=np.uint8)
        binary_walls[grid != 0] = 255 

        # Fechamento
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (kernel_size, kernel_size))
        walls_fixed = cv2.morphologyEx(binary_walls, cv2.MORPH_CLOSE, kernel)
        
        return cv2.bitwise_not(walls_fixed)

    def _merge_small_rooms(self, markers, min_pixels):
        """
        Itera sobre todas as salas detectadas.
        Se area < min_pixels: descobre o vizinho dominante e se funde a ele.
        """
        # Obter lista de IDs e seus tamanhos
        # flatten para contar pixels
        unique, counts = np.unique(markers, return_counts=True)
        
        # Dicionário {id: area}
        area_map = dict(zip(unique, counts))
        
        # Kernel para dilatar e achar vizinhos
        kernel = np.ones((3,3), np.uint8)

        merged_count = 0
        
        # Iteramos do MENOR para o MAIOR para evitar engolir salas grandes
        # Ordenamos os IDs por tamanho de área
        sorted_ids = sorted(unique, key=lambda x: area_map[x])

        for uid in sorted_ids:
            if uid <= 0: continue # Pula fundo (-1) e bordas
            
            area = area_map[uid]
            if area >= min_pixels:
                continue # Sala grande o suficiente, mantém.

            # --- LÓGICA DE FUSÃO ---
            # 1. Criar máscara dessa sala pequena
            mask = (markers == uid).astype(np.uint8)
            
            # 2. Dilatar para encostar nos vizinhos
            dilated = cv2.dilate(mask, kernel, iterations=1)
            
            # 3. Pegar os IDs que estão na borda dilatada
            # Subtraímos a máscara original para pegar SÓ a fronteira externa
            border_mask = dilated - mask
            neighbors = markers[border_mask > 0]
            
            # 4. Filtrar vizinhos inválidos (Fundo/Paredes)
            # Queremos apenas vizinhos > 0 e diferentes de mim mesmo
            valid_neighbors = neighbors[(neighbors > 0) & (neighbors != uid)]
            
            if len(valid_neighbors) > 0:
                # 5. Encontrar o vizinho mais frequente (moda)
                # bincount é mais rápido que scipy.stats.mode
                counts = np.bincount(valid_neighbors)
                dominant_neighbor = np.argmax(counts)
                
                # 6. SUBSTITUIR: A sala pequena vira parte do vizinho
                markers[mask == 1] = dominant_neighbor
                merged_count += 1
            else:
                # Se não tem vizinho válido (ilha isolada), aí sim zeramos
                markers[mask == 1] = 0

        if merged_count > 0:
            self.get_logger().info(f'Fusão: {merged_count} áreas pequenas anexadas a vizinhos maiores.')
            
        return markers

    def _process_segmentation(self, map_msg: OccupancyGrid) -> OccupancyGrid:
        h, w = map_msg.info.height, map_msg.info.width
        res = map_msg.info.resolution
        
        min_area_m2 = self.get_parameter('min_room_area_m2').value
        blur_sigma = self.get_parameter('potential_blur_sigma').value
        min_dist = self.get_parameter('min_room_distance_px').value
        
        grid = np.array(map_msg.data, dtype=np.int8).reshape((h, w))

        # 1. Reparo
        free_space_mask = self._repair_walls(grid, h, w)
        self._publish_debug(self.debug_walls_pub, (free_space_mask == 0).astype(np.uint8)*100, map_msg)

        # 2. Potencial
        dist_map = cv2.distanceTransform(free_space_mask, cv2.DIST_L2, 5)
        potential_field = cv2.GaussianBlur(dist_map, (0, 0), sigmaX=blur_sigma, sigmaY=blur_sigma)

        # 3. Picos
        local_maxima = peak_local_max(
            potential_field, 
            min_distance=int(min_dist),
            labels=free_space_mask,
            exclude_border=False
        )

        markers = np.zeros_like(grid, dtype=np.int32)
        room_count = 0
        for (y, x) in local_maxima:
            if potential_field[y, x] > 0.5:
                room_count += 1
                markers[y, x] = room_count

        if room_count == 0: return None

        # 4. Watershed
        base_img = cv2.cvtColor(free_space_mask, cv2.COLOR_GRAY2BGR)
        markers_final = markers.copy()
        markers_final[free_space_mask == 0] = -1
        cv2.watershed(base_img, markers_final)

        # --- ETAPA 5: FUSÃO INTELIGENTE ---
        min_pixels = int(min_area_m2 / (res * res))
        
        # Chamada da nova função de fusão
        markers_final = self._merge_small_rooms(markers_final, min_pixels)

        # --- ETAPA 6: SAÍDA ---
        final_grid = np.full_like(grid, -1, dtype=np.int8)

        # Pintar salas
        room_area = (markers_final > 0) & (free_space_mask == 255)
        
        if np.any(room_area):
            ids = markers_final[room_area]
            # Cores contrastantes (Costmap Scheme)
            vals = ((ids * 37) % 85) + 10
            final_grid[room_area] = vals.astype(np.int8)

        # Restaurar obstáculos
        final_grid[grid == 100] = 100
        final_grid[(grid == 0) & (final_grid == -1)] = 0

        out_msg = OccupancyGrid()
        out_msg.header = map_msg.header
        out_msg.info = map_msg.info
        out_msg.data = final_grid.flatten().tolist()
        return out_msg

    def _publish_debug(self, pub, data, ref):
        if pub.get_subscription_count() > 0:
            msg = OccupancyGrid()
            msg.header = ref.header
            msg.info = ref.info
            msg.data = data.flatten().astype(np.int8).tolist()
            pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VoronoiSegmentationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()