#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid, MapMetaData
import numpy as np
import cv2
import threading
from skimage.feature import peak_local_max
import sys
import os
import yaml
import argparse
from geometry_msgs.msg import Pose, Point, Quaternion

class VoronoiSegmentationNode(Node):
    """
    Segmentação Voronoi + Reparo de Paredes + Fusão de Salas Pequenas.
    Suporta execução via ROS (Topic) ou Terminal (Arquivos Estáticos).
    """

    def __init__(self, offline_config=None):
        super().__init__('voronoi_segmentation_node')

        # --- CONFIGURAÇÃO (Híbrida: ROS Params ou Config Manual) ---
        self.offline_mode = offline_config is not None
        
        if self.offline_mode:
            # Modo Terminal: Usa o dicionário passado
            self.p_min_area = offline_config.get('min_room_area_m2', 3.0)
            self.p_kernel_size = offline_config.get('wall_fix_kernel_size', 3)
            self.p_blur_sigma = offline_config.get('potential_blur_sigma', 2.0)
            self.p_min_dist = offline_config.get('min_room_distance_px', 15)
            self.get_logger().info("Iniciando em MODO OFFLINE (Sem tópicos).")
        else:
            # Modo ROS: Declara e lê parâmetros do servidor
            self.declare_parameter('min_room_area_m2', 3.0)
            self.declare_parameter('wall_fix_kernel_size', 3)
            self.declare_parameter('potential_blur_sigma', 2.0)
            self.declare_parameter('min_room_distance_px', 15)
            
            self.p_min_area = self.get_parameter('min_room_area_m2').value
            self.p_kernel_size = self.get_parameter('wall_fix_kernel_size').value
            self.p_blur_sigma = self.get_parameter('potential_blur_sigma').value
            self.p_min_dist = self.get_parameter('min_room_distance_px').value

            # QoS e Subs apenas se online
            map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
            self.map_sub = self.create_subscription(OccupancyGrid, '/map', self._map_callback, map_qos)
            self.room_pub = self.create_publisher(OccupancyGrid, '/topologic_map', map_qos)
            self.debug_walls_pub = self.create_publisher(OccupancyGrid, '/debug/repaired_walls', map_qos)
            self.timer = self.create_timer(2.0, self._processing_loop)
            self.get_logger().info('Voronoi Segmentation Node Ready.')

        self._current_map = None
        self._lock = threading.Lock()

    def _map_callback(self, msg: OccupancyGrid):
        with self._lock:
            self._current_map = msg

    def _processing_loop(self):
        if self._current_map is None: return
        with self._lock: map_msg = self._current_map
        
        try:
            segmented = self.process_segmentation(map_msg)
            if segmented and not self.offline_mode: 
                self.room_pub.publish(segmented)
        except Exception as e:
            self.get_logger().error(f'Processing Error: {e}')

    def _repair_walls(self, grid, height, width):
        """Cria paredes sólidas para evitar vazamento."""
        # Binarização agressiva
        binary_walls = np.zeros((height, width), dtype=np.uint8)
        binary_walls[grid != 0] = 255 

        # Fechamento
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (self.p_kernel_size, self.p_kernel_size))
        walls_fixed = cv2.morphologyEx(binary_walls, cv2.MORPH_CLOSE, kernel)
        
        return cv2.bitwise_not(walls_fixed)

    def _merge_small_rooms(self, markers, min_pixels):
        """Itera sobre todas as salas detectadas e funde as pequenas."""
        unique, counts = np.unique(markers, return_counts=True)
        area_map = dict(zip(unique, counts))
        kernel = np.ones((3,3), np.uint8)
        merged_count = 0
        sorted_ids = sorted(unique, key=lambda x: area_map[x])

        for uid in sorted_ids:
            if uid <= 0: continue 
            area = area_map[uid]
            if area >= min_pixels: continue

            mask = (markers == uid).astype(np.uint8)
            dilated = cv2.dilate(mask, kernel, iterations=1)
            border_mask = dilated - mask
            neighbors = markers[border_mask > 0]
            valid_neighbors = neighbors[(neighbors > 0) & (neighbors != uid)]
            
            if len(valid_neighbors) > 0:
                counts = np.bincount(valid_neighbors)
                dominant_neighbor = np.argmax(counts)
                markers[mask == 1] = dominant_neighbor
                merged_count += 1
            else:
                markers[mask == 1] = 0 # Ilha isolada

        if merged_count > 0:
            self.get_logger().debug(f'Fusão: {merged_count} áreas pequenas anexadas a vizinhos maiores.')
        return markers

    # Renomeei de _process_segmentation para process_segmentation (publico)
    def process_segmentation(self, map_msg: OccupancyGrid) -> OccupancyGrid:
        h, w = map_msg.info.height, map_msg.info.width
        res = map_msg.info.resolution
        
        # Usa os parâmetros armazenados no self
        grid = np.array(map_msg.data, dtype=np.int8).reshape((h, w))

        # 1. Reparo
        free_space_mask = self._repair_walls(grid, h, w)
        if not self.offline_mode:
             self._publish_debug(self.debug_walls_pub, (free_space_mask == 0).astype(np.uint8)*100, map_msg)

        # 2. Potencial
        dist_map = cv2.distanceTransform(free_space_mask, cv2.DIST_L2, 5)
        potential_field = cv2.GaussianBlur(dist_map, (0, 0), sigmaX=self.p_blur_sigma, sigmaY=self.p_blur_sigma)

        # 3. Picos
        local_maxima = peak_local_max(
            potential_field, 
            min_distance=int(self.p_min_dist),
            labels=free_space_mask,
            exclude_border=False
        )

        markers = np.zeros_like(grid, dtype=np.int32)
        room_count = 0
        for (y, x) in local_maxima:
            if potential_field[y, x] > 0.5:
                room_count += 1
                markers[y, x] = room_count

        if room_count == 0: 
            self.get_logger().warn("Nenhuma sala detectada.")
            return None

        # 4. Watershed
        base_img = cv2.cvtColor(free_space_mask, cv2.COLOR_GRAY2BGR)
        markers_final = markers.copy()
        markers_final[free_space_mask == 0] = -1
        cv2.watershed(base_img, markers_final)

        # 5. Fusão
        min_pixels = int(self.p_min_area / (res * res))
        markers_final = self._merge_small_rooms(markers_final, min_pixels)

        # 6. Saída
        final_grid = np.full_like(grid, -1, dtype=np.int8)
        room_area = (markers_final > 0) & (free_space_mask == 255)
        
        if np.any(room_area):
            ids = markers_final[room_area]
            # Algoritmo de hash simples para consistência de cor no ROS
            vals = ((ids * 37) % 85) + 10 
            final_grid[room_area] = vals.astype(np.int8)

        # Obstáculos originais
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

# --- FUNÇÕES UTILITÁRIAS PARA MODO OFFLINE ---

def load_map_from_yaml(yaml_path):
    """Carrega .yaml e .pgm e converte para mensagem OccupancyGrid."""
    with open(yaml_path, 'r') as f:
        map_data = yaml.safe_load(f)

    # Resolve caminho da imagem (relativo ao yaml)
    image_path = map_data['image']
    if not os.path.isabs(image_path):
        image_path = os.path.join(os.path.dirname(yaml_path), image_path)

    if not os.path.exists(image_path):
        raise FileNotFoundError(f"Imagem do mapa não encontrada: {image_path}")

    # Carrega imagem em escala de cinza
    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise ValueError("Falha ao abrir imagem do mapa com OpenCV.")

    # PGM loadado pelo OpenCV: 255 = livre (branco), 0 = ocupado (preto), ~205 = desconhecido
    # ROS OccupancyGrid: 0 = livre, 100 = ocupado, -1 = desconhecido
    
    # Se o modo for 'trinary', precisamos converter
    mode = map_data.get('mode', 'trinary')
    thresh_occ = map_data.get('occupied_thresh', 0.65)
    thresh_free = map_data.get('free_thresh', 0.196)
    negate = map_data.get('negate', 0)

    # Normaliza 0-1
    img_norm = img.astype(float) / 255.0
    if negate:
        img_norm = 1.0 - img_norm

    grid_data = np.full(img.shape, -1, dtype=np.int8)
    
    # Lógica padrão do map_server
    grid_data[img_norm >= thresh_occ] = 100
    grid_data[img_norm <= thresh_free] = 0
    # O resto fica -1

    # Inverte Y (imagem OpenCV (0,0) é topo-esquerda, ROS Map é baixo-esquerda geralmente)
    # ATENÇÃO: map_server geralmente já carrega correto, mas dependendo de como o pgm foi salvo
    # pode precisar de flip. O padrão ROS é 'flipado' verticalmente em relação a imagens comuns.
    grid_data = np.flipud(grid_data)

    # Cria mensagem
    msg = OccupancyGrid()
    msg.info.resolution = map_data['resolution']
    msg.info.width = img.shape[1]
    msg.info.height = img.shape[0]
    
    origin = map_data['origin'] # [x, y, yaw]
    msg.info.origin = Pose(position=Point(x=origin[0], y=origin[1], z=0.0))
    # (Ignorando quaternion complexo por simplificação, já que só precisamos da grid)
    
    msg.data = grid_data.flatten().tolist()
    return msg

def save_colored_map(occupancy_msg, output_path):
    """Converte OccupancyGrid segmentado para JPG colorido."""
    w = occupancy_msg.info.width
    h = occupancy_msg.info.height
    data = np.array(occupancy_msg.data, dtype=np.int8).reshape((h, w))
    
    # Precisamos 'desflipar' para salvar como imagem visível corretamente
    data = np.flipud(data)

    # Cria imagem colorida (BGR)
    # Fundo (Desconhecido) = Cinza
    # Obstáculo = Preto
    # Salas = Pseudo-Cores
    
    # Normaliza IDs das salas para 0-255 para aplicar colormap
    # IDs no OccupancyGrid estão entre 10 e 100 (aprox) pela lógica do nó, ou -1, ou 100
    
    visual_img = np.zeros((h, w, 3), dtype=np.uint8)
    
    # Máscaras
    mask_unknown = (data == -1)
    mask_obstacle = (data == 100)
    mask_rooms = (data >= 0) & (data != 100)

    # 1. Pinta tudo de cinza claro (Desconhecido)
    visual_img[mask_unknown] = [200, 200, 200]
    
    # 2. Pinta obstáculos de preto
    visual_img[mask_obstacle] = [0, 0, 0]

    # 3. Salas com cores
    if np.any(mask_rooms):
        room_vals = data[mask_rooms]
        
        # Expande o range para usar todo o espectro de cores (0-255)
        # O nó gera valores entre ~10 e ~95. Vamos esticar isso.
        normalized = ((room_vals.astype(float) * 5) % 255).astype(np.uint8)
        
        colored_layer = np.zeros((h, w), dtype=np.uint8)
        colored_layer[mask_rooms] = normalized
        
        # Aplica mapa de calor
        heatmap = cv2.applyColorMap(colored_layer, cv2.COLORMAP_JET)
        
        # Copia apenas os pixels das salas para a imagem final
        visual_img[mask_rooms] = heatmap[mask_rooms]

    cv2.imwrite(output_path, visual_img)
    print(f"Resultado salvo em: {output_path}")

def main(args=None):
    # Verifica argumentos de linha de comando
    parser = argparse.ArgumentParser(description="Segmentação Voronoi de Mapas ROS.")
    parser.add_argument('--map', type=str, help="Caminho para o arquivo .yaml do mapa.")
    parser.add_argument('--output', type=str, default="segmented_map.jpg", help="Caminho para salvar o JPG resultante.")
    
    # Truque para conviver com o rclpy e argparse
    # Filtra apenas argumentos conhecidos, o resto (se houver) deixa pro rclpy
    known_args, unknown_args = parser.parse_known_args()

    if known_args.map:
        # --- MODO OFFLINE (TERMINAL) ---
        print(f"Carregando mapa: {known_args.map}")
        
        try:
            # 1. Carrega Mapa
            map_msg = load_map_from_yaml(known_args.map)
            
            # 2. Inicializa ROS (Minimamente necessário para criar a Classe Node)
            rclpy.init(args=unknown_args)
            
            # Configuração manual (pode ajustar aqui se quiser)
            config = {
                'min_room_area_m2': 3.0,
                'wall_fix_kernel_size': 3,
                'potential_blur_sigma': 2.0,
                'min_room_distance_px': 15
            }
            
            node = VoronoiSegmentationNode(offline_config=config)
            
            # 3. Processa
            print("Processando segmentação...")
            segmented_msg = node.process_segmentation(map_msg)
            
            if segmented_msg:
                # 4. Salva
                save_colored_map(segmented_msg, known_args.output)
            else:
                print("Erro: A segmentação não retornou resultado.")
            
            node.destroy_node()
            rclpy.shutdown()
            
        except Exception as e:
            print(f"Erro Crítico: {e}")
            sys.exit(1)

    else:
        # --- MODO ONLINE (ROS NODE) ---
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
