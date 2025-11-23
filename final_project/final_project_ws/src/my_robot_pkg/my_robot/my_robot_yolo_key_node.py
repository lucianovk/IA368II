#!/usr/bin/env python3
import atexit
import math
import select
import sys
import termios
import tty
from typing import Optional, Tuple

import numpy as np
import rclpy
from rclpy.exceptions import ParameterAlreadyDeclaredException
from rclpy.node import Node
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener, TransformException
from geometry_msgs.msg import PointStamped
from tf2_geometry_msgs import do_transform_point

try:
    import cv2  # type: ignore
except Exception:  # pragma: no cover
    cv2 = None

try:
    from ultralytics import YOLO  # type: ignore
except Exception:
    YOLO = None


class YoloKeyNode(Node):
    """
    Lê /camera e roda YOLO ao pressionar 'c'; sai ao pressionar 'q'.
    Resultado salvo em yolo_result.jpg com bbox/label quando houver detecção.
    """

    def __init__(self):
        super().__init__('my_robot_yolo_key_node')

        try:
            self.declare_parameter('camera_topic', '/camera')
            self.declare_parameter('depth_topic', '/camera_depth')
            self.declare_parameter('base_frame', 'base_footprint')
            # defaults para 640x480 e FOV ~57°
            self.declare_parameter('fx', 585.5)
            self.declare_parameter('fy', 585.5)
            self.declare_parameter('cx', 319.5)
            self.declare_parameter('cy', 239.5)
            self.declare_parameter('fov_deg', 57.0)
            self.declare_parameter('use_sim_time', False)
        except ParameterAlreadyDeclaredException:
            pass

        self.camera_topic = self.get_parameter('camera_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.base_frame = self.get_parameter('base_frame').value
        self.fx = float(self.get_parameter('fx').value)
        self.fy = float(self.get_parameter('fy').value)
        self.cx = float(self.get_parameter('cx').value)
        self.cy = float(self.get_parameter('cy').value)
        self.fov_deg = float(self.get_parameter('fov_deg').value)
        self.intrinsics_computed = False
        self.last_image: Optional[Image] = None
        self.last_depth: Optional[Tuple[Image, np.ndarray]] = None
        self.det_counter = 0

        self.sub = self.create_subscription(
            Image, self.camera_topic, self.image_callback, 10
        )
        self.depth_sub = self.create_subscription(
            Image, self.depth_topic, self.depth_callback, 10
        )

        # TF buffer para mapear camera_link -> map
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Marker publisher
        self.marker_pub = self.create_publisher(MarkerArray, '/yolo_detections', 10)
        self.last_marker_ids = []

        # Modelo YOLO
        if YOLO is None:
            self.get_logger().error(
                'Pacote ultralytics não disponível. Instale e tente novamente.'
            )
            self.model = None
        else:
            try:
                self.model = YOLO('/dados/pessoal/sandbox/IA368II/github_lvk/final_project/final_project_ws/models/yolo11n.pt')
                self.get_logger().info('Modelo YOLO11n carregado de models/yolo11n.pt.')
            except Exception as exc:
                self.get_logger().error(f'Falha ao carregar YOLO11n: {exc}')
                self.model = None

        # Entrada de teclado não-bloqueante
        self.orig_term_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        atexit.register(self.restore_terminal)

        # Timer para polling do teclado
        self.timer = self.create_timer(0.1, self.poll_keyboard)
        self.get_logger().info(
            "Pressione 'c' para capturar e rodar YOLO na última imagem; 'q' para sair."
        )

    def restore_terminal(self):
        try:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.orig_term_settings)
        except Exception:
            pass

    def image_callback(self, msg: Image):
        # Calcula intrínsecos uma vez com base na resolução e fov se não vierem setados
        if not self.intrinsics_computed and msg.width > 0 and msg.height > 0:
            cx_calc = (msg.width - 1) * 0.5
            cy_calc = (msg.height - 1) * 0.5
            fov_rad = math.radians(self.fov_deg if self.fov_deg > 0 else 57.0)
            fx_calc = msg.width / (2.0 * math.tan(fov_rad * 0.5))
            fy_calc = fx_calc
            if self.cx <= 0:
                self.cx = cx_calc
            if self.cy <= 0:
                self.cy = cy_calc
            if self.fx <= 0:
                self.fx = fx_calc
            if self.fy <= 0:
                self.fy = fy_calc
            self.intrinsics_computed = True
            self.get_logger().info(
                f'Intrinsecos definidos: fx={self.fx:.2f}, fy={self.fy:.2f}, '
                f'cx={self.cx:.2f}, cy={self.cy:.2f} (res {msg.width}x{msg.height}, fov {self.fov_deg:.2f}°)'
            )
        self.last_image = msg

    def depth_callback(self, msg: Image):
        if msg.encoding.lower() not in ('32fc1', '32f'):
            self.get_logger().warn_once(f'Encoding depth não suportado: {msg.encoding}')
            return
        arr = np.frombuffer(msg.data, dtype=np.float32)
        expected = msg.width * msg.height
        if arr.size != expected:
            self.get_logger().warn_once(
                f'Tamanho depth inesperado: {arr.size} vs {expected}'
            )
            return
        arr = arr.reshape((msg.height, msg.width))
        self.last_depth = (msg, arr)

    def poll_keyboard(self):
        if not sys.stdin.isatty():
            return
        rlist, _, _ = select.select([sys.stdin], [], [], 0)
        if not rlist:
            return

        ch = sys.stdin.read(1)
        if ch.lower() == 'q':
            self.get_logger().info('Tecla q pressionada. Encerrando nó.')
            self.restore_terminal()
            rclpy.shutdown()
        elif ch.lower() == 'c':
            self.get_logger().info('Tecla c pressionada. Rodando detecção YOLO...')
            self.run_detection()

    def run_detection(self):
        if self.model is None:
            self.get_logger().warn('YOLO não carregado; não posso rodar detecção.')
            return
        if self.last_image is None:
            self.get_logger().warn('Nenhuma imagem recebida em /camera ainda.')
            return

        msg = self.last_image
        if msg.encoding.lower() not in ('rgb8', 'bgr8'):
            self.get_logger().warn(f'Encoding não suportado: {msg.encoding}')
            return

        img_np = np.frombuffer(msg.data, dtype=np.uint8)
        expected = msg.width * msg.height * 3
        if img_np.size != expected:
            self.get_logger().warn(
                f'Tamanho de imagem inesperado: {img_np.size} vs {expected}'
            )
            return
        img_np = img_np.reshape((msg.height, msg.width, 3))
        if msg.encoding.lower() == 'rgb8':
            img_rgb = img_np
        else:
            img_rgb = img_np[:, :, ::-1]

        try:
            results = self.model.predict(img_rgb, verbose=False)
        except Exception as exc:
            self.get_logger().error(f'Erro na inferência YOLO: {exc}')
            return

        if not results or len(results) == 0:
            self.get_logger().info('Nenhum resultado retornado pelo YOLO.')
            return

        det = results[0]
        if det.boxes is None or len(det.boxes) == 0:
            self.get_logger().info('Nenhum objeto detectado.')
            return

        annotated = det.plot()  # array BGR
        saved = self.save_image('yolo_result.jpg', annotated)
        if saved:
            self.get_logger().info(
                f'Detecção concluída e salva em yolo_result.jpg ({len(det.boxes)} objetos).'
            )

        # Estima posições se depth + TF disponíveis
        if self.last_depth is None:
            self.get_logger().warn('Sem depth recente para estimar posições.')
            return

        depth_msg, depth_arr = self.last_depth
        if depth_msg.width != msg.width or depth_msg.height != msg.height:
            self.get_logger().warn(
                'Depth e RGB com tamanhos diferentes; pulando estimativa de posição.'
            )
            return

        # Usa intrínsecos; se não fornecidos, assume centro da imagem
        cx = self.cx if self.cx > 0 else (msg.width - 1) * 0.5
        cy = self.cy if self.cy > 0 else (msg.height - 1) * 0.5
        if self.fx > 0 and self.fy > 0:
            fx = self.fx
            fy = self.fy
        else:
            # calcula a partir do FOV e resolução (assumindo lente simétrica)
            fov_rad = math.radians(self.fov_deg if self.fov_deg > 0 else 57.0)
            fx = float(msg.width) / (2.0 * math.tan(fov_rad * 0.5))
            fy = fx

        try:
            tf_map_cam = self.tf_buffer.lookup_transform(
                'map',
                msg.header.frame_id if msg.header.frame_id else 'camera_link',
                rclpy.time.Time(),
            )
        except TransformException as exc:
            self.get_logger().warn(f'Nao consegui TF camera -> map: {exc}')
            return

        tf_map_base = None
        try:
            tf_map_base = self.tf_buffer.lookup_transform(
                'map',
                self.base_frame,
                rclpy.time.Time(),
            )
        except TransformException:
            pass

        boxes = det.boxes.xyxy.cpu().numpy()
        labels_idx = det.boxes.cls.cpu().numpy().astype(int)
        names = det.names if hasattr(det, 'names') else self.model.names

        markers = []
        if self.last_marker_ids:
            clear_arr = MarkerArray()
            for ns, mid in self.last_marker_ids:
                m = Marker()
                m.header.frame_id = 'map'
                m.ns = ns
                m.id = mid
                m.action = Marker.DELETE
                clear_arr.markers.append(m)
            if clear_arr.markers:
                self.marker_pub.publish(clear_arr)
            self.last_marker_ids = []
        header_frame = 'map'
        stamp = msg.header.stamp
        det_infos = []
        base_xy = None
        if tf_map_base:
            base_xy = (
                tf_map_base.transform.translation.x,
                tf_map_base.transform.translation.y,
            )

        for i, box in enumerate(boxes):
            x1, y1, x2, y2 = box
            u = int((x1 + x2) * 0.5)
            v = int((y1 + y2) * 0.5)
            # clamp
            u = max(0, min(u, depth_msg.width - 1))
            v = max(0, min(v, depth_msg.height - 1))

            z = float(depth_arr[v, u])
            if z <= 0.0 or np.isnan(z) or np.isinf(z):
                self.get_logger().info(
                    f'Deteccao {i}: depth inválido em ({u},{v}) -> z={z:.3f}'
                )
                continue

            # Direção do raio no frame da câmera (pinhole)
            nx = (u - cx) / fx
            ny = (v - cy) / fy
            dir_cam = np.array([nx, ny, 1.0], dtype=float)
            norm = np.linalg.norm(dir_cam)
            if norm <= np.finfo(float).eps:
                self.get_logger().info(
                    f'Deteccao {i}: direcao de raio degenerada em ({u},{v})'
                )
                continue
            dir_cam /= norm

            # Usa z como alcance ao longo do raio (distância câmera->ponto)
            point_cam_3d = dir_cam * z

            # Constrói PointStamped no frame da câmera
            p_cam = PointStamped()
            p_cam.header.frame_id = msg.header.frame_id if msg.header.frame_id else 'camera_link'
            p_cam.header.stamp = msg.header.stamp
            p_cam.point.x = float(point_cam_3d[0])
            p_cam.point.y = float(point_cam_3d[1])
            p_cam.point.z = float(point_cam_3d[2])

            try:
                p_map = do_transform_point(p_cam, tf_map_cam)
            except Exception as exc:
                self.get_logger().warn(f'Falha ao transformar ponto camera->map: {exc}')
                continue

            x_map = p_map.point.x
            y_map = p_map.point.y
            z_map = p_map.point.z

            det_id = self.det_counter
            self.det_counter += 1
            label = names[labels_idx[i]] if names and labels_idx[i] in names else str(
                labels_idx[i]
            )
            robot_dist = None
            if base_xy is not None:
                dx = x_map - base_xy[0]
                dy = y_map - base_xy[1]
                robot_dist = math.hypot(dx, dy)
            self.get_logger().info(
                f'Deteccao {det_id}: {label} em map (x={x_map:.2f}, y={y_map:.2f}) '
                f'pixel=({u},{v}) depth={z:.2f}'
                + (f' dist_robo={robot_dist:.2f}' if robot_dist is not None else '')
            )

            # Marker (cubo) e texto
            m = Marker()
            m.header.frame_id = header_frame
            m.header.stamp = stamp
            m.ns = 'yolo'
            m.id = det_id
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = float(x_map)
            m.pose.position.y = float(y_map)
            m.pose.position.z = 0.1
            m.pose.orientation.w = 1.0
            m.scale.x = 0.1
            m.scale.y = 0.1
            m.scale.z = 0.1
            m.color.r = 1.0
            m.color.g = 0.5
            m.color.b = 0.0
            m.color.a = 0.8
            markers.append(m)
            self.last_marker_ids.append((m.ns, m.id))

            t = Marker()
            t.header.frame_id = header_frame
            t.header.stamp = stamp
            t.ns = 'yolo_text'
            t.id = det_id + 100000  # offset
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x = float(x_map)
            t.pose.position.y = float(y_map)
            t.pose.position.z = 0.25
            t.pose.orientation.w = 1.0
            t.scale.z = 0.15
            t.color.r = 1.0
            t.color.g = 1.0
            t.color.b = 1.0
            t.color.a = 0.9
            t.text = f'{label} ({det_id})'
            markers.append(t)
            self.last_marker_ids.append((t.ns, t.id))

            det_infos.append(
                {
                    'id': det_id,
                    'label': label,
                    'pixel': (u, v),
                    'depth': z,
                    'robot_dist': robot_dist,
                }
            )

        if markers:
            ma = MarkerArray()
            ma.markers = markers
            self.marker_pub.publish(ma)

        # Salva imagem anotada com pixel central e info
        if det_infos and cv2 is not None and annotated is not None:
            ann = annotated.copy()
            for info in det_infos:
                u, v = info['pixel']
                cv2.circle(ann, (u, v), 4, (0, 0, 255), -1)
                txt = f"{info['label']} ({info['id']}) z={info['depth']:.2f}"
                if info['robot_dist'] is not None:
                    txt += f" d={info['robot_dist']:.2f}"
                cv2.putText(
                    ann,
                    txt,
                    (u + 5, max(15, v - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.4,
                    (255, 255, 255),
                    1,
                    cv2.LINE_AA,
                )
            if self.save_image('yolo_debug.jpg', ann):
                self.get_logger().info(
                    'Imagem anotada salva em yolo_debug.jpg com centros de bbox e distancias.'
                )

    @staticmethod
    def save_image(path: str, img_bgr: np.ndarray) -> bool:
        if cv2 is not None:
            try:
                cv2.imwrite(path, img_bgr)
                return True
            except Exception:
                pass
        try:
            import imageio.v2 as iio  # type: ignore

            iio.imwrite(path, img_bgr[:, :, ::-1])  # imageio espera RGB
            return True
        except Exception:
            return False


def main(args=None):
    rclpy.init(args=args)
    node = YoloKeyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.restore_terminal()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
