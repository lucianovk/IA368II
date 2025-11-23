#!/usr/bin/env python3
"""
Gera um relatório em PDF com base no conteúdo da pasta detections.
O script não depende de NumPy/Matplotlib; usa apenas ReportLab.
"""

import argparse
import json
from pathlib import Path
from typing import Any, Dict, List

from reportlab.lib.pagesizes import A4
from reportlab.lib.units import cm
from reportlab.pdfgen import canvas
from reportlab.lib.utils import ImageReader

PAGE_WIDTH, PAGE_HEIGHT = A4
MARGIN = 2 * cm
LINE_HEIGHT = 0.6 * cm
IMAGE_MAX_WIDTH = (PAGE_WIDTH - 3 * MARGIN) / 2
IMAGE_MAX_HEIGHT = 8 * cm


def resolve_package_root(current_file: Path) -> Path:
    for ancestor in current_file.resolve().parents:
        direct = ancestor / 'src' / 'my_robot_pkg'
        if direct.exists():
            return direct
        workspace = ancestor / 'final_project_ws' / 'src' / 'my_robot_pkg'
        if workspace.exists():
            return workspace
    raise FileNotFoundError('Não foi possível localizar src/my_robot_pkg no workspace.')


def load_detections(detections_file: Path) -> List[Dict[str, Any]]:
    if not detections_file.exists():
        return []
    with detections_file.open('r', encoding='utf-8') as fp:
        raw = json.load(fp)
    detections: List[Dict[str, Any]] = []
    for entry in raw:
        detections.append(
            {
                'id': entry.get('id'),
                'label': entry.get('label', 'unknown'),
                'position': entry.get('position', {}),
                'confidence': float(entry.get('confidence', 0.0)),
                'replacements': int(entry.get('replacements', 0)),
            }
        )
    detections.sort(key=lambda det: det.get('id', 0))
    return detections


def build_summary_page(pdf: canvas.Canvas, detections: List[Dict[str, Any]]) -> None:
    pdf.setFont('Helvetica-Bold', 18)
    pdf.drawString(MARGIN, PAGE_HEIGHT - MARGIN, 'Resumo das Detecções')
    pdf.setFont('Helvetica', 12)
    y = PAGE_HEIGHT - MARGIN - LINE_HEIGHT * 2
    pdf.drawString(MARGIN, y, f'Total de detecções: {len(detections)}')
    y -= LINE_HEIGHT * 1.5
    if detections:
        pdf.setFont('Helvetica-Bold', 12)
        pdf.drawString(MARGIN, y, 'Distribuição por label:')
        y -= LINE_HEIGHT
        pdf.setFont('Helvetica', 12)
        counts: Dict[str, int] = {}
        for det in detections:
            counts[det['label']] = counts.get(det['label'], 0) + 1
        for label, count in sorted(counts.items(), key=lambda item: item[0]):
            pdf.drawString(MARGIN + 10, y, f'- {label}: {count}')
            y -= LINE_HEIGHT
            if y < MARGIN:
                pdf.showPage()
                y = PAGE_HEIGHT - MARGIN
    else:
        pdf.drawString(MARGIN, y, 'Nenhuma detecção registrada.')
    pdf.showPage()


def draw_detection_page(pdf: canvas.Canvas, detection: Dict[str, Any], detections_dir: Path) -> None:
    pdf.setFont('Helvetica-Bold', 16)
    header = f"Detecção #{detection.get('id', 'N/A')} - {detection.get('label', 'unknown')}"
    pdf.drawString(MARGIN, PAGE_HEIGHT - MARGIN, header)
    pdf.setFont('Helvetica', 12)
    pos = detection.get('position', {})
    info_lines = [
        f"Confiança: {detection.get('confidence', 0.0):.2f}",
        f"Posição (map): x={pos.get('x', 0.0):.2f}, y={pos.get('y', 0.0):.2f}",
        f"Substituições registradas: {detection.get('replacements', 0)}",
    ]
    y = PAGE_HEIGHT - MARGIN - LINE_HEIGHT * 1.5
    for line in info_lines:
        pdf.drawString(MARGIN, y, line)
        y -= LINE_HEIGHT

    input_path = detections_dir / f"input_{detection.get('id')}.jpg"
    detection_path = detections_dir / f"detection_{detection.get('id')}.jpg"
    y_image = y - LINE_HEIGHT
    draw_image(pdf, input_path, MARGIN, y_image, 'Imagem Original')
    draw_image(pdf, detection_path, MARGIN + IMAGE_MAX_WIDTH + MARGIN, y_image, 'Detecção Anotada')
    pdf.showPage()


def draw_image(pdf: canvas.Canvas, image_path: Path, x: float, y_top: float, caption: str) -> None:
    pdf.setFont('Helvetica-Bold', 12)
    pdf.drawString(x, y_top, caption)
    if not image_path.exists():
        pdf.setFont('Helvetica-Oblique', 11)
        pdf.drawString(x, y_top - LINE_HEIGHT, 'Imagem não encontrada.')
        return
    try:
        img = ImageReader(str(image_path))
        iw, ih = img.getSize()
        scale = min(IMAGE_MAX_WIDTH / iw, IMAGE_MAX_HEIGHT / ih)
        width = iw * scale
        height = ih * scale
        y_img = y_top - LINE_HEIGHT - height
        pdf.drawImage(img, x, y_img, width=width, height=height, preserveAspectRatio=True)
    except Exception as exc:
        pdf.setFont('Helvetica-Oblique', 11)
        pdf.drawString(x, y_top - LINE_HEIGHT, f'Erro ao carregar imagem: {exc}')


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Gera PDF com o resultado das detecções.')
    parser.add_argument(
        '--output',
        '-o',
        type=Path,
        help='Arquivo PDF de saída (default: detections/detections_report.pdf)',
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    package_root = resolve_package_root(Path(__file__))
    detections_dir = package_root / 'detections'
    detections_dir.mkdir(parents=True, exist_ok=True)
    detections_file = detections_dir / 'detections.json'
    detections = load_detections(detections_file)

    output_path = args.output
    if output_path is None:
        output_path = detections_dir / 'detections_report.pdf'
    else:
        output_path = output_path.resolve()
        output_path.parent.mkdir(parents=True, exist_ok=True)

    pdf = canvas.Canvas(str(output_path), pagesize=A4)
    build_summary_page(pdf, detections)
    if not detections:
        pdf.setFont('Helvetica', 14)
        pdf.drawString(MARGIN, PAGE_HEIGHT - MARGIN, 'Nenhuma detecção disponível.')
        pdf.showPage()
    else:
        for detection in detections:
            draw_detection_page(pdf, detection, detections_dir)
    pdf.save()
    print(f'Relatório gerado em {output_path}')


if __name__ == '__main__':
    main()
