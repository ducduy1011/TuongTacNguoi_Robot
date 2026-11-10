"""Phan vung thu cong + OpenCV chon diem dich tren o trong."""
import argparse
import math
from pathlib import Path

import cv2
import numpy as np
import yaml


def pixel_to_world(u, v, height, resolution, origin):
    # Tam o; anh co truc y nguoc voi truc y cua ban do ROS.
    x = (u + 0.5) * resolution
    y = (height - v - 0.5) * resolution
    ox, oy, yaw = origin
    return (ox + x * math.cos(yaw) - y * math.sin(yaw),
            oy + x * math.sin(yaw) + y * math.cos(yaw))


def segment(map_file, zones_file, output):
    map_file, output = Path(map_file).resolve(), Path(output)
    meta = yaml.safe_load(map_file.read_text())
    if meta.get('mode', 'trinary') != 'trinary':
        raise ValueError('Chi ho tro ban do mode: trinary.')
    resolution = float(meta['resolution'])
    if resolution <= 0:
        raise ValueError('resolution phai duong.')
    image = cv2.imread(str(map_file.parent / meta['image']), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise ValueError('Khong doc duoc anh ban do.')
    cfg = yaml.safe_load(Path(zones_file).read_text())
    if len(cfg['regions']) < 3:
        raise ValueError('Can it nhat 3 khu vuc.')
    margin = float(cfg.get('clearance_m', 0.35))
    if margin <= 0:
        raise ValueError('clearance_m phai duong.')
    occupancy = image.astype(np.float64) / 255.0
    if not meta.get('negate', 0):
        occupancy = 1.0 - occupancy
    # Unknown va occupied deu khong duoc chon lam dich.
    free = (occupancy < float(meta['free_thresh'])).astype(np.uint8)
    # Them vien vat can de khong chon diem sat bien anh.
    padded = np.pad(free, 1)
    distance = cv2.distanceTransform(padded, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)[1:-1, 1:-1]
    clearance = distance * resolution
    labels = np.zeros(image.shape, np.uint16)
    preview = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    colors = [(80, 180, 250), (100, 220, 100), (240, 160, 100)]
    regions, seen = [], set()
    for index, region in enumerate(cfg['regions'], 1):
        if region['id'] in seen:
            raise ValueError('ID khu vuc bi trung.')
        seen.add(region['id'])
        polygon = np.asarray(region['polygon'], dtype=np.int32)
        h, w = image.shape
        if (polygon.ndim != 2 or polygon.shape[1] != 2 or len(polygon) < 3
                or np.any(polygon < 0) or np.any(polygon[:, 0] >= w)
                or np.any(polygon[:, 1] >= h)):
            raise ValueError(f"Da giac {region['id']} khong hop le.")
        mask = np.zeros_like(free)
        cv2.fillPoly(mask, [polygon], 1)
        area = (mask != 0) & (free != 0)
        if np.any(area & (labels != 0)):
            raise ValueError('Cac vung bi chong lan tren o trong.')
        region_distance = cv2.distanceTransform(
            np.pad(mask, 1), cv2.DIST_L2, cv2.DIST_MASK_PRECISE)[1:-1, 1:-1]
        # Dich nam sau trong vung, tranh dung sat ranh gioi B/C.
        inside = region_distance * resolution >= margin + resolution
        valid = area & (clearance >= margin) & inside
        if not np.any(valid):
            raise ValueError(f"Vung {region['id']} khong co diem du khoang trong.")
        v, u = np.unravel_index(np.argmax(np.where(valid, clearance, -1)), image.shape)
        x, y = pixel_to_world(u, v, h, resolution, meta['origin'])
        labels[area] = index
        color = np.array(colors[(index - 1) % len(colors)])
        preview[area] = (0.55 * preview[area] + 0.45 * color).astype(np.uint8)
        cv2.circle(preview, (int(u), int(v)), 9, (0, 0, 255), -1)
        cv2.putText(preview, region['id'], (int(u) + 14, int(v)),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 180), 3)
        regions.append({
            'id': region['id'], 'name': region['name'],
            'aliases': region.get('aliases', []),
            'pixel': [int(u), int(v)], 'clearance_m': round(float(clearance[v, u]), 3),
            'goal': {'x': float(x), 'y': float(y), 'yaw': 0.0},
        })
    # Diem den bo sung nam trong vung da chia; van kiem tra o trong.
    for point in cfg.get('waypoints', []):
        if point['id'] in seen:
            raise ValueError('ID diem den bi trung.')
        seen.add(point['id'])
        u, v = point['pixel']
        if (not isinstance(u, int) or not isinstance(v, int)
                or not 0 <= u < w or not 0 <= v < h
                or not free[v, u] or clearance[v, u] < margin):
            raise ValueError(f"Diem {point['id']} khong du khoang trong.")
        x, y = pixel_to_world(u, v, h, resolution, meta['origin'])
        regions.append({
            'id': point['id'], 'name': point['name'],
            'aliases': point.get('aliases', []), 'pixel': [u, v],
            'clearance_m': round(float(clearance[v, u]), 3),
            'goal': {'x': float(x), 'y': float(y),
                     'yaw': float(point.get('yaw', 0.0))},
        })
        cv2.circle(preview, (u, v), 9, (0, 0, 255), -1)
        cv2.putText(preview, point['id'], (u + 14, v),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 180), 3)
    output.mkdir(parents=True, exist_ok=True)
    (output / 'regions.yaml').write_text(yaml.safe_dump(
        {'map': str(map_file), 'frame_id': 'map', 'regions': regions},
        allow_unicode=True, sort_keys=False))
    cv2.imwrite(str(output / 'segmented_map.png'), preview)
    cv2.imwrite(str(output / 'labels.png'), labels)
    return regions


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', required=True)
    parser.add_argument('--zones', required=True)
    parser.add_argument('--output', default='output')
    args = parser.parse_args()
    try:
        for region in segment(args.map, args.zones, args.output):
            print(region['id'], region['name'], region['goal'])
    except (ValueError, KeyError, OSError) as exc:
        parser.exit(1, f'Loi: {exc}\n')


if __name__ == '__main__':
    main()
