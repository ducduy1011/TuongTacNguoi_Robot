from pathlib import Path
import math

import cv2
import numpy as np
import pytest
import yaml

from voice_navigation.commands import parse_command, speech_grammar
from voice_navigation.segment_map import pixel_to_world, segment

ROOT = Path(__file__).resolve().parents[1]
CONFIG = yaml.safe_load((ROOT / 'config/zones.yaml').read_text())
REGIONS = CONFIG['regions'] + CONFIG.get('waypoints', [])


@pytest.mark.parametrize('text, expected', [
    ('Đi đến khu vực A.', ['A']),
    ('Di chuyển từ khu vực A đến khu vực B.', ['A', 'B']),
    ('Đi đến khu lưu trữ.', ['C']),
    ('ĐI ĐẾN HÀNH LANG!', ['B']),
    ('di den khu vuc xe', ['C']),
    ('Đi từ A đến A.', ['A']),
    ('Đi đến gần bàn.', ['BAN']),
    ('Đi đến bàn.', ['BAN']),
    ('Di chuyển từ hành lang đến gần bàn.', ['B', 'BAN']),
])
def test_commands(text, expected):
    assert parse_command(text, REGIONS) == expected


@pytest.mark.parametrize('text', [
    'Đi đến khu vực D.', 'Di chuyển từ A đến D.',
    'Di chuyển từ D đến B.', 'Xin chào', 'Đi đến A rồi đến B',
])
def test_invalid_commands(text):
    with pytest.raises(ValueError):
        parse_command(text, REGIONS)


def test_speech_grammar():
    phrases = speech_grammar(REGIONS)
    assert '[unk]' in phrases
    assert 'đi đến khu vực dê' in phrases
    assert 'di chuyển từ khu vực a đến khu vực b' in phrases
    assert parse_command('đi đến khu lưu trữ', REGIONS) == ['C']
    with pytest.raises(ValueError, match='khong ton tai'):
        parse_command('đi đến khu vực dê', REGIONS)
    for phrase in phrases:
        if phrase in ['[unk]', 'đi đến khu vực d', 'đi đến khu vực dê']:
            continue
        assert parse_command(phrase, REGIONS)


def test_pixel_conversion():
    assert pixel_to_world(0, 0, 10, 0.1, [1, 2, 0]) == pytest.approx((1.05, 2.95))
    assert pixel_to_world(0, 0, 10, 0.1, [1, 2, math.pi / 2]) == pytest.approx((0.05, 2.05))


def test_real_map(tmp_path):
    regions = segment(ROOT / 'maps/warehouse.yaml', ROOT / 'config/zones.yaml', tmp_path)
    image = cv2.imread(str(ROOT / 'maps/warehouse.pgm'), 0)
    labels = cv2.imread(str(tmp_path / 'labels.png'), cv2.IMREAD_UNCHANGED)
    polygons = yaml.safe_load((ROOT / 'config/zones.yaml').read_text())['regions']
    assert set(np.unique(labels)) == {0, 1, 2, 3}
    for index, region in enumerate(regions, 1):
        u, v = region['pixel']
        assert image[v, u] > 229  # free_thresh = 0.1, negate = 0
        assert region['clearance_m'] >= 0.35
        if index <= len(polygons):
            assert labels[v, u] == index
            polygon = np.asarray(polygons[index - 1]['polygon'], dtype=np.int32)
            assert cv2.pointPolygonTest(polygon, (float(u), float(v)), True) * 0.03 >= 0.35
    assert regions[-1]['id'] == 'BAN'
    assert regions[-1]['goal']['yaw'] == pytest.approx(-math.pi / 2)


@pytest.mark.parametrize('negate', [0, 1])
def test_unknown_cells_and_negate(tmp_path, negate):
    image = np.full((40, 60), 255, np.uint8)
    image[:4, :] = 205  # unknown
    image[15:20, :] = 0  # occupied
    if negate:
        image = 255 - image
    cv2.imwrite(str(tmp_path / 'map.pgm'), image)
    meta = {'image': 'map.pgm', 'resolution': 0.1, 'origin': [0, 0, 0],
            'negate': negate, 'free_thresh': 0.1, 'occupied_thresh': 0.65}
    (tmp_path / 'map.yaml').write_text(yaml.safe_dump(meta))
    zones = {'regions': [
        {'id': name, 'name': name,
         'polygon': [[x, 0], [x + 19, 0], [x + 19, 39], [x, 39]]}
        for name, x in zip('ABC', [0, 20, 40])
    ]}
    (tmp_path / 'zones.yaml').write_text(yaml.safe_dump(zones))
    regions = segment(tmp_path / 'map.yaml', tmp_path / 'zones.yaml', tmp_path / 'out')
    for region in regions:
        u, v = region['pixel']
        assert v >= 4 and not 15 <= v < 20
    zones['clearance_m'] = 100
    (tmp_path / 'zones.yaml').write_text(yaml.safe_dump(zones))
    with pytest.raises(ValueError, match='khoang trong'):
        segment(tmp_path / 'map.yaml', tmp_path / 'zones.yaml', tmp_path / 'out')
