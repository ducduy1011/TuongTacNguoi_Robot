#!/usr/bin/env python3
"""Download two real 3-D Gazebo models. This script never starts/spawns anything."""
import hashlib
import io
from pathlib import Path
import urllib.request
import xml.etree.ElementTree as ET
import zipfile


MODELS = [
    ('Chair', 'Chair', '5bcf90f8177efdd373cf8099209372e6c13179113320db0a24c6f5f573a11e33'),
    ('Standing%20person', 'Standing_person',
     '385c4c518662f75b6c719849a4f47100bac7617058407c7d6db41bf802fc5c5e'),
]


def main():
    root = Path(__file__).resolve().parents[1] / '.models' / 'scene'
    for remote, folder, digest in MODELS:
        target = root / folder
        if (target / '.prepared-v1').is_file():
            print(f'Already prepared: {target}')
            continue
        url = f'https://fuel.gazebosim.org/1.0/OpenRobotics/models/{remote}/3/{remote}.zip'
        print(f'Downloading {url}', flush=True)
        data = urllib.request.urlopen(url, timeout=90).read()
        if hashlib.sha256(data).hexdigest() != digest:
            raise ValueError(f'Archive checksum changed: {url}')
        target.mkdir(parents=True, exist_ok=True)
        with zipfile.ZipFile(io.BytesIO(data)) as archive:
            for member in archive.infolist():
                if not (target / member.filename).resolve().is_relative_to(target.resolve()):
                    raise ValueError('Unsafe path in model archive.')
            archive.extractall(target)
        sdf = ET.parse(target / 'model.sdf')
        model = sdf.getroot().find('model')
        static = ET.SubElement(model, 'static')
        static.text = 'true'
        # Absolute mesh URIs work even though the official launch resets resource paths.
        for uri in model.iter('uri'):
            if uri.text and uri.text.startswith('model://'):
                uri.text = (target / uri.text[len('model://'):].partition('/')[2]).as_uri()
        sdf.write(target / 'model.sdf', encoding='utf-8', xml_declaration=True)
        (target / '.prepared-v1').write_text(digest + '\n')
        print(f'Prepared: {target}')


if __name__ == '__main__':
    main()
