"""Chi chap nhan cac mau cau dieu huong cua bai thuc hanh."""
import re
import unicodedata


def speech_grammar(regions):
    """Uu tien tu vung dieu huong; van co nhanh cho loi noi ngoai danh sach."""
    # Bo cac ID/bi danh don am (a, b, xe, ban...) de giam nham lan.
    # Bo phan phan tich van chap nhan day du bi danh khi nhap van ban.
    names = sorted({str(name).lower() for region in regions
                    for name in [region['name'], *region.get('aliases', [])]
                    if len(str(name).split()) >= 2 or str(name).lower() == 'bàn'})
    phrases = ['[unk]', 'đi đến khu vực d', 'đi đến khu vực dê']
    for name in names:
        phrases.extend([f'đi đến {name}', f'di chuyển đến {name}'])
    for start in names:
        for end in names:
            phrases.extend([f'đi từ {start} đến {end}',
                            f'di chuyển từ {start} đến {end}'])
    return sorted(set(phrases))


def normalize(text):
    text = unicodedata.normalize('NFD', text.lower().replace('đ', 'd'))
    text = ''.join(c for c in text if unicodedata.category(c) != 'Mn')
    return ' '.join(re.sub(r'[^a-z0-9\s]', ' ', text).split())


def parse_command(text, regions):
    aliases = {}
    for region in regions:
        for name in [region['id'], region['name'], *region.get('aliases', [])]:
            key = normalize(name)
            if key in aliases and aliases[key] != region['id']:
                raise ValueError(f'Ten khu vuc bi trung: {name}')
            aliases[key] = region['id']
    text = normalize(text)
    route = re.fullmatch(r'(?:di chuyen|di) tu (.+?) den (.+)', text)
    direct = re.fullmatch(r'(?:di|di chuyen) den (.+)', text)
    names = route.groups() if route else (direct.groups() if direct else None)
    if names is None:
        raise ValueError('Mau lenh: Di den <vung> / Di chuyen tu <vung> den <vung>.')
    result = []
    for name in names:
        if name not in aliases:
            raise ValueError(f'Khu vuc khong ton tai: {name}')
        if not result or result[-1] != aliases[name]:
            result.append(aliases[name])
    return result
