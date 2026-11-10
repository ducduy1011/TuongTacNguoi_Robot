"""Chi gui cau nghe ro; khong bien am thanh mo ho thanh dich robot."""
import pytest

from voice_navigation.speech import recognized_text


def test_clear_command():
    result = {'text': 'đi đến bàn', 'result': [
        {'word': 'đi', 'conf': 0.9}, {'word': 'đến', 'conf': 0.8},
        {'word': 'bàn', 'conf': 0.9}]}
    assert recognized_text(result, 0.6) == 'đi đến bàn'


@pytest.mark.parametrize('result', [
    {'text': 'đi đến bàn', 'result': [{'conf': 0.3}]},
    {'text': 'đi đến [unk]', 'result': [{'conf': 0.95}]},
    {'text': 'đi đến bàn'},
])
def test_unclear_command_is_rejected(result):
    with pytest.raises(ValueError, match='Chua nghe ro'):
        recognized_text(result, 0.6)


def test_silence():
    assert recognized_text({'text': ''}, 0.6) == ''
