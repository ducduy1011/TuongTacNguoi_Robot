"""Vosk tieng Viet offline: microphone -> /voice_command."""
import json
from pathlib import Path
import queue
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import yaml

from .commands import speech_grammar


def recognized_text(result, min_confidence):
    """Khong gui cau co tu khong ro hoac do tin cay trung binh thap."""
    text = result.get('text', '').strip()
    if not text:
        return ''
    words = result.get('result', [])
    confidence = sum(word.get('conf', 0.0) for word in words) / max(len(words), 1)
    if '[unk]' in text or confidence < min_confidence:
        raise ValueError('Chua nghe ro. Hay noi lai tron cau, gan micro hon.')
    return text


def main(args=None):
    rclpy.init(args=args)
    node = Node('speech_recognizer')
    node.declare_parameter('model_path', '')
    node.declare_parameter('device', -1)
    node.declare_parameter('regions_file', '')
    node.declare_parameter('sample_rate', 16000)
    node.declare_parameter('min_confidence', 0.6)
    audio = queue.Queue(maxsize=100)

    def callback(indata, frames, time_info, status):
        if status:
            node.get_logger().warning(str(status))
        try:
            audio.put_nowait(bytes(indata))
        except queue.Full:
            pass

    try:
        import sounddevice as sd
        from vosk import Model, KaldiRecognizer, SetLogLevel
        SetLogLevel(-1)
        model_path = Path(node.get_parameter('model_path').value).expanduser()
        if not (model_path / 'am').is_dir():
            raise ValueError('model_path phai tro den thu muc model Vosk da giai nen.')
        device = node.get_parameter('device').value
        device = None if device < 0 else device
        rate = int(node.get_parameter('sample_rate').value)
        sd.check_input_settings(device=device, channels=1, dtype='int16', samplerate=rate)
        model = Model(str(model_path))
        regions_file = node.get_parameter('regions_file').value
        if regions_file:
            regions = yaml.safe_load(Path(regions_file).read_text())['regions']
            recognizer = KaldiRecognizer(model, rate, json.dumps(
                speech_grammar(regions), ensure_ascii=False))
        else:
            recognizer = KaldiRecognizer(model, rate)
        recognizer.SetWords(True)
        min_confidence = float(node.get_parameter('min_confidence').value)
        if not 0 <= min_confidence <= 1:
            raise ValueError('min_confidence phai nam trong [0, 1].')
        node.get_logger().info(f'Micro: {sd.query_devices(device, "input")["name"]}; {rate} Hz')
        publisher = node.create_publisher(String, 'voice_command', 10)
        partial, last_partial = '', 0.0
        with sd.RawInputStream(samplerate=rate, blocksize=int(rate * 0.2),
                               device=device, dtype='int16', channels=1,
                               callback=callback):
            node.get_logger().info('Dang nghe tieng Viet. Noi lenh, roi ngung mot nhip.')
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0)
                try:
                    chunk = audio.get(timeout=0.2)
                except queue.Empty:
                    continue
                if recognizer.AcceptWaveform(chunk):
                    result = json.loads(recognizer.Result())
                    partial = ''
                    if result.get('text'):
                        node.get_logger().info('Nhan dang: ' + result['text'])
                    try:
                        text = recognized_text(result, min_confidence)
                    except ValueError as exc:
                        node.get_logger().warning(str(exc))
                        continue
                    if text:
                        publisher.publish(String(data=text))
                else:
                    text = json.loads(recognizer.PartialResult()).get('partial', '')
                    if text and text != partial and time.monotonic() - last_partial >= 0.5:
                        node.get_logger().info('Dang nhan: ' + text)
                        partial, last_partial = text, time.monotonic()
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        node.get_logger().error(f'Khong chay duoc nhan dang: {exc}')
        raise SystemExit(1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
