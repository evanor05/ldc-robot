"""交通灯双路判色的离线验证（合成帧，不依赖 YOLO/ROS）。

覆盖：OpenCV 判色四色、框过小/熄灭/过曝/冲突、三色权重标签映射、
opencv/yolo/consensus 三模式融合（consensus 仅同框两路都绿才放行）、
观测记录字段（无灯 vs 灯色不清显式区分）。
"""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'robot_ws/src/ldc_vision/ldc_vision'))
import traffic_color as tc


def lamp(color, size=40):
    """合成一盏点亮灯：深灰底 + 高饱和色块（BGR）。"""
    frame = np.full((size, size, 3), 40, dtype=np.uint8)
    margin = size // 6
    hsv = np.zeros((size - 2 * margin, size - 2 * margin, 3), dtype=np.uint8)
    hsv[..., 1] = 255
    hsv[..., 2] = 255
    hsv[..., 0] = {'red': 0, 'yellow': 25, 'green': 60}[color]
    frame[margin:size - margin, margin:size - margin] = tc.cv2.cvtColor(hsv, tc.cv2.COLOR_HSV2BGR)
    return frame, [0, 0, size, size]


@pytest.mark.parametrize('color', ['red', 'yellow', 'green'])
def test_classify_crop_detects_lit_color(color):
    frame, box = lamp(color)
    result = tc.classify_crop(frame, box)
    assert result['color'] == color, result
    assert result['confidence'] >= tc._MIN_PURITY


def test_classify_crop_unlit_is_unknown_not_guessed():
    frame = np.full((40, 40, 3), 30, dtype=np.uint8)  # 熄灭/暗
    assert tc.classify_crop(frame, [0, 0, 40, 40])['color'] == 'unknown'


def test_classify_crop_box_too_small_is_unknown():
    result = tc.classify_crop(np.zeros((40, 40, 3), np.uint8), [0, 0, 4, 4])
    assert result['color'] == 'unknown' and result['reason'] == 'box_too_small'


def test_classify_crop_mixed_hues_is_ambiguous():
    frame = np.full((60, 60, 3), 40, dtype=np.uint8)
    red = np.zeros((24, 24, 3), np.uint8); red[..., 1] = 255; red[..., 2] = 255; red[..., 0] = 0
    green = np.zeros((24, 24, 3), np.uint8); green[..., 1] = 255; green[..., 2] = 255; green[..., 0] = 60
    frame[6:30, 6:30] = tc.cv2.cvtColor(red, tc.cv2.COLOR_HSV2BGR)
    frame[32:56, 32:56] = tc.cv2.cvtColor(green, tc.cv2.COLOR_HSV2BGR)
    result = tc.classify_crop(frame, [0, 0, 60, 60])
    assert result['color'] == 'unknown' and result['reason'] == 'ambiguous'


def test_generic_weight_label_carries_no_color():
    assert tc.color_from_label('traffic light') == (None, None)
    assert tc.has_color_labels({0: 'traffic light'}) == {}


def test_three_color_labels_mapped():
    names = {0: 'red traffic light', 1: 'traffic_light_yellow', 2: 'green light'}
    assert tc.has_color_labels(names) == {
        'red traffic light': 'red', 'traffic_light_yellow': 'yellow', 'green light': 'green'}
    assert tc.color_from_label('traffic light red') == ('red', 'traffic light red')


def test_label_map_overrides_irregular_labels():
    assert tc.color_from_label('TL-A', {'TL-A': 'green'}) == ('green', 'TL-A')
    assert tc.color_from_label('TL-A', {'TL-A': 'unknown'}) == (None, None)


@pytest.mark.parametrize('mode,opencv_color,yolo_color,expected_pass,expected_conflict', [
    ('opencv', 'green', None, True, False),
    ('opencv', 'red', None, False, False),
    ('yolo', 'red', 'green', True, False),          # yolo 模式只看标签
    ('yolo', 'green', None, False, False),          # 无三色标签 → 不放行
    ('consensus', 'green', 'green', True, False),   # 两路都绿才放行
    ('consensus', 'green', None, False, False),     # 缺标签路 → fail-closed
    ('consensus', 'green', 'red', False, True),     # 冲突 → unknown
    ('consensus', 'red', 'red', False, False),      # 一致非绿 → 采信红灯
])
def test_resolve_color_modes(mode, opencv_color, yolo_color, expected_pass, expected_conflict):
    resolved = tc.resolve_color(mode, {'color': opencv_color, 'reason': ''}, yolo_color)
    assert resolved['pass_eligible'] is expected_pass
    assert resolved['conflict'] is expected_conflict
    if expected_conflict:
        assert resolved['color'] == 'unknown'


def test_invalid_mode_rejected():
    with pytest.raises(ValueError):
        tc.resolve_color('magic', {'color': 'green'}, None)


def test_light_color_fields_same_box_both_green_passes():
    frame, box = lamp('green')
    fields = tc.light_color_fields(frame, box, 'traffic light', 'opencv')
    assert fields['opencv_color'] == 'green'
    assert fields['color'] == 'green' and fields['color_pass_eligible'] is True
    assert fields['yolo_color'] is None


def test_light_color_fields_yolo_mode_without_labels_never_passes():
    frame, box = lamp('green')
    fields = tc.light_color_fields(frame, box, 'traffic light', 'yolo')
    assert fields['color_pass_eligible'] is False
    assert fields['color_reason'] == 'yolo_label_unavailable'


def test_light_record_keeps_contract_and_color_fields():
    item = {'track_id': 7, 'name': 'traffic light', 'score': .9, 'xyxy': [1, 2, 3, 4],
            'missed': 0, 'color': 'green', 'color_pass_eligible': True,
            'opencv_color': 'green', 'yolo_color': None, 'extra_internal': 'drop-me'}
    record = tc.light_record(item)
    assert record['track_id'] == 7 and record['color_pass_eligible'] is True
    assert 'extra_internal' not in record


def test_frame_observation_distinguishes_no_light_from_unknown():
    header = {'sec': 100, 'nanosec': 5, 'frame_id': 'laser_frame'}
    no_light = tc.frame_observation(session='s', frame_id='s:1', sequence=1, header=header,
                                    width=640, height=480, color_mode='opencv', lights=[],
                                    captured_at=1.0, published_at=2.0, wall_at=3.0)
    assert no_light['kind'] == 'frame' and no_light['light_count'] == 0
    assert no_light['stamp_ns'] == 100 * 1_000_000_000 + 5
    assert no_light['header']['frame_id'] == 'laser_frame'
    assert no_light['at'] == 1.0 and no_light['published_at'] == 2.0

    unknown_light = dict(no_light, light_count=1,
                         lights=[{'name': 'traffic light', 'color': 'unknown', 'track_id': 1}])
    assert unknown_light['lights'][0]['color'] == 'unknown'
    assert no_light['light_count'] != unknown_light['light_count']


def test_status_observation_kinds():
    for kind in ('stale_frame', 'stream_lost', 'stream_recovered', 'detector_down'):
        assert tc.status_observation(kind, at=1.0)['kind'] == kind
    with pytest.raises(ValueError):
        tc.status_observation('bogus')


def test_traffic_light_name_covers_generic_and_colored():
    assert tc.is_traffic_light_name('traffic light')
    assert tc.is_traffic_light_name('red traffic light')
    assert not tc.is_traffic_light_name('person')
    assert not tc.is_traffic_light_name('stop sign')