"""Standard-library protocol guards: wrong-frame detections must never display."""
import os
import unittest
from ldc_vision.ipc import write_packet, read_packet
from ldc_vision.results import checked_detections


class ProtocolTests(unittest.TestCase):
    def test_header_identity_and_original_coordinates(self):
        header = dict(sec=1, nanosec=2, frame_id='camera_optical_frame')
        result = dict(id='1', header=header, width=800, height=600, coordinates='original_pixels',
                      detections=[dict(name='stop sign', score=.8, xyxy=[700, 500, 790, 590])])
        self.assertEqual(checked_detections(result, '1', header, 800, 600)[0]['xyxy'], [700, 500, 790, 590])
        with self.assertRaises(ValueError):
            checked_detections(result, '2', header, 800, 600)
        with self.assertRaises(ValueError):
            checked_detections(result, '1', header, 640, 640)

    def test_ipc_roundtrip_and_exit_are_distinct(self):
        reader, writer = os.pipe()
        try:
            write_packet(writer, {'id': '1'}, b'jpeg')
            self.assertEqual(read_packet(reader, 1), ({'id': '1'}, b'jpeg'))
            os.close(writer)
            writer = None
            with self.assertRaises(EOFError):
                read_packet(reader, .1)
        finally:
            os.close(reader)
            if writer is not None:
                os.close(writer)
