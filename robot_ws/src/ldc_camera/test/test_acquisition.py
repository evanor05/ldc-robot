"""Package-local regressions, discoverable by colcon's unittest runner."""
import tempfile
import unittest
from ldc_camera.mjpeg import MjpegParser
from ldc_camera.calibration import Calibration
from std_msgs.msg import Header


class AcquisitionTests(unittest.TestCase):
    def test_fragmented_jpeg_is_preserved_exactly(self):
        jpeg = b'\xff\xd8original bytes\xff\xd9'
        data = b'--f\r\nContent-Type: image/jpeg\r\nContent-Length: ' + str(len(jpeg)).encode() + b'\r\n\r\n' + jpeg + b'\r\n'
        parser = MjpegParser('multipart/x-mixed-replace;boundary=f')
        frames = []
        for byte in data:
            frames.extend(parser.feed(bytes([byte])))
        parser.finish()
        self.assertEqual(frames, [jpeg])

    def test_partial_jpeg_does_not_turn_into_a_frame(self):
        parser = MjpegParser('boundary=f')
        self.assertEqual(parser.feed(b'--f\r\nContent-Type: image/jpeg\r\nContent-Length: 100\r\n\r\n\xff\xd8'), [])
        with self.assertRaises(ValueError):
            parser.finish()

    def test_calibration_does_not_invent_intrinsics_or_wrong_dimensions(self):
        with tempfile.NamedTemporaryFile('w') as handle:
            handle.write('image_width: 640\nimage_height: 480\n')
            handle.flush()
            header = Header(frame_id='camera_optical_frame')
            msg, detail = Calibration(handle.name).message(header, 800, 600)
            self.assertEqual((msg.width, msg.height), (800, 600))
            self.assertEqual(msg.header, header)
            self.assertEqual(msg.k[0], 0)
            self.assertIn('尺寸', detail)
