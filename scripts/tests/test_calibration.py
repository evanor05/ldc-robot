import csv
import importlib.util
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
import numpy as np
import cv2
import yaml
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'scripts/calibration'))
from cli import initialize
from measurements import analyze,SCHEMAS
from camera import board_points,solve_views,camera_ray,make_board
from summarize import summarize


class CalibrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp=tempfile.TemporaryDirectory();cls.session=Path(cls.temp.name)/'session';initialize(cls.session)
    @classmethod
    def tearDownClass(cls):cls.temp.cleanup()
    def test_empty_templates_are_unmeasured_and_audit_flags_geometry(self):
        result=analyze(self.session);self.assertEqual(result['status'],'awaiting_measurements');self.assertEqual(result['candidates'],{})
        baseline=json.loads((self.session/'baseline.json').read_text())
        self.assertEqual(baseline['firmware']['track_width_mm'],131.7)
        self.assertAlmostEqual(baseline['urdf']['wheel_center_separation_m'],.162)
        self.assertTrue(any('圆半径' in w for w in baseline['warnings']))
        with self.assertRaises(FileExistsError):initialize(self.session)
    def test_scale_and_track_corrections_have_correct_direction(self):
        with tempfile.TemporaryDirectory() as temp:
            import shutil
            folder=Path(temp)/'session';shutil.copytree(self.session,folder)
            for kind,rows in [('distance',[['a','forward',2,1.9,0,'true',''],['b','reverse',-2,-1.9,0,'true','']]),
                              ('turn',[['a','ccw',360,400,'wheel_only','true',''],['b','cw',-360,-400,'wheel_only','true','']])]:
                with (folder/(kind+'.csv')).open('w',newline='') as h:
                    writer=csv.writer(h);writer.writerow(SCHEMAS[kind]);writer.writerows(rows)
            result=analyze(folder);self.assertAlmostEqual(result['candidates']['common_wheel_diameter_mm'],65*2/1.9)
            self.assertAlmostEqual(result['candidates']['effective_track_width_mm'],131.7*(2/1.9)/(360/400))
            self.assertEqual(result['measurements']['turn']['quality'],'provisional')
            p=folder/'turn.csv';p.write_text(p.read_text().replace('wheel_only','ekf'))
            self.assertNotIn('effective_track_width_mm',analyze(folder)['candidates'])
    def test_wrong_direction_nan_and_duplicate_ids_are_not_fitted(self):
        from measurements import scale_fit,number,rows_from
        with self.assertRaises(ValueError):scale_fit([2],[-2])
        with self.assertRaises(ValueError):number({'value':'nan'},'value')
        with tempfile.NamedTemporaryFile('w',suffix='.csv') as h:
            writer=csv.writer(h);writer.writerow(SCHEMAS['distance']);writer.writerows([['same','forward',2,2,0,'true','']]*2);h.flush()
            with self.assertRaises(ValueError):rows_from(Path(h.name),'distance')
    def test_checkerboard_dimensions_and_no_overwrite(self):
        with tempfile.TemporaryDirectory() as temp:
            p=Path(temp)/'board.svg';make_board(p)
            text=p.read_text();self.assertIn('width="297mm"',text);self.assertIn('square 20 mm',text)
            self.assertAlmostEqual(float(board_points(9,6,20)[-1,0]),.16)
            with self.assertRaises(FileExistsError):make_board(p)
    def test_known_synthetic_intrinsics_recovered_with_holdout(self):
        obj=board_points(9,6,20);k=np.array([[600.,0,400],[0,610,300],[0,0,1]])
        objects=[];images=[];rng=np.random.default_rng(7)
        for i in range(30):
            rvec=np.array([((i%5)-2)*.15,((i%7)-3)*.12,.02*(i%4)])
            z=.7+.03*(i%3);cx=[140,400,660][i%3];cy=[125,300,475][(i//3)%3]
            tvec=np.array([(cx-400)*z/600-.08,(cy-300)*z/610-.05,z])
            points,_=cv2.projectPoints(obj,rvec,tvec,k,np.zeros(5));points+=rng.normal(0,.05,points.shape)
            objects.append(obj.copy());images.append(points.astype(np.float32))
        fitted,dist,report=solve_views(objects,images,(800,600))
        self.assertAlmostEqual(fitted[0,0],600,delta=5);self.assertAlmostEqual(fitted[1,1],610,delta=5)
        self.assertEqual(report['holdout_views'],6);self.assertLess(max(report['holdout_rms_px']),.5)
    def test_pixel_ray_reports_direction_without_invented_distance(self):
        with tempfile.NamedTemporaryFile('w') as h:
            yaml.safe_dump(dict(image_width=640,image_height=480,distortion_model='plumb_bob',camera_matrix=dict(data=[100,0,320,0,100,240,0,0,1]),distortion_coefficients=dict(data=[0]*5)),h);h.flush()
            result=camera_ray(h.name,420,240,640,480);self.assertAlmostEqual(result['bearing_right_deg'],45);self.assertIsNone(result['distance_m'])
            with self.assertRaises(ValueError):camera_ray(h.name,420,240,800,600)
    def test_yaw_unwrap_and_timestamp_reset_detection(self):
        with tempfile.NamedTemporaryFile('w+') as h:
            for i in range(401):
                angle=i*2*np.pi/400
                msg=dict(header=dict(frame_id='odom',stamp=dict(sec=1+i//20,nanosec=(i%20)*50_000_000)),child_frame_id='base_link',
                         pose=dict(pose=dict(position=dict(x=0,y=0,z=0),orientation=dict(x=0,y=0,z=float(np.sin(angle/2)),w=float(np.cos(angle/2))))))
                h.write(json.dumps(dict(topic='/odom',data=msg))+'\n')
            h.flush();result=summarize(h.name);self.assertAlmostEqual(result['odom']['unwrapped_yaw_deg'],360)
            h.write(json.dumps(dict(topic='/odom',data=msg))+'\n');h.flush()
            with self.assertRaises(ValueError):summarize(h.name)

    def test_wall_fit_rejects_zero_increment_and_recovers_known_plane(self):
        with tempfile.NamedTemporaryFile('w+') as handle:
            scan=dict(angle_min=-.3,angle_increment=0.,range_min=.1,range_max=10.,ranges=[2.]*31)
            handle.write(json.dumps(dict(topic='/scan',data=scan))+'\n')
            handle.flush();self.assertIsNone(summarize(handle.name)['front_wall'])
            scan['angle_increment']=.02
            scan['ranges']=[2/math.cos(-.3+i*.02) for i in range(31)]
            handle.write(json.dumps(dict(topic='/scan',data=scan))+'\n');handle.flush()
            wall=summarize(handle.name)['front_wall'];self.assertEqual(wall['samples'],1);self.assertAlmostEqual(wall['distance_m'],2)
