"""Recorded failures and absent evidence must never turn into healthy diagnostics."""
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
from scripts import diagnose_run as diag, log_sampler as sampler


class ReportTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.write('bridge-events', [{'at': 10., 'type': 'navigation_start'}])
        self.write('bridge-samples', [dict(kind='tick', at=10.+i,
            lidar=dict(ok=True, ageSeconds=.1, maxGap5s=.1),
            odometry=dict(ok=True, ageSeconds=.1, maxGap5s=.1)) for i in range(2)])
        self.write('net', [dict(at=10., backend=dict(reachable=True),
            ping=dict(ip='127.0.0.1', ok=True, rttMs=1.))])
        self.write('frontend-events', [{'at': 10., 'type': 'connected'}])
        (self.root/'console.log').write_text('[10.0] session established\n')

    def write(self, name, rows):
        (self.root/(name+'.jsonl')).write_text(''.join(json.dumps(r)+'\n' for r in rows))

    def report(self):
        return diag.analyse(str(self.root))

    def test_complete_healthy_control(self):
        s, _ = self.report()
        self.assertEqual(s['assessment'], 'no_observed_anomalies')

    def test_console_without_session_events_does_not_erase_run_timestamps(self):
        (self.root/'console.log').write_text('[10.0] agent started\n')
        s,_=self.report()
        self.assertEqual(s['assessment'],'no_observed_anomalies')
        self.assertEqual(s['durationSeconds'],1.)

    def test_dropout_without_gap_and_zero_or_partial_validity_never_healthy(self):
        self.write('bridge-events', [{'at': 10., 'type': 'sensor_dropout'}])
        for ratio in [0, 1]:
            rows=[dict(kind='tick', at=10.+i, lidar=dict(ok=i<ratio, ageSeconds=.1, maxGap5s=0.)) for i in range(2)]
            self.write('bridge-samples', rows)
            s, raw = self.report()
            self.assertEqual(s['assessment'], 'issues_detected')
            self.assertNotIn('未发现明显异常', diag.render(s, raw))
            self.assertTrue(s['findings'])

    def test_no_ping_attempt_is_unknown_not_zero_failures(self):
        self.write('net', [dict(at=10., ping={})])
        s, _ = self.report()
        self.assertEqual(s['network']['pingSamples'], 0)
        self.assertIsNone(s['network']['pingFailures'])
        self.assertIsNone(s['network']['pingFailureRatio'])
        self.assertNotEqual(s['assessment'], 'no_observed_anomalies')

    def test_ping_denominator_excludes_unattempted_and_unknown_outcomes(self):
        self.write('net', [dict(at=10.+i,ping=p) for i,p in enumerate([
            {},dict(ip='a',ok=True,rttMs=1),dict(ip='a',ok=False),dict(ip='a')])])
        n=self.report()[0]['network']
        self.assertEqual((n['pingSamples'],n['pingSuccesses'],n['pingFailures'],n['pingUnknownResults']),(3,1,1,1))
        self.assertEqual(n['pingFailureRatio'],.5)

    def test_missing_empty_and_unparseable_console_are_unknown(self):
        for content in [None,'','not a timestamped log']:
            p=self.root/'console.log'
            if content is None:p.unlink(missing_ok=True)
            else:p.write_text(content)
            s,_=self.report()
            self.assertIsNone(s['microRosRebuilds'])
            self.assertTrue(s['unknowns'])

    def test_complete_disconnect_rebuild_has_measured_duration(self):
        (self.root/'console.log').write_text('[10.0] session closed\n[12.5] session established\n')
        s,_=self.report()
        self.assertEqual(s['microRosRebuilds'],1)
        self.assertEqual(s['microRosWorstRebuildSeconds'],2.5)
        self.assertEqual(s['microRosOpenDisconnects'],0)

    def test_open_disconnect_is_reported_even_without_rebuild(self):
        (self.root/'console.log').write_text('[10.0] session established\n[11.0] session closed\n')
        s,_=self.report()
        self.assertEqual(s['microRosOpenDisconnects'],1)
        self.assertEqual(s['assessment'],'issues_detected')

    def test_corrupt_nonobject_and_missing_fields_do_not_crash_or_pass(self):
        for text in ['[]\nnull\n{broken\n','{"kind":"tick"}\n','']:
            (self.root/'bridge-samples.jsonl').write_text(text)
            s,raw=self.report()
            self.assertNotEqual(s['assessment'],'no_observed_anomalies')
            self.assertTrue(s['unknowns'])
            diag.render(s,raw)

    def test_nav_frontend_and_backend_faults_block_health(self):
        self.write('bridge-events',[dict(at=10.,type='navigation_fault')])
        self.write('frontend-events',[dict(at=10.,type='window-error')])
        s,_=self.report()
        self.assertEqual(s['assessment'],'issues_detected')

    def test_separate_output_preserves_source(self):
        output=self.root/'analysis'
        before={p.name:p.read_bytes() for p in self.root.iterdir() if p.is_file()}
        with patch('sys.argv',['diagnose_run.py',str(self.root),'--output',str(output)]):
            diag.main()
        self.assertTrue((output/'summary.json').exists())
        self.assertEqual(before,{p.name:p.read_bytes() for p in self.root.iterdir() if p.is_file()})


class SamplerTests(unittest.TestCase):
    def test_all_six_counters_track_growth_reset_missing_and_interfaces(self):
        tracker=sampler.CounterTracker()
        values=dict(rxPackets=10,txPackets=20,rxDropped=2,txDropped=3,rxErrors=4,txErrors=5)
        first=tracker.update('wlan0',values)
        self.assertTrue(all(v is None for v in first['delta'].values()))
        next_values={k:v+2 for k,v in values.items()}
        second=tracker.update('wlan0',next_values)
        self.assertEqual(list(second['delta'].values()),[2]*6)
        reset=tracker.update('wlan0',{k:1 for k in values})
        self.assertEqual(list(reset['delta'].values()),[0]*6)
        self.assertEqual(len(reset['resets']),6)
        tracker.update('wlan0',{})
        self.assertTrue(all(v is None for v in tracker.update('wlan0',values)['delta'].values()))
        self.assertTrue(all(v is None for v in tracker.update('wlan1',values)['delta'].values()))

    def test_commands_use_argv_and_never_shell(self):
        with patch.object(sampler.subprocess,'run') as run:
            run.return_value.stdout='';run.return_value.returncode=0
            sampler.ping('127.0.0.1')
            args,kw=run.call_args
            self.assertIsInstance(args[0],list)
            self.assertFalse(kw.get('shell',False))
        with patch.object(sampler.subprocess,'run') as run:
            sampler.ping('127.0.0.1;touch /tmp/never')
            sampler.wifi_line('../../tmp')
            sampler.power_save('-bad')
            self.assertFalse(run.called)

    def test_failures_are_unknown_not_connected_or_power_save_off(self):
        with patch.object(sampler.subprocess,'run',side_effect=FileNotFoundError):
            self.assertIsNone(sampler.wifi_line('wlan0')['connected'])
            self.assertIsNone(sampler.power_save('wlan0'))


if __name__=='__main__':unittest.main()
