"""Standalone monitor tests; no ROS, hardware, or third-party dependencies."""
import http.client
import importlib.util
import json
import os
from pathlib import Path
import tempfile
import threading
import unittest

MODULE_PATH = Path(__file__).resolve().parents[1] / "tienkung_runtime_monitor" / "runtime_monitor.py"
SPEC = importlib.util.spec_from_file_location("runtime_monitor", MODULE_PATH)
monitor_module = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(monitor_module)
RuntimeMonitor = monitor_module.RuntimeMonitor
load_urdf = monitor_module.load_urdf


def sample(timestamp=1, **extra):
    return dict(timestamp=timestamp, mode="STOP", obs=[1, 2, 3], target=[0],
                dof_pos=[0], dof_vel=[0], motion_ref=[], **extra)


def line(value):
    return json.dumps(value).encode() + b"\n"


METADATA = dict(type="metadata", schema_version=2, joint_order_policy=["joint"], config={})
TREE = '''<robot name="test"><link name="base"/><link name="tip"/>
<joint name="joint" type="revolute"><parent link="base"/><child link="tip"/>
<origin xyz="1 2 3" rpy="0 0 1"/><axis xyz="0 0 2"/></joint></robot>'''


class TailTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.path = self.root / "policy_log_1.jsonl"

    def monitor(self, **options):
        if "log_dir" not in options:
            options["log_file"] = self.path
        value = RuntimeMonitor(**options)
        self.addCleanup(value.close)
        return value

    def test_missing_partial_invalid_and_live_fields(self):
        monitor = self.monitor()
        monitor.poll_once()
        self.assertEqual(monitor.snapshot()["frames"], [])
        payload = sample(2, raw_action=[0.1], inference_executed=False,
                         inference_latency_ms=None, state_ready=False, command_published=False)
        data = line(payload)
        self.path.write_bytes(line(METADATA) + b"invalid\n" + data[:-2])
        monitor.poll_once()
        self.assertEqual(monitor.snapshot()["metadata"], METADATA)
        self.assertEqual(monitor.snapshot()["parse_errors"], 1)
        self.assertEqual(monitor.snapshot()["frames"], [])
        with self.path.open("ab") as stream:
            stream.write(data[-2:])
        monitor.poll_once()
        frame = monitor.snapshot()["frames"][0]
        self.assertNotIn("obs", frame)
        self.assertFalse(frame["command_published"])
        self.assertEqual(frame["raw_action"], [0.1])
        self.assertEqual(frame["monitor_seq"], 1)
        self.assertIn(b'"obs"', self.path.read_bytes())
        self.assertEqual(monitor.snapshot(1)["frames"], [])
        self.assertIsNotNone(monitor.snapshot()["age_sec"])

    def test_rotation_truncation_and_regrowth(self):
        monitor = self.monitor()
        self.path.write_bytes(line(METADATA) + line(sample(1)))
        monitor.poll_once()
        self.path.write_bytes(line(sample(2)))
        monitor.poll_once()
        self.assertEqual(monitor.generation, 2)
        self.assertIsNone(monitor.metadata)
        self.assertEqual(monitor.last_seq, 2)
        # Truncate and regrow beyond the previous offset between polls.
        self.path.write_bytes(line(sample(3)) + line(sample(4)))
        monitor.poll_once()
        self.assertEqual(monitor.generation, 3)
        self.assertEqual([f["timestamp"] for f in monitor.snapshot()["frames"]], [3, 4])
        self.path.rename(self.root / "old.jsonl")
        self.path.write_bytes(line(sample(5)))
        monitor.poll_once()
        self.assertEqual(monitor.generation, 4)
        self.assertEqual(monitor.snapshot()["frames"][0]["monitor_seq"], 5)

    def test_newest_run_switch_and_no_log_directory(self):
        directory = self.root / "not-yet"
        monitor = self.monitor(log_dir=directory)
        monitor.poll_once()
        self.assertIsNone(monitor.source)
        directory.mkdir()
        old = directory / "policy_log_1.jsonl"
        old.write_bytes(line(METADATA) + line(sample(1)))
        os.utime(old, (1, 1))
        monitor.poll_once()
        new = directory / "policy_log_2.jsonl"
        new.write_bytes(line(METADATA) + line(sample(2)))
        os.utime(new, (2, 2))
        monitor.poll_once()
        snapshot = monitor.snapshot()
        self.assertEqual(snapshot["generation"], 2)
        self.assertEqual(snapshot["source"], str(new))
        self.assertEqual([f["timestamp"] for f in snapshot["frames"]], [2])
        self.assertEqual(snapshot["last_seq"], 2)

    def test_bounded_history_response_and_lines(self):
        monitor = self.monitor(history_limit=5, response_limit=3, max_line_bytes=512)
        self.path.write_bytes(line(METADATA) + b"x" * 600 + b"\n" +
                              b"".join(line(sample(index)) for index in range(20)))
        monitor.poll_once()
        snapshot = monitor.snapshot()
        self.assertEqual(snapshot["history_size"], 5)
        self.assertEqual(len(snapshot["frames"]), 3)
        self.assertTrue(snapshot["truncated"])
        self.assertEqual(snapshot["skipped_lines"], 1)
        with self.path.open("ab") as stream:
            stream.write(b"x" * 1000)
        monitor.poll_once()
        self.assertLessEqual(len(monitor._partial), 512)
        with self.path.open("ab") as stream:
            stream.write(b"x" * 1000 + b"\n" + line(sample(21)))
        monitor.poll_once()
        self.assertEqual(monitor.snapshot()["frames"][-1]["timestamp"], 21)
        self.assertEqual(monitor.skipped_lines, 2)

    def test_tail_window_metadata_and_byte_budget(self):
        monitor = self.monitor(max_tail_bytes=600, max_history_bytes=300)
        self.path.write_bytes(line(METADATA) + b"".join(line(sample(i)) for i in range(100)))
        monitor.poll_once()
        self.assertEqual(monitor.metadata, METADATA)
        self.assertGreater(monitor.skipped_bytes, 0)
        self.assertLessEqual(monitor._history_bytes, 300)
        self.assertEqual(monitor.snapshot()["frames"][-1]["timestamp"], 99)

    def test_default_full_bounded_history_and_opt_in_obs(self):
        monitor = self.monitor(history_limit=4, include_obs=True)
        self.path.write_bytes(b"".join(line(sample(i)) for i in range(6)))
        monitor.poll_once()
        self.assertEqual(len(monitor.snapshot(0)["frames"]), 4)
        self.assertIn("obs", monitor.snapshot()["frames"][0])
        self.assertTrue(monitor.snapshot(1)["truncated"])

    def test_nonfinite_and_wrong_records_rejected(self):
        monitor = self.monitor()
        self.path.write_bytes(b'[]\n{"timestamp":NaN,"mode":"STOP"}\n'
                              b'{"timestamp":1,"mode":"STOP","x":1e999}\n'
                              b'{"type":"metadata","schema_version":1}\n{}\n')
        monitor.poll_once()
        self.assertEqual(monitor.parse_errors, 5)
        self.assertFalse(monitor.snapshot()["frames"])

    def test_logger_stats_separate_from_samples_and_reset_on_rotation(self):
        monitor = self.monitor()
        stats = dict(type="logger_stats", schema_version=2, dropped_records=3,
                     written_records=1, write_errors=0)
        self.path.write_bytes(line(METADATA) + line(sample()) + line(stats))
        monitor.poll_once()
        snapshot = monitor.snapshot()
        self.assertEqual(snapshot["logger_stats"], stats)
        self.assertEqual(snapshot["parse_errors"], 0)
        self.assertEqual(snapshot["last_seq"], 1)
        self.assertEqual(len(snapshot["frames"]), 1)
        self.path.rename(self.root / "old.jsonl")
        self.path.write_bytes(line(sample(2)))
        monitor.poll_once()
        self.assertIsNone(monitor.snapshot()["logger_stats"])
        with self.path.open("ab") as stream:
            stream.write(line(dict(stats, write_errors=-1)))
        monitor.poll_once()
        self.assertEqual(monitor.parse_errors, 1)

    def test_historical_source_age_and_huge_timestamp(self):
        monitor = self.monitor()
        self.path.write_bytes(line(sample(10 ** 400)) + line(sample(0)))
        os.utime(self.path, (1, 1))
        monitor.poll_once()
        snapshot = monitor.snapshot()
        self.assertEqual(snapshot["parse_errors"], 1)
        self.assertEqual(snapshot["last_seq"], 1)
        self.assertEqual(snapshot["frames"][0]["timestamp"], 0)
        self.assertGreater(snapshot["source_age_sec"], 3600)
        self.assertGreater(snapshot["age_sec"], 3600)

    def test_background_single_thread(self):
        monitor = self.monitor()
        monitor.start()
        thread = monitor._thread
        monitor.start()
        self.assertIs(monitor._thread, thread)
        monitor.close()
        self.assertFalse(thread.is_alive())


class UrdfTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.path = self.root / "robot.urdf"

    def load(self, text, **kwargs):
        self.path.write_text(text)
        return load_urdf(self.path, **kwargs)

    def test_tree_defaults_normalized_axis_and_missing(self):
        self.assertIsNone(load_urdf(None))
        self.assertIsNone(load_urdf(self.path))
        tree = self.load(TREE)
        self.assertEqual(tree["root_links"], ["base"])
        self.assertEqual(tree["joints"][0]["axis"], [0, 0, 1])
        self.assertEqual(tree["joints"][0]["origin_xyz"], [1, 2, 3])
        self.assertEqual(tree["visuals"], [])
        fixed = self.load(TREE.replace('type="revolute"', 'type="fixed"').replace('0 0 2', '0 0 0'))
        self.assertEqual(fixed["joints"][0]["axis"], [0, 0, 0])

    def test_reject_invalid_xml_and_kinematics(self):
        invalid = [
            '<!DOCTYPE robot [<!ENTITY x "foo">]><robot name="&x;"/>',
            '<robot>', TREE.replace('0 0 2', '0 0 0'),
            TREE.replace('0 0 2', '1 2'), TREE.replace('0 0 2', 'nan 0 1'),
            TREE.replace('1 2 3', 'inf 2 3'),
            TREE.replace('<axis xyz="0 0 2"/>', '<axis/>'),
            TREE.replace('child link="tip"', 'child link="base"'),
            TREE.replace('child link="tip"', 'child link="unknown"'),
            TREE.replace('</robot>', '<link name="tip"/></robot>'),
            TREE.replace('</robot>', '<joint name="second" type="fixed"><parent link="base"/>'
                         '<child link="tip"/></joint></robot>'),
            TREE.replace('</joint>', '<mimic joint="unknown"/></joint>'),
            TREE.replace('</joint>', '<mimic joint="joint"/></joint>'),
        ]
        for text in invalid:
            with self.subTest(text=text), self.assertRaises(ValueError):
                self.load(text)
        with self.assertRaises(ValueError):
            self.load(TREE, max_bytes=10)
        self.path.write_bytes(TREE.encode('utf-16'))
        with self.assertRaises(ValueError):
            load_urdf(self.path)

    def test_visuals_and_mesh_confinement(self):
        mesh_dir = self.root / "meshes"
        mesh_dir.mkdir()
        mesh = mesh_dir / "body.STL"
        mesh.write_bytes(b"solid body\nendsolid body\n")
        text = TREE.replace('<link name="base"/>', '<link name="base"><visual>'
                            '<origin xyz="1 0 0"/><geometry><mesh filename="meshes/body.STL" '
                            'scale="1 2 3"/></geometry></visual><visual><geometry>'
                            '<box size="1 2 3"/></geometry></visual><visual><geometry>'
                            '<sphere radius="2"/></geometry></visual><visual><geometry>'
                            '<cylinder radius="1" length="3"/></geometry></visual></link>')
        files = []
        tree = self.load(text, mesh_files=files)
        self.assertEqual(files, [mesh.resolve()])
        self.assertEqual(tree["visuals"][0]["geometry"],
                         dict(type="mesh", url="/meshes/0", scale=[1, 2, 3]))
        self.assertEqual(tree["visuals"][0]["origin_xyz"], [1, 0, 0])
        for filename in ('../body.STL', '/tmp/body.STL', 'file:///tmp/body.STL',
                         'https://example.com/body.STL', 'meshes/%2e%2e/body.STL'):
            with self.subTest(filename=filename), self.assertRaises(ValueError):
                self.load(text.replace('meshes/body.STL', filename))
        outside = self.root / "outside.STL"
        outside.write_bytes(b"secret")
        mesh.unlink()
        mesh.symlink_to(outside)
        with self.assertRaises(ValueError):
            self.load(text, asset_root=mesh_dir)

    def test_package_mesh_local_resolution(self):
        mesh = self.root / "body.stl"
        mesh.write_bytes(b"solid body\nendsolid body\n")
        text = '<robot><link name="base"><visual><geometry><mesh filename="package://{}/body.stl"/>'
        text = text.format(self.root.name) + '</geometry></visual></link></robot>'
        files = []
        self.load(text, mesh_files=files)
        self.assertEqual(files, [mesh.resolve()])


class HttpTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.assets = self.root / "monitor"
        self.assets.mkdir()
        for filename in ('index.html', 'monitor.js', 'monitor.css', 'robot_view.js'):
            (self.assets / filename).write_text(filename)
        log = self.root / "private.jsonl"
        log.write_bytes(line(METADATA) + line(sample()))
        self.monitor = RuntimeMonitor(log_file=log)
        self.monitor.poll_once()
        self.addCleanup(self.monitor.close)
        self.mesh = self.root / 'body.stl'
        self.mesh.write_bytes(b'solid body\nendsolid body\n')
        self.server = monitor_module.MonitorServer(
            ('127.0.0.1', 0), self.monitor, self.assets,
            mesh_files=[self.mesh], asset_root=self.root)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.addCleanup(self.stop_server)

    def stop_server(self):
        self.server.shutdown()
        self.thread.join()
        self.server.server_close()

    def request(self, path, method='GET', headers=None):
        connection = http.client.HTTPConnection('127.0.0.1', self.server.server_port, timeout=3)
        self.addCleanup(connection.close)
        connection.request(method, path, headers=headers or {})
        response = connection.getresponse()
        return response.status, dict(response.getheaders()), response.read()

    def test_fixed_routes_snapshot_headers_health(self):
        for route in ('/', '/monitor.js', '/monitor.css', '/robot_view.js', '/meshes/0'):
            self.assertEqual(self.request(route)[0], 200)
        status, headers, body = self.request('/api/snapshot?after=0')
        self.assertEqual(status, 200)
        self.assertEqual(json.loads(body)['last_seq'], 1)
        self.assertNotIn('obs', json.loads(body)['frames'][0])
        self.assertEqual(headers['X-Content-Type-Options'], 'nosniff')
        self.assertIn("frame-ancestors 'none'", headers['Content-Security-Policy'])
        self.assertNotIn('Access-Control-Allow-Origin', headers)
        self.assertEqual(json.loads(self.request('/api/snapshot?since=0')[2])['history_size'], 1)
        health = json.loads(self.request('/api/health')[2])
        self.assertEqual(health['status'], 'ok')
        self.assertNotIn('frames', health)

    def test_no_arbitrary_files_controls_or_traversal(self):
        for path in ('/../private.jsonl', '/%2e%2e/private.jsonl', '/private.jsonl',
                     '/api/control', '/api/log', '/monitor/', '//evil.com/api/snapshot',
                     '/meshes/../private.jsonl', '/meshes/1', '/meshes/00'):
            with self.subTest(path=path):
                self.assertNotEqual(self.request(path)[0], 200)
        for method in ('POST', 'PUT', 'PATCH', 'DELETE', 'OPTIONS', 'HEAD'):
            self.assertEqual(self.request('/api/snapshot', method)[0], 405)
        for query in ('after=-1', 'after=x', 'after=1&after=2', 'after=0&since=0',
                      'path=/tmp/foo', 'after=', 'after=999999999999999999999999'):
            self.assertEqual(self.request('/api/snapshot?' + query)[0], 400)

    def test_host_rebinding_and_cross_site_blocked(self):
        for host in ('evil.example', '127.0.0.1.evil.example', 'localhost:1',
                     'user@localhost', 'localhost/path', 'localhost,evil.example'):
            self.assertEqual(self.request('/api/snapshot', headers={'Host': host})[0], 403)
        self.assertEqual(self.request('/api/snapshot', headers={'Sec-Fetch-Site': 'cross-site'})[0], 403)
        host = 'localhost:' + str(self.server.server_port)
        self.assertEqual(self.request('/api/health', headers={'Host': host})[0], 200)

    def test_asset_symlink_and_changed_mesh_denied(self):
        (self.assets / 'monitor.js').unlink()
        (self.assets / 'monitor.js').symlink_to(self.root / 'private.jsonl')
        self.assertEqual(self.request('/monitor.js')[0], 404)
        self.mesh.unlink()
        self.mesh.symlink_to(self.root / 'private.jsonl')
        self.assertEqual(self.request('/meshes/0')[0], 404)


if __name__ == '__main__':
    unittest.main()
