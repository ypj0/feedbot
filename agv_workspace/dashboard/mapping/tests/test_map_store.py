import json
from http.client import HTTPConnection
from http.server import ThreadingHTTPServer
from pathlib import Path
import sys
import tempfile
import threading
import unittest
from types import SimpleNamespace
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from map_store import DOWNLOAD, Handler, Store, inspect_map


def write_map(directory):
    (directory / 'feedbot.yaml').write_text('image: feedbot.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n')
    (directory / 'feedbot.pgm').write_bytes(b'P5\n# map\n2 1\n255\n\x00\xfe')


class MapStoreTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.store = Store(self.root)

    def test_file_validation_and_download_paths(self):
        write_map(self.root)
        result = inspect_map(self.root)
        self.assertEqual((result['width'], result['height']), (2, 1))
        self.assertEqual(len(result['files']['feedbot.pgm']['sha256']), 64)
        self.assertIsNone(DOWNLOAD.fullmatch('/api/maps/../feedbot.yaml'))
        (self.root / 'feedbot.pgm').write_bytes(b'P5\n2 1\n255\n\x00')
        with self.assertRaises(ValueError):
            inspect_map(self.root)

    def test_async_save_and_duplicate_submission(self):
        entered = threading.Event()
        release = threading.Event()
        def save_command(argv, **kwargs):
            self.assertEqual(argv[-1], 'save_map_timeout:=15.0')
            directory = Path(argv[argv.index('-f') + 1]).parent
            entered.set()
            release.wait(2)
            write_map(directory)
            return SimpleNamespace(returncode=0)
        with patch('map_store.subprocess.run', side_effect=save_command):
            started = self.store.start()
            self.assertEqual(started['phase'], 'saving')
            self.assertTrue(entered.wait(2))
            with self.assertRaises(ValueError):
                self.store.start()
            release.set()
            for worker in threading.enumerate():
                if worker is not threading.current_thread() and worker.name.startswith('Thread-'):
                    worker.join(2)
        result = self.store.snapshot()
        self.assertEqual(result['phase'], 'saved')
        self.assertEqual(Store(self.root).snapshot(), result)

    def test_failure_is_visible_and_not_retried(self):
        directory = self.root / 'failed'
        directory.mkdir()
        def failed_command(argv, **kwargs):
            kwargs['stdout'].write('Failed to spin map subscription')
            return SimpleNamespace(returncode=1)
        with patch('map_store.subprocess.run', side_effect=failed_command) as command:
            self.store.save(directory)
        self.assertEqual(command.call_count, 1)
        self.assertEqual(self.store.snapshot()['phase'], 'failed')
        self.assertIn('Failed to spin', self.store.snapshot()['error'])
        self.assertIsNone(self.store.snapshot()['result'])

    def test_interrupted_save_is_not_success(self):
        self.store.state['phase'] = 'saving'
        self.store.persist()
        state = Store(self.root).snapshot()
        self.assertEqual(state['phase'], 'failed')
        self.assertIn('中断', state['error'])

    def test_http_auth_status_download_and_confirmation(self):
        directory = self.root / 'fixture'
        directory.mkdir()
        write_map(directory)
        server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
        server.store = self.store
        server.token = 'test-token'
        worker = threading.Thread(target=server.serve_forever)
        worker.start()
        connection = HTTPConnection(*server.server_address)
        try:
            connection.request('GET', '/api/save')
            response = connection.getresponse()
            self.assertEqual(response.status, 403)
            response.read()
            headers = {'X-Test-Token': 'test-token'}
            connection.request('GET', '/api/save', headers=headers)
            response = connection.getresponse()
            self.assertEqual(json.loads(response.read())['phase'], 'idle')
            connection.request('GET', '/api/maps/fixture/feedbot.pgm', headers=headers)
            response = connection.getresponse()
            self.assertEqual(response.status, 200)
            self.assertIn('attachment', response.getheader('Content-Disposition'))
            self.assertEqual(response.read(), (directory / 'feedbot.pgm').read_bytes())
            connection.request('POST', '/api/save', '{}', headers)
            response = connection.getresponse()
            self.assertEqual(response.status, 409)
            response.read()
            with patch.object(self.store, 'start', return_value={'phase': 'saving'}) as start:
                connection.request('POST', '/api/save', '{"confirm_save":true}', headers)
                response = connection.getresponse()
                self.assertEqual(response.status, 202)
                response.read()
                start.assert_called_once()
        finally:
            connection.close()
            server.shutdown()
            server.server_close()
            worker.join()


if __name__ == '__main__':
    unittest.main()
