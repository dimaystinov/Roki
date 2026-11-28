import io
import json
from pathlib import Path
import tempfile
import unittest
import zipfile
from upstream_updater import extract_source, activate, rollback, SourceFinder

def archive(extra=None):
    stream = io.BytesIO()
    with zipfile.ZipFile(stream, 'w') as z:
        for path in ['roki_operator.py', 'operator_gui/__init__.py', 'qml/Operator.qml']:
            z.writestr('repo-sha/'+path, '# source')
        if extra: z.writestr(*extra)
    return stream.getvalue()

class UpstreamUpdaterTests(unittest.TestCase):
    def test_extracts_snapshot_without_changing_neighbour_install(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);(root/'Roki.txt').write_text('custom');extract_source(archive(),root/'stage')
            self.assertTrue(((root/'stage/roki_operator.py').resolve()).is_file())
            self.assertEqual((root/'Roki.txt').read_text(),'custom')
    def test_rejects_path_traversal_and_symlink(self):
        for path in ['repo-sha/../../outside','/absolute','repo-sha/../outside','repo-sha/C:/outside']:
            with self.subTest(path=path),tempfile.TemporaryDirectory() as d,self.assertRaises(ValueError):
                extract_source(archive((path,'bad')),Path(d)/'stage')
        link=zipfile.ZipInfo('repo-sha/link');link.create_system=3;link.external_attr=0o120777<<16
        with tempfile.TemporaryDirectory() as d,self.assertRaises(ValueError):
            extract_source(archive((link,'../../outside')),Path(d)/'stage')
    def test_failed_probe_preserves_working_version(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);old='a'*40;new='b'*40
            for sha in [old,new]:(root/'versions'/sha).mkdir(parents=True)
            activate(root,old,lambda p:None)
            def fail(p):raise RuntimeError('Qt incompatible')
            with self.assertRaises(RuntimeError):activate(root,new,fail)
            self.assertEqual(json.loads((root/'current.json').read_text())['sha'],old)
            activate(root,new,lambda p:None);rollback(root)
            self.assertEqual(json.loads((root/'current.json').read_text())['sha'],old)
    def test_source_finder_never_falls_back_to_bundled_app(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);extract_source(archive(),root/'stage');finder=SourceFinder(root/'stage')
            self.assertEqual(Path(finder.find_spec('roki_operator').origin),(root/'stage/roki_operator.py').resolve())
            with self.assertRaises(ModuleNotFoundError):finder.find_spec('operator_gui.missing')
            self.assertIsNone(finder.find_spec('json'))

    def test_linux_native_links_are_excluded_from_platform_snapshot(self):
        link=zipfile.ZipInfo('repo-sha/native/qml/lib.so')
        link.create_system=3;link.external_attr=0o120777<<16
        with tempfile.TemporaryDirectory() as d:
            target=Path(d)/'stage'
            extract_source(archive((link,'../../linux-only')),target)
            self.assertFalse((target/'native').exists())
            self.assertTrue((target/'roki_operator.py').exists())
