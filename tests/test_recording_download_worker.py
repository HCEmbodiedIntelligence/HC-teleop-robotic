"""Large-file progress and interruption without touching robotics or large data."""
import tempfile
from pathlib import Path
import unittest
import zipfile

from middleware.core.recording.recording_download_worker import build_download, CHUNK_SIZE


class ArchiveWorkerTests(unittest.TestCase):
    def test_progress_advances_inside_one_file_and_zip_is_intact(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);source=root/'large.mcap';source.write_bytes(b'x'*(3*CHUNK_SIZE+17))
            archive=root/'out.zip';events=[]
            result=build_download(root,['large.mcap'],archive,progress=lambda **fields:events.append(fields))
            steps=[e['bytes_done'] for e in events if e.get('bytes_done')]
            self.assertTrue(any(0<n<source.stat().st_size for n in steps))
            self.assertEqual(steps,sorted(steps))
            self.assertEqual(steps[-1],source.stat().st_size)
            self.assertTrue(result['owned'])
            with zipfile.ZipFile(archive) as output:
                self.assertIsNone(output.testzip())
                self.assertEqual(output.read('mcap/large.mcap'),source.read_bytes())

    def test_cancel_in_middle_of_one_file_removes_partial_archive_only(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);source=root/'large.mcap';source.write_bytes(b'x'*(3*CHUNK_SIZE+17))
            archive=root/'out.zip';observed=[]
            def cancel(**fields):
                if fields.get('bytes_done',0)>0:
                    observed.append(fields['bytes_done'])
                    raise InterruptedError('cancelled by operator')
            with self.assertRaises(InterruptedError):
                build_download(root,['large.mcap'],archive,progress=cancel)
            self.assertLess(observed[0],source.stat().st_size)
            self.assertFalse(archive.exists())
            self.assertEqual(source.stat().st_size,3*CHUNK_SIZE+17)

    def test_changing_source_fails_and_removes_partial_archive(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);source=root/'large.mcap';source.write_bytes(b'x'*(2*CHUNK_SIZE+17))
            archive=root/'out.zip';changed=False
            def mutate(**fields):
                nonlocal changed
                if not changed and fields.get('bytes_done',0)>0:
                    with source.open('ab') as stream:stream.write(b'changed')
                    changed=True
            with self.assertRaisesRegex(ValueError,'source changed'):
                build_download(root,['large.mcap'],archive,progress=mutate)
            self.assertFalse(archive.exists())
            self.assertTrue(source.read_bytes().endswith(b'changed'))
