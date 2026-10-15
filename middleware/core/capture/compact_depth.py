"""Sealed 12-bit depth media; masks and source timing remain in MCAP references."""
import base64
import hashlib
import zlib
import os
import threading

import av
import numpy as np

from .compact_depth_codec import DEFAULT_PROFILE, DepthVideo, quantize
from .depth_source import require_depth_source
from .media_spool import MediaSpool


class CompactDepth:
    def __init__(self, root, ident, fps, shape, segment=0):
        self.relative = f'videos/{ident}_depth/{segment:06d}.mp4'
        self.path = root / self.relative
        self.video = DepthVideo(self.path, fps, shape)
        self.expected = hashlib.sha256()
        self.frames = 0

    def write(self, data, raw):
        meta = data['depth']
        pixels = np.frombuffer(raw, dtype=meta['dtype']).reshape(meta['shape'])
        source = require_depth_source(meta, pixels)
        result = quantize(pixels, source['scale_m_per_unit'],
                          invalid_values=source['invalid_values'])
        self.video.write_codes(result.codes)
        packed = result.codes.astype('<u2').tobytes()
        self.expected.update(packed)
        reference = {
            'schema_version': 'compact-depth-frame/v1',
            'video_segment_path': self.relative, 'display_frame_index': self.frames,
            'pts': self.frames, 'time_base': [1, int(self.video.fps)],
            'profile': DEFAULT_PROFILE.document(), 'profile_sha256': DEFAULT_PROFILE.sha256,
            'codes_sha256': hashlib.sha256(packed).hexdigest(), 'source_dtype': pixels.dtype.str,
            'flags_zlib_base64': base64.b64encode(zlib.compress(result.flags.tobytes())).decode(),
            'quantization_report': result.error_report(),
        }
        self.frames += 1
        return {**data, 'depth_video': reference}

    def close(self):
        self.video.close()
        actual = hashlib.sha256()
        count = 0
        with av.open(str(self.path)) as container:
            stream = container.streams.video[0]
            if (stream.codec_context.name, stream.codec_context.format.name) != ('hevc', 'gray12le'):
                raise ValueError('depth output must be HEVC gray12le')
            for frame in container.decode(video=0):
                actual.update(frame.to_ndarray(format='gray12le').astype('<u2').tobytes())
                count += 1
        if count != self.frames or actual.digest() != self.expected.digest():
            raise ValueError('depth MP4 decode changed quantized codes')


def _encode_worker(connection, root, ident, cfg):
    """Separate process per camera: PyAV/x265 work cannot hold the manager GIL."""
    from pathlib import Path
    current=None
    segment=0
    first=None
    def commit(video):
        video.close()
        if cfg['storage']['fsync_on_commit']:
            with video.path.open('rb') as f:os.fsync(f.fileno())
        return {'path':video.relative,'kind':'depth_mp4','source_id':ident,
                'frames':video.frames,'closed':True,'fsynced':cfg['storage']['fsync_on_commit']}
    try:
        connection.send({'ready':True})
        while True:
            item=connection.recv()
            if item is None:
                connection.send({'commits':[commit(current)] if current is not None else []})
                break
            data,raw=item
            commits=[]
            if current is not None and (data['capture_time_ns']-first >= cfg['storage']['video_segment_seconds']*1e9
                    or tuple(data['depth']['shape'][:2]) != current.video.shape):
                commits.append(commit(current));current=None;segment+=1
            if current is None:
                current=CompactDepth(Path(root),ident,cfg['sources'][ident].get('fps',cfg['dataset']['fps']),
                                     data['depth']['shape'],segment)
                first=data['capture_time_ns']
            connection.send({'reference':current.write(data,raw),'commits':commits})
    except BaseException as error:
        try:connection.send({'error':f'{type(error).__name__}: {error}'})
        except (OSError,EOFError):pass
    finally:
        if current is not None:current.video.close()
        connection.close()


class DepthEncoder:
    """Disk-backed input and independent CPU process per depth camera."""
    def __init__(self, ident, cfg, storage, fail):
        import multiprocessing
        self.ident,self.cfg,self.storage,self.fail=ident,cfg,storage,fail
        self.queue=MediaSpool(storage.path/'.media-spool'/(ident+'-depth.frames'),
                              sync=cfg['storage']['fsync_on_commit'],
                               reserve_bytes=cfg['storage']['minimum_free_bytes'])
        self.encoded,self.error=0,None
        context=multiprocessing.get_context('spawn')
        self.connection,child=context.Pipe()
        self.process=context.Process(target=_encode_worker,
            args=(child,str(storage.path),ident,cfg),name='depth-encoder-'+ident,daemon=True)
        self.process.start();child.close()
        self.thread=threading.Thread(target=self.run,name='depth-feed-'+ident,daemon=True)
        self.thread.start()

    def submit(self,sample):
        return self.queue.offer((sample.data(),sample.depth_values),
                                 len(sample.document)+len(sample.depth_values)+128)

    def receive(self):
        if not self.connection.poll(30):raise RuntimeError('depth encoder response timeout')
        reply=self.connection.recv()
        if 'error' in reply:raise RuntimeError(reply['error'])
        for value in reply.get('commits',[]):
            if not self.storage.required('commit',value):raise RuntimeError('depth commit queue failed')
        if 'reference' in reply:
            if not self.storage.required('depth_video_frame',reply['reference']):
                raise RuntimeError('depth frame reference queue failed')
            self.encoded+=1
        return reply

    def run(self):
        try:
            self.receive()
            while not self.queue.closed or self.queue.items:
                item=self.queue.get()
                if item is None:continue
                self.connection.send(item)
                self.receive()
            self.connection.send(None)
            self.receive()
        except Exception as error:
            self.error=f'{type(error).__name__}: {error}'
            self.fail('depth_encoder_failure:'+self.ident+':'+self.error)
        finally:
            self.connection.close()

    def stop(self):
        self.queue.close()
        self.thread.join()
        if self.thread.is_alive():self.fail('depth_encoder_close_timeout:'+self.ident)
        self.process.join(1)
        if self.process.is_alive():
            self.process.terminate();self.process.join(5)
        self.thread.join(1)

    def status(self):
        return {'queue':self.queue.status(),'encoded':self.encoded,'error':self.error,
                'encoder_pid':self.process.pid,'execution':'independent_process'}
