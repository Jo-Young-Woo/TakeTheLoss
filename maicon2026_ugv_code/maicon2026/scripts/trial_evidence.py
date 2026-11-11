"""Bounded diagnostic image output, separate from simulation/control work."""
from copy import deepcopy
from pathlib import Path
import queue
import threading


class BoundedAimEvidence:
    def __init__(self, directory, replace, write_json, *, limit=24, interval_s=1.):
        self.directory=Path(directory)
        self.replace,self.write_json=replace,write_json
        self.limit,self.interval_s=limit,interval_s
        self.queue=queue.Queue(maxsize=2)
        self.stop=threading.Event()
        self.last_attempt=-float('inf')
        self.seen=set()
        self.stats=dict(accepted=0,saved=0,missing_exact_source=0,queue_full=0,errors=[],files=[])
        self.worker=threading.Thread(target=self._run,name='aim-evidence-writer',daemon=True)
        self.worker.start()

    def wants(self,snapshot):
        if (not snapshot or not snapshot.get('updated_this_tick') or
                snapshot.get('mode') not in ('STRIKE','HUNT') or
                self.stats['accepted']>=self.limit):
            return False
        return float(snapshot['now'])-self.last_attempt>=self.interval_s

    def offer(self,snapshot,image):
        if not self.wants(snapshot):return False
        now=float(snapshot['now'])
        self.last_attempt=now
        stamp=snapshot.get('consumed_frame_stamp')
        if image is None or not snapshot.get('source_frame_available'):
            self.stats['missing_exact_source']+=1
            return False
        if stamp in self.seen:return False
        try:self.queue.put_nowait((self.stats['accepted'],image.copy(),deepcopy(snapshot)))
        except queue.Full:
            self.stats['queue_full']+=1
            return False
        self.seen.add(stamp)
        self.stats['accepted']+=1
        return True

    def _run(self):
        import cv2
        while not self.stop.is_set() or not self.queue.empty():
            try:index,pixels,snapshot=self.queue.get(timeout=.1)
            except queue.Empty:continue
            try:
                self.directory.mkdir(parents=True,exist_ok=True)
                name=f'aim_source_{index:03d}'
                path=self.directory/(name+'.png')
                temporary=path.with_suffix('.png.tmp')
                ok,data=cv2.imencode('.png',pixels)
                if not ok:raise OSError('PNG encoder rejected exact source image')
                temporary.write_bytes(data.tobytes())
                self.replace(temporary,path)
                metadata=dict(snapshot,source_image=path.name,exact_capture_stamp_match=True,
                              pixels='Original detector source RGB/BGR frame; no annotation or crop')
                if self.write_json(self.directory/(name+'.json'),metadata) is False:
                    raise OSError('Source image metadata preview skipped')
                self.stats['saved']+=1
                self.stats['files'].append(path.name)
            except Exception as exc:
                # Evidence output is optional; report failure without disrupting
                # the mission or substituting a different camera frame.
                self.stats['errors'].append(type(exc).__name__+': '+str(exc))
            finally:self.queue.task_done()

    def close(self):
        self.stop.set();self.worker.join(timeout=5.)
        if self.worker.is_alive():self.stats['errors'].append('Evidence writer did not drain within 5 seconds')
        return deepcopy(self.stats)
