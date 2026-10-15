"""Bounded offers with explicit item and byte limits; bounded sorted caches."""
import bisect
from collections import deque
import threading


class ByteQueue:
    def __init__(self, items, byte_limit):
        self.items_limit,self.byte_limit=items,byte_limit
        self.items=deque()
        self.bytes=self.peak_bytes=self.peak_items=self.rejected=0
        self.condition=threading.Condition()
        self.closed=False

    def offer(self,item,size):
        # Never wait for queue capacity. Allow at most 1 ms for a consumer or
        # status snapshot holding the tiny accounting lock; that is not an
        # overflow and must not randomly discard input from an empty queue.
        if not self.condition.acquire(timeout=.001):
            self.rejected+=1
            return False
        try:
            if self.closed or len(self.items)>=self.items_limit or self.bytes+size>self.byte_limit:
                self.rejected+=1
                return False
            self.items.append((item,size))
            self.bytes+=size
            self.peak_bytes=max(self.peak_bytes,self.bytes)
            self.peak_items=max(self.peak_items,len(self.items))
            self.condition.notify()
            return True
        finally:
            self.condition.release()

    def get(self,timeout=.05):
        with self.condition:
            if not self.items and not self.closed:
                self.condition.wait(timeout)
            if not self.items:
                return None
            item,size=self.items.popleft()
            self.bytes-=size
            return item

    def close(self):
        with self.condition:
            self.closed=True
            self.condition.notify_all()

    def status(self):
        with self.condition:
            return {'items':len(self.items),'bytes':self.bytes,'peak_items':self.peak_items,
                'peak_bytes':self.peak_bytes,'rejected':self.rejected,'closed':self.closed}


class TimeCache:
    """One aligner owner. Holds numeric/metadata only, never the RGB/depth pixels."""
    def __init__(self, cfg):
        self.cfg=cfg
        self.values={}
        self.bytes=0
        self.peak_bytes=0
        self.evictions=0

    def add(self,sample,now):
        from .models import Sample
        if type(sample) is not Sample:
            sample=Sample(sample.source_id,sample.source_seq,sample.capture_time_ns,sample.receive_time_ns,
                sample.arrival_time_ns,sample.clock_epoch,sample.document)
        source=self.values.setdefault(sample.source_id,[])
        data=sample.data()
        sort_time=data.get(data.get('action_time_field',''),sample.capture_time_ns)
        bisect.insort(source,(sort_time,sample.source_seq,sample),key=lambda x:x[:2])
        self.bytes+=sample.nbytes
        while len(source)>self.cfg['cache_items_per_source']:
            self.bytes-=source.pop(0)[2].nbytes
            self.evictions+=1
        while self.bytes>self.cfg['cache_bytes']:
            oldest=min((v for v in self.values.values() if v),key=lambda v:v[0][:2])
            self.bytes-=oldest.pop(0)[2].nbytes
            self.evictions+=1
        self.evict(now-int(self.cfg['retention_ms']*1e6))
        self.peak_bytes=max(self.peak_bytes,self.bytes)

    def evict(self,cutoff):
        for values in self.values.values():
            if values and values[-1][2].data().get('joint_units'):
                # Partial streams need a predecessor per axis, plus termination
                # evidence. A single message predecessor loses the other arm.
                keep=set()
                for i,(_,_,sample) in enumerate(values):
                    if sample.capture_time_ns>=cutoff:
                        keep.add(i)
                predecessors={}
                for i,(_,_,sample) in enumerate(values):
                    d=sample.data()
                    for name in d['joint_names']:
                        if sample.capture_time_ns<cutoff:
                            predecessors[(name,bool(d.get('validity_event')))]=i
                keep.update(predecessors.values())
                removed=[entry for i,entry in enumerate(values) if i not in keep]
                self.bytes-=sum(entry[2].nbytes for entry in removed)
                values[:]=[entry for i,entry in enumerate(values) if i in keep]
                continue
            # Preserve one predecessor (holding actions and interpolation brackets).
            while len(values)>1 and values[1][0]<cutoff:
                self.bytes-=values.pop(0)[2].nbytes

    def eligible(self,source,deadline):
        return [x[2] for x in self.values.get(source,[]) if x[2].arrival_time_ns<=deadline]
