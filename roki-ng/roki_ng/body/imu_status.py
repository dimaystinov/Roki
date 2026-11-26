"""Local LCD status. Repeated quantized values mean suspect, never proven failure."""
import json
import math
from pathlib import Path

STATUS_PATH = Path('/run/roki-body-imu.json')


class ImuDisplayStatus:
    def __init__(self, path=STATUS_PATH):
        self.path = Path(path)
        self.previous = None
        self.changed_at = None
        self.seen_change = False
        self.write_at = 0

    def update(self, sample, now):
        q = sample.get('quaternion_xyzw')
        valid = (sample.get('valid') and isinstance(q, (tuple,list)) and len(q)==4
                 and all(isinstance(v,(int,float)) and math.isfinite(v) for v in q))
        if sample.get('measurement_stalled'):
            state='stalled'
        elif valid:
            counter=sample.get('sensor_sequence')
            q=(counter, sample.get('sensor_timestamp_ticks')) if counter is not None else tuple(q)
            if self.previous is None:
                self.changed_at=now
                self.seen_change=False
            elif q != self.previous:
                self.changed_at=now
                self.seen_change=True
            self.previous=q
            state=('suspect' if now-self.changed_at >= 3 else
                   'updating' if self.seen_change else 'checking')
        else:
            state='no_data'
            self.previous=self.changed_at=None
            self.seen_change=False
        if now < self.write_at:
            return
        self.write_at=now+.5
        data={'state':state, 'written_at':now,
              'active_source':sample.get('active_source', 'body' if valid else 'none'),
              'counter_present':sample.get('sensor_sequence') is not None,
              'unchanged_s':int(now-self.changed_at) if self.changed_at is not None else 0}
        temp=self.path.with_suffix('.tmp')
        try:
            temp.write_text(json.dumps(data))
            temp.replace(self.path)
        except OSError:
            # LCD is optional: failure must not interrupt servo scheduling.
            pass


def read_status(path=STATUS_PATH, now=0):
    unavailable={'state':'no_data','unchanged_s':0}
    try:
        value=json.loads(Path(path).read_text())
        age=now-value['written_at']
        if not 0 <= age <= 2 or value['state'] not in ('checking','updating','suspect','no_data','stalled'):
            return unavailable
        return value
    except (OSError,ValueError,TypeError,KeyError):
        return unavailable
