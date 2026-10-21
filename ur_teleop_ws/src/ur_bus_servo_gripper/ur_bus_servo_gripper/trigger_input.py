"""Strict, coherent bridge input contract. No Joy fallback and no invented event time."""
import json
import math


class TriggerInput:
    def __init__(self):
        self.last = None
        self.rejected = 0
        self.events = 0

    def parse(self, text, now_ns, source_fresh):
        try:
            d = json.loads(text)
            if not isinstance(d, dict) or d.get('schema') != 1 or not source_fresh:
                raise ValueError('schema or DDS age')
            for key in ('stamp_ns', 'observation_seq', 'event_seq', 'receipt_ns', 'imu_seq', 'loss_seq', 'rejected_events'):
                if type(d[key]) is not int or d[key] < 0:
                    raise ValueError(key)
            for key in ('event_time', 'event_age_s', 'imu_age_s'):
                if type(d[key]) not in (float, int) or not math.isfinite(d[key]):
                    raise ValueError(key)
            for key in ('pose', 'optical'):
                if type(d[key]) is not bool:
                    raise ValueError(key)
            if not all(isinstance(d[k], str) and 0 < len(d[k]) <= 512 for k in ('session', 'device')):
                raise ValueError('identity')
            age = (now_ns - d['stamp_ns']) / 1e9
            if not 0 <= age < .25 or d['observation_seq'] < 1:
                raise ValueError('publication age/order')
            if d['state'] not in ('GENUINE_FRESH_INPUT', 'VALID_HELD_INPUT',
                                  'TEMPORARILY_UNVERIFIED', 'DISCONNECTED_OR_STALE'):
                raise ValueError('state')
            raw = d['raw']
            if raw is not None and (type(raw) not in (float, int) or not math.isfinite(raw) or not 0 <= raw <= 1):
                raise ValueError('trigger')
            old = self.last
            changed = old is not None and (d['session'], d['device']) != (old['session'], old['device'])
            same = old is not None and not changed
            new = not same or d['event_seq'] > old['event_seq']
            if same:
                if d['observation_seq'] <= old['observation_seq'] or d['stamp_ns'] <= old['stamp_ns']:
                    raise ValueError('observation replay')
                if (d['event_seq'] < old['event_seq'] or d['imu_seq'] < old['imu_seq'] or
                        d['loss_seq'] < old['loss_seq']):
                    raise ValueError('source replay')
                if not new and any(d[k] != old[k] for k in ('raw', 'event_time', 'receipt_ns')):
                    raise ValueError('cache identity changed')
                if not new and d['event_age_s'] + 1e-6 < old['event_age_s']:
                    raise ValueError('event age refreshed')
                if new and d['event_time'] <= old['event_time']:
                    raise ValueError('event clock order')
            valid = d['state'] in ('GENUINE_FRESH_INPUT', 'VALID_HELD_INPUT')
            if valid and (raw is None or d['event_seq'] < 1 or d['event_time'] <= 0 or
                          not 0 < d['receipt_ns'] <= d['stamp_ns'] or d['event_age_s'] < 0 or
                          d['imu_seq'] < 1 or not 0 <= d['imu_age_s'] + age < .25 or
                          not d['pose'] or not d['optical']):
                raise ValueError('invalid live evidence')
            # Recovery verification consumes advancing device observations, not timer callbacks.
            advancing = valid and (not same or d['imu_seq'] > old['imu_seq'])
            self.events += bool(new and d['event_seq'])
            self.last = dict(d)
            d.update(valid=valid, new_event=bool(new and valid and d['event_age_s'] + age < .25),
                     advancing=advancing, identity_changed=changed,
                     continuity_changed=bool(same and d['loss_seq'] != old['loss_seq']),
                     sample_at=d['event_time'], rejected_samples=self.rejected,
                     genuine_events_received=self.events)
            return d
        except (ValueError, TypeError, KeyError, OverflowError) as exc:
            self.rejected += 1
            return dict(valid=False, state='TEMPORARILY_UNVERIFIED',
                        rejection=str(exc), rejected_samples=self.rejected,
                        genuine_events_received=self.events)
