"""Conservative HTTP Date-to-monotonic clock mapping for restart time recovery."""
from __future__ import annotations

import datetime
from email.utils import parsedate_to_datetime
import math


KST = datetime.timezone(datetime.timedelta(hours=9))


class ClockSync:
    def __init__(self):
        self.offset = None
        self.uncertainty_s = None

    def observe(self, date_header, sent_mono, received_mono):
        try:
            server = parsedate_to_datetime(str(date_header))
            if server.tzinfo is None:
                return False
            rtt = float(received_mono)-float(sent_mono)
            if not math.isfinite(rtt) or not 0 <= rtt <= 5.:
                return False
            offset = server.timestamp()-(sent_mono+received_mono)/2
        except (ValueError, TypeError, OverflowError):
            return False
        self.offset = offset
        self.uncertainty_s = 1. + rtt/2  # HTTP Date and issued_at both have whole-second precision.
        return True

    def elapsed(self, issued_at, now_mono):
        if self.offset is None:
            return None
        try:
            issued = datetime.datetime.strptime(str(issued_at), "%Y-%m-%d %H:%M:%S").replace(tzinfo=KST)
            elapsed = float(now_mono)+self.offset-issued.timestamp()
        except (ValueError, TypeError, OverflowError):
            return None
        return elapsed if math.isfinite(elapsed) and 0 <= elapsed <= 300 else None

    @staticmethod
    def publication_delta(order_issued_at, target_issued_at):
        try:
            start = datetime.datetime.strptime(str(order_issued_at), "%Y-%m-%d %H:%M:%S").replace(tzinfo=KST)
            target = datetime.datetime.strptime(str(target_issued_at), "%Y-%m-%d %H:%M:%S").replace(tzinfo=KST)
            return (target-start).total_seconds()
        except (ValueError, TypeError, OverflowError):
            return None
