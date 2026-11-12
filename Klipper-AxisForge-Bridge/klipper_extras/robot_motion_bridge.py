"""Experimental, read-only motion export for a ROS 2 follower.

Install this file as ``klippy/extras/robot_motion_bridge.py`` and enable it
with a ``[robot_motion_bridge]`` section in printer.cfg.
"""

MAX_PULL_MOVES = 128
NEVER_TIME = 9999999999999999.0
BATCH_INTERVAL = 0.100


def encode_move(move):
    """Convert a Klipper pull_move into a JSON-compatible record."""
    return {
        "print_time": move.print_time,
        "duration": move.move_t,
        "start_velocity": move.start_v,
        "acceleration": move.accel,
        "start_position": [move.start_x, move.start_y, move.start_z],
        "direction": [move.x_r, move.y_r, move.z_r],
    }


class TrapQStream:
    def __init__(self, trapq, chelper):
        self.trapq = trapq
        self.chelper = chelper
        self.last_batch = (0.0, 0.0)

    def reset(self, print_time):
        self.last_batch = (print_time, 0.0)

    def _extract(self, start_time):
        ffi_main, ffi_lib = self.chelper.get_ffi()
        chunks = []
        end_time = NEVER_TIME
        while True:
            data = ffi_main.new("struct pull_move[%d]" % (MAX_PULL_MOVES,))
            count = ffi_lib.trapq_extract_old(
                self.trapq, data, len(data), start_time, end_time
            )
            if not count:
                break
            chunks.append([data[index] for index in range(count)])
            if count < len(data):
                break
            end_time = data[count - 1].print_time
        chunks.reverse()
        return [move for chunk in chunks for move in reversed(chunk)]

    def pull(self, safe_until):
        last_time, last_duration = self.last_batch
        start_time = last_time + min(last_duration, 0.1)
        moves = self._extract(start_time)
        records = []
        for move in moves:
            if safe_until is not None and move.print_time + move.move_t >= safe_until:
                break
            record = encode_move(move)
            if (record["print_time"], record["duration"]) == self.last_batch:
                continue
            records.append(record)
        if records:
            last = records[-1]
            self.last_batch = (last["print_time"], last["duration"])
        return records


class RobotMotionBridge:
    """Stream retained queued and historical toolhead and extruder moves."""

    def __init__(self, config):
        self.printer = config.get_printer()
        self.streams = {}
        self.sequence = 0
        self.motion_queuing = None

        import chelper
        from . import bulk_sensor

        self.chelper = chelper
        self.batch = bulk_sensor.BatchBulkHelper(
            self.printer,
            self._process_batch,
            start_cb=self._start_stream,
            batch_interval=BATCH_INTERVAL,
        )
        self.batch.add_mux_endpoint(
            "robot_motion/stream",
            "name",
            "all",
            {"header": ("protocol", "sequence", "clock", "streams")},
        )
        self.printer.register_event_handler("klippy:connect", self._connect)

    def _connect(self):
        toolhead = self.printer.lookup_object("toolhead")
        self.motion_queuing = self.printer.lookup_object("motion_queuing")
        self.streams["toolhead"] = TrapQStream(toolhead.get_trapq(), self.chelper)
        for index in range(99):
            name = "extruder%d" % (index,)
            if name == "extruder0":
                name = "extruder"
            extruder = self.printer.lookup_object(name, None)
            if extruder is None:
                break
            self.streams[name] = TrapQStream(extruder.get_trapq(), self.chelper)

    def _start_stream(self):
        if not self.streams:
            return
        reactor = self.printer.get_reactor()
        mcu = self.printer.lookup_object("mcu")
        print_time = mcu.estimated_print_time(reactor.monotonic())
        for stream in self.streams.values():
            stream.reset(print_time)
        self.sequence = 0

    def _process_batch(self, eventtime):
        if not self.streams or self.motion_queuing is None:
            return {}
        safe_until = self.motion_queuing.check_drip_timing()
        data = {}
        latest_time = None
        for name, stream in self.streams.items():
            records = stream.pull(safe_until)
            if records:
                data[name] = records
                last = records[-1]
                segment_end = last["print_time"] + last["duration"]
                latest_time = max(latest_time or segment_end, segment_end)
        if not data:
            return {}
        self.sequence += 1
        return {
            "protocol": 1,
            "sequence": self.sequence,
            "clock": "klipper_print_time",
            "latest_motion_time": latest_time,
            "streams": data,
        }

    def get_status(self, eventtime):
        return {
            "protocol": 1,
            "clock": "klipper_print_time",
            "queues": sorted(self.streams),
            "streaming": self.batch.is_started,
            "control_ready": False,
        }


def load_config(config):
    return RobotMotionBridge(config)
