#!/usr/bin/env python3
import os
import re
import subprocess
import rospy
import psutil
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from mavros_msgs.msg import TimesyncStatus

# Jetson Nano sysfs paths
GPU_LOAD_PATH = '/sys/devices/gpu.0/load'
THERMAL_ZONES = '/sys/devices/virtual/thermal/thermal_zone{}/temp'

# Timesync thresholds (milliseconds)
CHRONY_WARN_MS = 10.0
CHRONY_ERROR_MS = 50.0
MAVROS_WARN_MS = 20.0
MAVROS_ERROR_MS = 100.0

# Per-process CPU thresholds (% of one core, psutil convention)
PROC_TOP_N = 10
PROC_MIN_CPU = 0.5
PROC_WARN_TOTAL = 250.0   # >2.5 cores worth of activity from tracked procs
PROC_ERROR_TOTAL = 350.0  # ~all 4 cores saturated by tracked procs


def read_sysfs(path):
    try:
        with open(path, 'r') as f:
            return f.read().strip()
    except (IOError, OSError):
        return None


def find_thermal_zones():
    cpu_zone = None
    gpu_zone = None
    for i in range(8):
        type_path = '/sys/devices/virtual/thermal/thermal_zone{}/type'.format(i)
        zone_type = read_sysfs(type_path)
        if zone_type is None:
            continue
        zone_type = zone_type.lower()
        if 'cpu' in zone_type or zone_type == 'cpu-therm':
            cpu_zone = i
        elif 'gpu' in zone_type or zone_type == 'gpu-therm':
            gpu_zone = i
    return cpu_zone, gpu_zone


def get_gpu_percent():
    raw = read_sysfs(GPU_LOAD_PATH)
    if raw is not None:
        try:
            return float(raw) / 10.0
        except ValueError:
            pass
    return -1.0


def get_temp(zone_id):
    if zone_id is None:
        return -1.0
    raw = read_sysfs(THERMAL_ZONES.format(zone_id))
    if raw is not None:
        try:
            return float(raw) / 1000.0
        except ValueError:
            pass
    return -1.0


def get_chrony_status():
    status = DiagnosticStatus()
    status.name = 'Timesync: Chrony (Jetson-Laptop)'
    status.hardware_id = 'jetson_chrony'

    try:
        tracking = subprocess.check_output(
            ['chronyc', '-n', 'tracking'],
            timeout=2,
            stderr=subprocess.STDOUT
        ).decode('utf-8')

        offset_match = re.search(r'Last offset\s*:\s*([+-]?\d+\.?\d*e?[+-]?\d*)', tracking)
        leap_match = re.search(r'Leap status\s*:\s*(.+)', tracking)
        stratum_match = re.search(r'Stratum\s*:\s*(\d+)', tracking)
        ref_match = re.search(r'Reference ID\s*:\s*\S+\s*\((.+?)\)', tracking)
        freq_match = re.search(r'Frequency\s*:\s*([+-]?\d+\.?\d*)\s*ppm', tracking)

        if offset_match:
            offset_s = float(offset_match.group(1))
            offset_ms = abs(offset_s) * 1000.0

            status.values.append(KeyValue(key='Offset ms', value='{:.3f}'.format(offset_s * 1000)))
            status.values.append(KeyValue(key='Abs Offset ms', value='{:.3f}'.format(offset_ms)))

            if offset_ms < CHRONY_WARN_MS:
                status.level = DiagnosticStatus.OK
                status.message = 'Synced ({:.1f}ms)'.format(offset_ms)
            elif offset_ms < CHRONY_ERROR_MS:
                status.level = DiagnosticStatus.WARN
                status.message = 'Drift: {:.1f}ms'.format(offset_ms)
            else:
                status.level = DiagnosticStatus.ERROR
                status.message = 'LARGE OFFSET: {:.1f}ms - clock not synced'.format(offset_ms)
        else:
            status.level = DiagnosticStatus.WARN
            status.message = 'Could not parse chrony offset'

        if leap_match:
            leap = leap_match.group(1).strip()
            status.values.append(KeyValue(key='Leap Status', value=leap))
            if leap != 'Normal':
                status.level = DiagnosticStatus.ERROR
                status.message = 'Leap status: {}'.format(leap)

        if stratum_match:
            status.values.append(KeyValue(key='Stratum', value=stratum_match.group(1)))
        if ref_match:
            status.values.append(KeyValue(key='Reference', value=ref_match.group(1)))
        if freq_match:
            status.values.append(KeyValue(key='Freq Offset ppm', value=freq_match.group(1)))

    except subprocess.TimeoutExpired:
        status.level = DiagnosticStatus.ERROR
        status.message = 'chronyc timed out'
    except FileNotFoundError:
        status.level = DiagnosticStatus.ERROR
        status.message = 'chrony not installed'
    except Exception as e:
        status.level = DiagnosticStatus.ERROR
        status.message = 'chrony error: {}'.format(str(e))

    return status


class MavrosTimesyncMonitor:
    def __init__(self):
        self.estimated_offset_ns = None
        self.observed_offset_ns = None
        self.rtt_ms = None
        self.last_recv_time = None

        rospy.Subscriber('/mavros/timesync_status', TimesyncStatus, self._cb)

    def _cb(self, msg):
        self.estimated_offset_ns = msg.estimated_offset_ns
        self.observed_offset_ns = msg.observed_offset_ns
        self.rtt_ms = msg.round_trip_time_ms
        self.last_recv_time = rospy.Time.now()

    def get_status(self):
        status = DiagnosticStatus()
        status.name = 'Timesync: MAVROS (Jetson-PX4)'
        status.hardware_id = 'mavros_timesync'

        if self.last_recv_time is None:
            status.level = DiagnosticStatus.STALE
            status.message = 'No timesync messages received'
            return status

        age = (rospy.Time.now() - self.last_recv_time).to_sec()
        if age > 5.0:
            status.level = DiagnosticStatus.STALE
            status.message = 'No update for {:.1f}s'.format(age)
            status.values.append(KeyValue(key='Age s', value='{:.1f}'.format(age)))
            return status

        error_ms = abs(self.estimated_offset_ns - self.observed_offset_ns) / 1e6

        status.values.append(KeyValue(key='Estimated Offset ms',
                                      value='{:.3f}'.format(self.estimated_offset_ns / 1e6)))
        status.values.append(KeyValue(key='Observed Offset ms',
                                      value='{:.3f}'.format(self.observed_offset_ns / 1e6)))
        status.values.append(KeyValue(key='Filter Error ms',
                                      value='{:.3f}'.format(error_ms)))
        status.values.append(KeyValue(key='RTT ms',
                                      value='{:.1f}'.format(self.rtt_ms)))

        if error_ms < MAVROS_WARN_MS:
            status.level = DiagnosticStatus.OK
            status.message = 'Converged (err: {:.1f}ms, RTT: {:.0f}ms)'.format(
                error_ms, self.rtt_ms)
        elif error_ms < MAVROS_ERROR_MS:
            status.level = DiagnosticStatus.WARN
            status.message = 'Converging: {:.1f}ms error'.format(error_ms)
        else:
            status.level = DiagnosticStatus.ERROR
            status.message = 'NOT CONVERGED: {:.0f}ms error - DO NOT ARM'.format(error_ms)

        return status


class ProcessMonitor:
    """Tracks per-process CPU on the Jetson and reports the top consumers.

    psutil.Process.cpu_percent() is delta-based: the first call returns 0.0,
    and subsequent calls compare against the previous sample. So we have to
    keep Process objects alive between iterations rather than re-creating
    them via process_iter() each cycle.

    Output values are per-core (top convention): a single-threaded process
    pinning one CPU reads 100%, a 4-thread process saturating all cores
    reads 400%. The Jetson Nano has 4 cores total.
    """

    def __init__(self, top_n=PROC_TOP_N, min_cpu=PROC_MIN_CPU):
        self.top_n = top_n
        self.min_cpu = min_cpu
        self.tracked = {}        # pid -> psutil.Process
        self.name_cache = {}     # pid -> friendly name (cmdline doesn't change)
        self._self_pid = os.getpid()
        self._refresh_processes()  # prime first sample

    def _friendly_name(self, proc):
        if proc.pid in self.name_cache:
            return self.name_cache[proc.pid]
        try:
            cmdline = proc.cmdline()
            if not cmdline:
                name = proc.name()
            else:
                base = os.path.basename(cmdline[0])
                # Python scripts: find the .py file or __name:= override
                if base.startswith('python'):
                    chosen = None
                    for arg in cmdline[1:]:
                        if '__name:=' in arg:
                            chosen = arg.split('__name:=')[1]
                            break
                        if arg.endswith('.py'):
                            chosen = os.path.basename(arg)[:-3]
                            break
                    name = chosen if chosen else base
                else:
                    # Compiled nodes (mavros_node, aruco_ros, nodelet, etc.)
                    # Check for __name:= override (common with nodelets)
                    chosen = base
                    for arg in cmdline[1:]:
                        if '__name:=' in arg:
                            chosen = '{}({})'.format(base, arg.split('__name:=')[1])
                            break
                    name = chosen
        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
            name = 'pid_{}'.format(proc.pid)

        self.name_cache[proc.pid] = name
        return name

    def _refresh_processes(self):
        # Add new pids, drop dead ones
        seen = set()
        for p in psutil.process_iter(['pid']):
            try:
                pid = p.pid
                seen.add(pid)
                if pid not in self.tracked:
                    self.tracked[pid] = p
                    p.cpu_percent(interval=None)  # prime
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                pass

        dead = set(self.tracked.keys()) - seen
        for pid in dead:
            self.tracked.pop(pid, None)
            self.name_cache.pop(pid, None)

    def get_status(self):
        self._refresh_processes()

        results = []
        for pid, proc in list(self.tracked.items()):
            if pid == self._self_pid:
                continue  # skip ourselves to avoid self-measurement bias
            try:
                cpu = proc.cpu_percent(interval=None)
                if cpu >= self.min_cpu:
                    results.append((self._friendly_name(proc), cpu, pid))
            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                self.tracked.pop(pid, None)
                self.name_cache.pop(pid, None)

        results.sort(key=lambda x: x[1], reverse=True)
        top = results[:self.top_n]
        total = sum(c for _, c, _ in results)

        status = DiagnosticStatus()
        status.name = 'Per-Process CPU'
        status.hardware_id = 'jetson_nano_proc'

        if total > PROC_ERROR_TOTAL:
            status.level = DiagnosticStatus.ERROR
            status.message = 'Cores saturated: {:.0f}% summed across cores'.format(total)
        elif total > PROC_WARN_TOTAL:
            status.level = DiagnosticStatus.WARN
            status.message = 'High load: {:.0f}%'.format(total)
        else:
            status.level = DiagnosticStatus.OK
            top3 = ', '.join('{}={:.0f}%'.format(n, c) for n, c, _ in top[:3])
            status.message = 'Top: {}'.format(top3) if top3 else 'idle'

        status.values.append(KeyValue(key='Total tracked %', value='{:.1f}'.format(total)))
        status.values.append(KeyValue(key='Process count', value=str(len(results))))
        for name, cpu, pid in top:
            status.values.append(KeyValue(
                key='{} [{}]'.format(name, pid),
                value='{:.1f}'.format(cpu)
            ))

        return status


def main():
    rospy.init_node('system_monitor', anonymous=False)
    pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=1)
    rate = rospy.Rate(rospy.get_param('~rate', 1))

    cpu_zone, gpu_zone = find_thermal_zones()
    if cpu_zone is not None:
        rospy.loginfo('CPU thermal zone: thermal_zone%d', cpu_zone)
    else:
        rospy.logwarn('Could not identify CPU thermal zone')
    if gpu_zone is not None:
        rospy.loginfo('GPU thermal zone: thermal_zone%d', gpu_zone)
    else:
        rospy.logwarn('Could not identify GPU thermal zone')

    mavros_timesync = MavrosTimesyncMonitor()
    rospy.loginfo('MAVROS timesync monitor started')

    proc_monitor = ProcessMonitor()
    rospy.loginfo('Process monitor started (tracking %d processes)',
                  len(proc_monitor.tracked))

    psutil.cpu_percent(interval=None)  # prime system-wide

    while not rospy.is_shutdown():
        cpu_pct = psutil.cpu_percent(interval=None)
        ram_pct = psutil.virtual_memory().percent
        gpu_pct = get_gpu_percent()
        cpu_temp = get_temp(cpu_zone)
        gpu_temp = get_temp(gpu_zone)

        level = DiagnosticStatus.OK
        message = 'OK'
        if cpu_temp > 80 or gpu_temp > 80:
            level = DiagnosticStatus.ERROR
            message = 'THERMAL CRITICAL'
        elif cpu_temp > 65 or gpu_temp > 65:
            level = DiagnosticStatus.WARN
            message = 'THERMAL WARNING'
        elif cpu_pct > 90 or ram_pct > 90:
            level = DiagnosticStatus.WARN
            message = 'HIGH LOAD'

        system_status = DiagnosticStatus()
        system_status.level = level
        system_status.name = 'Jetson System Monitor'
        system_status.message = message
        system_status.hardware_id = 'jetson_nano'
        system_status.values = [
            KeyValue(key='CPU %', value='{:.1f}'.format(cpu_pct)),
            KeyValue(key='RAM %', value='{:.1f}'.format(ram_pct)),
            KeyValue(key='GPU %', value='{:.1f}'.format(gpu_pct)),
            KeyValue(key='CPU Temp C', value='{:.1f}'.format(cpu_temp)),
            KeyValue(key='GPU Temp C', value='{:.1f}'.format(gpu_temp)),
        ]

        chrony_status = get_chrony_status()
        mavros_status = mavros_timesync.get_status()
        proc_status = proc_monitor.get_status()

        msg = DiagnosticArray()
        msg.header.stamp = rospy.Time.now()
        msg.status = [system_status, chrony_status, mavros_status, proc_status]
        pub.publish(msg)

        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
