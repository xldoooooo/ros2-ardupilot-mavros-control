#!/usr/bin/env python3
"""Select the optional planner process without loading FPGA dependencies in CPU mode."""

import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_parameter_events, qos_profile_sensor_data)
from rcl_interfaces.msg import Parameter, ParameterEvent, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from std_msgs.msg import String
from std_srvs.srv import SetBool
from guided_interfaces.msg import ControlStatus


class BackendManager(Node):
    """Keep CPU running and commit a source change only after explicit readiness."""

    def __init__(self):
        """Connect only standard ROS management interfaces until FPGA is requested."""
        super().__init__('avoidance_backend_manager')
        root = next((path for path in Path(__file__).resolve().parents
                     if (path / 'scripts/onboard').is_dir()), Path.cwd())
        root = Path(self.declare_parameter('project_root', str(root)).value)
        self._entrypoint = Path(self.declare_parameter(
            'module_entrypoint', str(root / 'fpga_avoidance/start.sh')).value)
        self._frame = self.declare_parameter('planning_frame', 'odom').value
        self._cloud = self.declare_parameter('cloud_topic', '/odin1/cloud_slam_down').value
        self._body_twist = self.declare_parameter('odometry_twist_in_body', True).value
        self._lower = list(self.declare_parameter('search_lower', [-20.0, -20.0, 0.2]).value)
        self._upper = list(self.declare_parameter('search_upper', [20.0, 20.0, 1.8]).value)
        self._startup_timeout = float(self.declare_parameter('startup_timeout', 12.0).value)
        self._status_timeout = float(self.declare_parameter('status_timeout', 3.0).value)
        self._state_lock = threading.RLock()
        self._selection_lock = threading.Lock()
        self._process = None
        self._backend = 'cpu'
        self._source_confirmed = False
        self._source_revision = 0
        self._source_read_pending = False
        self._source_read_future = None
        self._source_read_started = 0.0
        self._error = ''
        self._transitioning = False
        self._module_status = {}
        self._module_time = 0.0
        self._control_status = None
        self._control_time = 0.0
        group = ReentrantCallbackGroup()
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._publisher = self.create_publisher(String, '/avoidance/backend/status', latched)
        self.create_subscription(ControlStatus, '/onboard_control/status',
                                 self._on_control, qos_profile_sensor_data)
        self.create_subscription(String, '/fpga_avoidance/status', self._on_module, latched)
        self.create_subscription(ParameterEvent, '/parameter_events', self._on_bridge_parameters,
                                 qos_profile_parameter_events)
        self._bridge_set = self.create_client(SetParameters, '/avoidance_bridge/set_parameters',
                                             callback_group=group)
        self._bridge_get = self.create_client(GetParameters, '/avoidance_bridge/get_parameters',
                                             callback_group=group)
        self.create_service(SetBool, '/avoidance/backend/select', self._select,
                            callback_group=group)
        self.create_timer(1.0, self._publish)
        self._publish()

    def _on_control(self, message):
        """Record the authority's landed/task state using reception time."""
        with self._state_lock:
            self._control_status = message
            self._control_time = time.monotonic()

    def _on_module(self, message):
        """Accept diagnostics only from the process started by this manager."""
        try:
            status = json.loads(message.data)
        except (ValueError, TypeError):
            return
        with self._state_lock:
            if (not isinstance(status, dict) or self._process is None or
                    status.get('process_id') != self._process.pid):
                return
            self._module_status = status
            self._module_time = time.monotonic()

    def _on_bridge_parameters(self, event):
        """Follow bridge restarts and stop an owned backend only after it is unselected."""
        if event.node != '/avoidance_bridge':
            return
        for parameter in [*event.new_parameters, *event.changed_parameters]:
            if (parameter.name != 'native_namespace' or
                    parameter.value.type != ParameterType.PARAMETER_STRING):
                continue
            namespace = parameter.value.string_value
            if namespace not in ('/avoidance', '/fpga_avoidance'):
                continue
            with self._state_lock:
                previously_fpga = self._backend == 'fpga'
                self._source_revision += 1
                self._record_source(namespace)
            if previously_fpga and namespace == '/avoidance':
                self._stop_process()
            self._publish()

    def _gate_error(self):
        """Permit a source change only with recent disarmed, landed, idle evidence."""
        with self._state_lock:
            status = self._control_status
            if status is None or time.monotonic() - self._control_time > self._status_timeout:
                return 'fresh onboard control status unavailable'
            if (status.armed or not status.on_ground or status.controller_active or
                    status.reboot_in_progress or status.waypoint_count != 0 or
                    status.control_mode != ControlStatus.MODE_IDLE):
                return 'backend selection requires disarmed, landed, idle state without a task'
        return ''

    def _health_error(self):
        """Keep a failed selected backend visible without changing the flight source."""
        with self._state_lock:
            if self._process is None or self._process.poll() is not None:
                return 'FPGA process is not running'
            if time.monotonic() - self._module_time > self._status_timeout:
                return 'FPGA status unavailable or stale'
            status = self._module_status
            if status.get('mock') or str(status.get('source', '')).lower().startswith('mock'):
                return 'MOCK backend is unavailable for flight selection'
            if not status.get('ready', False):
                return str(status.get('error') or 'FPGA backend is not ready')
        return ''

    def _cpu_error(self):
        """Report the controller's current CPU planner readiness without adding a gate."""
        with self._state_lock:
            status = self._control_status
            if status is None or time.monotonic() - self._control_time > self._status_timeout:
                return 'fresh onboard control status unavailable'
            if not status.avoidance_ready:
                return str(status.avoidance_detail or 'CPU avoidance planner is not ready')
        return ''

    def _publish(self):
        """Publish selection truth; CPU mode never opens the optional configuration."""
        if (self._source_read_pending and
                time.monotonic() - self._source_read_started > self._status_timeout):
            previous = self._source_read_future
            self._source_read_future = None
            self._source_read_pending = False
            self._source_confirmed = False
            previous.cancel()
        # A restarted bridge may declare its default before DDS matches parameter events.
        if (not self._source_read_pending and not self._transitioning and
                self._bridge_get.service_is_ready()):
            self._source_read_pending = True
            self._source_read_started = time.monotonic()
            source_revision = self._source_revision
            future = self._bridge_get.call_async(GetParameters.Request(names=['native_namespace']))
            self._source_read_future = future
            future.add_done_callback(lambda result: self._on_source_read(result, source_revision))
        with self._state_lock:
            backend_error = self._health_error() if self._backend == 'fpga' else self._cpu_error()
            status = {
                'backend': self._backend,
                'available': self._is_installed(),
                'ready': not backend_error and self._source_confirmed,
                'error': backend_error or self._error,
                'source': self._module_status.get('source', 'pending')
                          if self._backend == 'fpga' else 'cpu',
                'transitioning': self._transitioning,
                'source_confirmed': self._source_confirmed,
                'fpga_ready': self._process is not None and not self._health_error(),
            }
            if self._backend == 'fpga':
                for key in ('transactions', 'hardware_transactions', 'hardware_participation',
                            'solve_ms', 'cpu_ms', 'mock'):
                    if key in self._module_status:
                        status[key] = self._module_status[key]
        self._publisher.publish(String(data=json.dumps(status)))

    def _is_installed(self):
        """Inspect install markers only; CPU startup never reads FPGA configuration."""
        return (self._entrypoint.is_file() and os.access(self._entrypoint, os.X_OK) and
                (self._entrypoint.parent / '.installed').is_file())

    def _on_source_read(self, future, source_revision):
        """Reconcile missed restart events without applying replies from an old selection."""
        if future is not self._source_read_future:
            return
        try:
            with self._state_lock:
                if self._transitioning or source_revision != self._source_revision:
                    return
                previously_fpga = self._backend == 'fpga'
                self._accept_source(future.result())
                unselected = previously_fpga and self._backend == 'cpu'
            if unselected:
                self._stop_process()
        except Exception as exc:
            self._error = f'cannot read current bridge source: {exc}'
        finally:
            self._source_read_pending = False

    def _accept_source(self, result):
        """Record only the two supported authoritative bridge namespaces."""
        if result.values:
            self._record_source(result.values[0].string_value)

    def _record_source(self, namespace):
        """Apply a supported namespace received from the bridge's own interfaces."""
        with self._state_lock:
            if namespace in ('/avoidance', '/fpga_avoidance'):
                self._backend = 'fpga' if namespace == '/fpga_avoidance' else 'cpu'
                self._source_confirmed = True

    def _wait_future(self, future, timeout):
        """Wait outside the executor's other callback threads for an RPC result."""
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout):
            raise RuntimeError('avoidance bridge parameter response timed out')
        return future.result()

    def _set_namespace(self, namespace):
        """Commit through the bridge, which independently checks the landing gate."""
        error = self._gate_error()
        if error:
            raise RuntimeError(error)
        if not self._bridge_set.wait_for_service(timeout_sec=1.0):
            raise RuntimeError('avoidance bridge parameter service unavailable')
        parameter = Parameter(name='native_namespace', value=ParameterValue(
            type=ParameterType.PARAMETER_STRING, string_value=namespace))
        with self._state_lock:
            self._source_revision += 1
            source_revision = self._source_revision
            self._source_confirmed = False
        result = self._wait_future(self._bridge_set.call_async(
            SetParameters.Request(parameters=[parameter])), 3.0)
        self._source_confirmed = True
        if not result.results or not all(item.successful for item in result.results):
            raise RuntimeError('; '.join(item.reason for item in result.results) or
                               'avoidance bridge rejected planner selection')
        with self._state_lock:
            target = 'fpga' if namespace == '/fpga_avoidance' else 'cpu'
            if self._source_revision != source_revision and self._backend != target:
                raise RuntimeError('avoidance bridge source changed during selection')
            self._record_source(namespace)

    def _read_namespace(self):
        """Resolve source truth after a parameter RPC fails or loses its reply."""
        if not self._bridge_get.wait_for_service(timeout_sec=0.2):
            return
        result = self._wait_future(self._bridge_get.call_async(
            GetParameters.Request(names=['native_namespace'])), 1.0)
        self._accept_source(result)

    def _start_process(self):
        """Start only the optional entrypoint in its own process group."""
        if not self._is_installed():
            raise RuntimeError('FPGA module is not installed')
        with self._state_lock:
            self._module_status = {}
            self._module_time = 0.0
            self._process = subprocess.Popen([
                sys.executable, str(Path(__file__).with_name('avoidance_backend_process.py')),
                str(os.getpid()),
                str(self._entrypoint), '--ros-args', '-r', '__ns:=/fpga_avoidance',
                '-p', f'planning_frame:={self._frame}', '-p', f'cloud_topic:={self._cloud}',
                '-p', f'odometry_twist_in_body:={str(self._body_twist).lower()}',
                '-p', f'search_lower:={json.dumps(self._lower)}',
                '-p', f'search_upper:={json.dumps(self._upper)}',
            ], start_new_session=True)
        deadline = time.monotonic() + self._startup_timeout
        while time.monotonic() < deadline:
            error = self._gate_error()
            if error:
                raise RuntimeError(error)
            error = self._health_error()
            if not error:
                return
            if self._process.poll() is not None:
                raise RuntimeError(f'FPGA startup exited with code {self._process.returncode}')
            time.sleep(0.05)
        raise RuntimeError(self._health_error())

    def _stop_process(self):
        """Stop only this manager's isolated module process group."""
        with self._state_lock:
            process = self._process
            self._process = None
            self._module_status = {}
            self._module_time = 0.0
        if process is None or process.poll() is not None:
            return
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
            try:
                os.killpg(process.pid, sig)
            except ProcessLookupError:
                break
            try:
                process.wait(timeout=0.5)
            except subprocess.TimeoutExpired:
                continue
            # Solver workers may outlive their parent briefly; signal the owned group as well.
            try:
                os.killpg(process.pid, 0)
            except ProcessLookupError:
                break

    def _select(self, request, response):
        """Prepare the target before switching, preserving the CPU launch on errors."""
        if not self._selection_lock.acquire(blocking=False):
            response.message = 'backend selection is already in progress'
            return response
        try:
            if not self._source_confirmed:
                self._read_namespace()
                if not self._source_confirmed:
                    raise RuntimeError('current bridge source unavailable')
            target = 'fpga' if request.data else 'cpu'
            if target == self._backend:
                error = self._health_error() if target == 'fpga' else ''
                response.success = not error
                response.message = error or f'{target.upper()} backend already selected'
                return response
            error = self._gate_error()
            if error:
                raise RuntimeError(error)
            self._transitioning = True
            self._error = ''
            self._publish()
            if target == 'fpga':
                self._start_process()
            self._set_namespace('/fpga_avoidance' if request.data else '/avoidance')
            if target == 'cpu':
                self._stop_process()
            response.success = True
            response.message = f'{target.upper()} backend selected'
        except Exception as exc:
            self._error = str(exc)
            # A timed-out response does not prove that the bridge rejected the change.
            try:
                self._read_namespace()
            except Exception as read_error:
                self._error += f'; cannot confirm bridge source: {read_error}'
            if request.data and self._backend == 'fpga' and not self._gate_error():
                try:
                    self._set_namespace('/avoidance')
                except Exception as rollback_error:
                    self._error += f'; CPU restoration failed: {rollback_error}'
            if self._backend == 'cpu' and self._source_confirmed:
                self._stop_process()
            response.message = self._error
        finally:
            self._transitioning = False
            self._publish()
            self._selection_lock.release()
        return response


def main(args=None):
    """Use independent callback threads so readiness and parameter replies keep flowing."""
    rclpy.init(args=args)
    node = BackendManager()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown(timeout_sec=2.0)
        node._stop_process()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
