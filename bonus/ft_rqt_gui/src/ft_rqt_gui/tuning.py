"""Bounded dynamic clients and a reversible, per-side prediction horizon update."""

from dataclasses import dataclass
import math
import socket
import threading

from dynamic_reconfigure.client import Client
from dynamic_reconfigure.srv import Reconfigure
import rospy
from rospy.impl.tcpros_base import TCPROSTransport

from .processes import master_call


class _TimedService(rospy.ServiceProxy):
    """固定服务 URI，限制连接和响应时间，防止旧会话写入替代节点。"""

    def __init__(self, name, uri):
        super().__init__(name, Reconfigure, persistent=True)
        self.uri = uri
        self._call_lock = threading.Lock()

    def call(self, *args, **kwargs):
        with self._call_lock:
            transport = TCPROSTransport(self.protocol, self.resolved_name)
            timer = None
            try:
                address, port = rospy.core.parse_rosrpc_uri(self.uri)
                transport.connect(address, port, self.uri, timeout=2.0)
                self.transport = transport
                sock = transport.socket

                def interrupt():
                    try:
                        sock.shutdown(socket.SHUT_RDWR)
                    except OSError:
                        pass
                    transport.close()

                # receive_once 会恢复阻塞模式，因此关闭本次 socket 限制响应时间。
                timer = threading.Timer(3.0, interrupt)
                timer.daemon = True
                timer.start()
                return super().call(*args, **kwargs)
            finally:
                if timer:
                    timer.cancel()
                transport.close()
                self.transport = None


class TuningClient(Client):
    def _get_service_proxy(self, suffix, timeout):
        name = rospy.resolve_name(self.name + '/' + suffix)
        return _TimedService(name, master_call('lookupService', name))


@dataclass
class Endpoint:
    client: TuningClient
    fields: dict


class HorizonError(RuntimeError):
    def __init__(self, message, inconsistent=False):
        super().__init__(message)
        self.inconsistent = inconsistent


def connect_endpoint(server):
    """实际读取参数描述；旧版本缺少服务时由界面显示不支持。"""
    client = TuningClient(server, timeout=2.0)
    try:
        if client.get_configuration(timeout=2.0) is None:
            raise RuntimeError('No parameter configuration received: ' + server)
        fields = {field['name']: field for field in client.get_parameter_descriptions(timeout=2.0)}
        return Endpoint(client, fields)
    except Exception:
        client.close()
        raise


def apply_horizon(endpoints, value, cancelled):
    """先读取两端旧值，再逐个更新；失败或取消时按相反顺序恢复。"""
    if not math.isfinite(value) or value < 0.0:
        raise HorizonError('Prediction horizon must be finite and nonnegative.')
    previous, attempted = [], []

    def check_cancelled():
        if cancelled.is_set():
            raise RuntimeError('The parameter session changed during the update.')

    try:
        for endpoint in endpoints:
            check_cancelled()
            field = endpoint.fields['prediction_time_s']
            if not field['min'] <= value <= field['max']:
                raise ValueError('Prediction horizon is outside this node\'s supported range.')
            config = endpoint.client.update_configuration({})
            previous.append((endpoint, config['prediction_time_s']))
        for endpoint, old in previous:
            check_cancelled()
            attempted.append((endpoint, old))  # 超时也可能已经生效，需要尝试恢复。
            accepted = endpoint.client.update_configuration({'prediction_time_s': value})
            if not math.isclose(accepted['prediction_time_s'], value, abs_tol=1e-9):
                raise RuntimeError('A node rejected the prediction horizon.')
        check_cancelled()
    except Exception as error:
        failures = []
        for endpoint, old in reversed(attempted):
            try:
                restored = endpoint.client.update_configuration({'prediction_time_s': old})
                if not math.isclose(restored['prediction_time_s'], old, abs_tol=1e-9):
                    raise RuntimeError('The previous value was rejected.')
            except Exception:
                failures.append(endpoint.client.name)
        if failures:
            raise HorizonError('Horizon update failed and rollback could not be confirmed for ' +
                               ', '.join(failures) + '. Restart this session.', True) from error
        raise HorizonError('Horizon update failed; previous values retained: ' + str(error)) from error
