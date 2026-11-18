"""Read a small allowlist of effective Nav2 parameters, never the whole parameter tree."""
import hashlib
import json
import math
import time
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import ParameterType


def value_of(value):
    fields = {ParameterType.PARAMETER_BOOL: 'bool_value', ParameterType.PARAMETER_INTEGER: 'integer_value',
              ParameterType.PARAMETER_DOUBLE: 'double_value', ParameterType.PARAMETER_STRING: 'string_value',
              ParameterType.PARAMETER_STRING_ARRAY: 'string_array_value',
              ParameterType.PARAMETER_DOUBLE_ARRAY: 'double_array_value'}
    field = fields.get(value.type)
    if not field:
        return None
    value = getattr(value, field)
    if field.endswith('_array_value'):
        return [item if not isinstance(item, float) or math.isfinite(item) else None for item in value]
    return value if not isinstance(value, float) or math.isfinite(value) else None


class RuntimeProfile:
    PARAMETERS = {
        'planner_server': ['planner_plugins'],
        'controller_server': ['controller_plugins', 'controller_frequency', 'odom_topic'],
        'amcl': ['base_frame_id', 'odom_frame_id', 'scan_topic', 'update_min_d', 'update_min_a'],
        'map_server': ['yaml_filename'],
        'local_costmap/local_costmap': ['robot_base_frame', 'robot_radius', 'footprint', 'resolution'],
        'global_costmap/global_costmap': ['robot_base_frame', 'robot_radius', 'footprint', 'resolution'],
    }

    def __init__(self, node):
        self.clients = {name: node.create_client(GetParameters, f'/{name}/get_parameters') for name in self.PARAMETERS}
        self.reset()

    def reset(self):
        self.generation = getattr(self, 'generation', 0)+1
        for name, (future, _) in getattr(self, 'pending', {}).items():
            if not future.done():
                self.clients[name].remove_pending_request(future)
                future.cancel()
        self.pending = {}
        self.values = {}
        self.observed = {}
        self.polled_at = 0.

    def poll(self):
        now = time.monotonic()
        if now-self.polled_at < 3:
            return
        self.polled_at = now
        for name, client in self.clients.items():
            if name in self.pending:
                old, sent = self.pending[name]
                if not old.done():
                    if now-sent < 2:
                        continue
                    client.remove_pending_request(old)
                    old.cancel()
            if not client.service_is_ready():
                continue
            keys = list(self.PARAMETERS[name])
            for kind in ('planner_plugins', 'controller_plugins'):
                aliases = self.values.get(name, {}).get(kind, [])
                for alias in aliases if isinstance(aliases, list) else []:
                    if not isinstance(alias, str) or not alias or len(alias)>100:
                        continue
                    keys.append(alias+'.plugin')
                    if kind == 'controller_plugins':
                        keys += [alias+'.trans_stopped_velocity', alias+'.max_vel_x']
            future = client.call_async(GetParameters.Request(names=keys))
            self.pending[name] = (future, now)
            generation = self.generation
            def done(result, key=name, names=keys, epoch=generation):
                if epoch != self.generation:
                    return
                try:
                    self.values[key] = dict(zip(names, map(value_of, result.result().values)))
                    self.observed[key] = time.monotonic()
                except Exception:
                    pass
            future.add_done_callback(done)

    def snapshot(self):
        now = time.monotonic()
        serialized = json.dumps(self.values, sort_keys=True, ensure_ascii=False, allow_nan=False)
        return dict(source='ROS GetParameters', parameters=self.values,
                    sha256=hashlib.sha256(serialized.encode()).hexdigest() if self.values else None,
                    fresh=bool(self.observed) and all(now-t <= 10 for t in self.observed.values()),
                    observedNodes=list(self.observed), expectedNodes=list(self.PARAMETERS),
                    complete=len(self.observed) == len(self.PARAMETERS),
                    firmwareVerified=False)
