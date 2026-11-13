"""Load a ROS node script without ROS or robot hardware: stand-in modules for rospy, tf, geometry_msgs, std_msgs,
nav_msgs, roslib, serial, smbus and playground.srv are put in sys.modules, the script is imported under a private name,
and its callbacks can be called with a fake message. Recorded calls (serial writes, motor commands) are kept in
`calls`. Used by the tests only.
"""
import importlib.util
import sys
import types
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent


class Recorder:
    def __init__(self, name, calls):
        self._name, self._calls = name, calls

    def __getattr__(self, attr):
        def call(*a, **k):
            self._calls.append((self._name + "." + attr,) + a)
            return 0
        return call


class Anything:
    """Message stand-in: any attribute can be read (a new Anything) or set."""
    def __getattr__(self, k):
        v = Anything()
        object.__setattr__(self, k, v)
        return v


class Vec:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = x, y, z


class Twist:
    def __init__(self, x=0.0, z=0.0):
        self.linear, self.angular = Vec(x=x), Vec(z=z)


def load(rel_path, calls):
    msg = types.ModuleType("msg")
    for n in ("Twist", "Quaternion", "Int16", "Int32", "Int64", "Float32", "String", "Header", "UInt64", "Odometry",
              "Imu", "PoseWithCovarianceStamped"):
        setattr(msg, n, type(n, (Anything,), {}))
    mods = {"rospy": Recorder("rospy", calls), "roslib": Recorder("roslib", calls),
            "tf": types.ModuleType("tf"), "tf.transformations": types.ModuleType("tf.transformations"),
            "geometry_msgs": types.ModuleType("geometry_msgs"), "geometry_msgs.msg": msg,
            "std_msgs": types.ModuleType("std_msgs"), "std_msgs.msg": msg,
            "nav_msgs": types.ModuleType("nav_msgs"), "nav_msgs.msg": msg,
            "sensor_msgs": types.ModuleType("sensor_msgs"), "sensor_msgs.msg": msg,
            "playground": types.ModuleType("playground"), "playground.srv": types.ModuleType("playground.srv"),
            "serial": types.SimpleNamespace(Serial=lambda *a, **k: Recorder("ser", calls)),
            "smbus": types.SimpleNamespace(SMBus=lambda *a: Recorder("bus", calls))}
    saved = {k: sys.modules.get(k) for k in mods}
    sys.modules.update(mods)
    sys.path.insert(0, str(ROOT))
    try:
        spec = importlib.util.spec_from_file_location("_node_" + Path(rel_path).stem, ROOT / rel_path)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
    finally:
        sys.path.remove(str(ROOT))
        for k, v in saved.items():
            if v is None:
                sys.modules.pop(k, None)
            else:
                sys.modules[k] = v
    return mod
