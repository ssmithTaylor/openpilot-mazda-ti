"""Windows test shim for openpilot-mazda-ti. Rebuilt 2026-08-25.

Junctions provide the real package dirs (common/selfdrive/frogpilot/system/third_party/tools,
plus cereal/opendbc at pyshim root). This file injects stand-ins for the compiled/native bits
that never build on Windows, BEFORE anything imports them. Anything that CALLS a stub raises
loudly; the controller suites only construct them.
"""
import sys
import types
import os

_pyshim = os.path.dirname(os.path.dirname(__file__))


def _mod(name):
    m = types.ModuleType(name)
    m.__dict__["__stub__"] = True
    sys.modules[name] = m
    return m


# --- opendbc compiled CAN parser/packer -> CANDefine + inert parser/packer ---
try:
    import opendbc.can  # real dir via junction
except Exception:
    pass


def _install_can_stubs():
    for modname in ("opendbc.can.parser_pyx", "opendbc.can.packer_pyx"):
        if modname in sys.modules:
            continue
        m = _mod(modname)

        class _CANParser:
            def __init__(self, *a, **k):
                self.vl = {}
                self.vl_all = {}
                self.ts_nanos = {}
                self.can_valid = True
                self.can_invalid_cnt = 0

            def update(self, *a, **k):
                return []

        class _CANPacker:
            def __init__(self, *a, **k):
                pass

            def make_can_msg(self, *a, **k):
                return [0, b"", 0]

        class _CANDefine:
            def __init__(self, *a, **k):
                self.dv = {}
                self.dv_by_addr = {}

        m.CANParser = _CANParser
        m.CANPacker = _CANPacker
        m.CANDefine = _CANDefine


_install_can_stubs()


# --- openpilot.common.params_pyx -> dict-backed Params ---
def _install_params_stub():
    modname = "openpilot.common.params_pyx"
    if modname in sys.modules:
        return
    m = _mod(modname)
    _store = {}

    class UnknownKeyName(Exception):
        pass

    class ParamKeyType:
        PERSISTENT = 1
        CLEAR_ON_MANAGER_START = 2
        CLEAR_ON_ONROAD_TRANSITION = 4
        CLEAR_ON_OFFROAD_TRANSITION = 8
        DEVELOPMENT_ONLY = 16
        CLEAR_ON_IGNITION_ON = 32
        ALL = 0xFFFFFFFF

    class Params:
        def __init__(self, d=None):
            pass

        def get(self, k, block=False, encoding=None):
            v = _store.get(k)
            if v is not None and encoding:
                return v.decode(encoding) if isinstance(v, bytes) else v
            return v

        def get_bool(self, k):
            return _store.get(k) in (b"1", "1", True)

        def put(self, k, v):
            _store[k] = v.encode() if isinstance(v, str) else v

        def put_bool(self, k, v):
            _store[k] = b"1" if v else b"0"

        def put_nonblocking(self, k, v):
            self.put(k, v)

        def put_bool_nonblocking(self, k, v):
            self.put_bool(k, v)

        def remove(self, k):
            _store.pop(k, None)

        def get_param_path(self, *a):
            return "/dev/null"

        def check_key(self, k):
            return True

    m.Params = Params
    m.UnknownKeyName = UnknownKeyName
    m.ParamKeyType = ParamKeyType


_install_params_stub()


# --- msgq.ipc_pyx -> constructible-but-inert Context/Poller/sockets ---
def _install_msgq_stub():
    # msgq/rednose/teleoprtc/tinygrad root symlinks are dead text on Windows; give them package
    # stubs so `import msgq...` resolves, then the compiled ipc lives under a submodule stub.
    for pkg in ("msgq", "rednose", "teleoprtc", "tinygrad"):
        if pkg not in sys.modules:
            p = _mod(pkg)
            p.__path__ = []
    # cereal.messaging imports these five straight off the msgq package
    _msgq = sys.modules["msgq"]
    _msgq.fake_event_handle = lambda *a, **k: None
    _msgq.pub_sock = lambda *a, **k: None
    _msgq.sub_sock = lambda *a, **k: None
    _msgq.drain_sock_raw = lambda *a, **k: []
    _msgq.context = None
    if "msgq.ipc_pyx" not in sys.modules:
        m = _mod("msgq.ipc_pyx")

        class _Sock:
            def __init__(self, *a, **k):
                pass

            def connect(self, *a, **k):
                pass

            def send(self, *a, **k):
                pass

            def receive(self, *a, **k):
                return None

            def setTimeout(self, *a, **k):
                pass

            def close(self, *a, **k):
                pass

        class Context:
            def __init__(self, *a, **k):
                pass

            def term(self):
                pass

        class Poller:
            def __init__(self, *a, **k):
                pass

            def registerSocket(self, *a, **k):
                pass

            def poll(self, *a, **k):
                return []

        class SubSocket(_Sock):
            pass

        class PubSocket(_Sock):
            pass

        class MessageBuilder:
            pass

        class SocketEventHandle:
            def __init__(self, *a, **k):
                pass

            def wait(self, *a, **k):
                return False

            def clear(self, *a, **k):
                pass

            def set(self, *a, **k):
                pass

        m.Context = Context
        m.Poller = Poller
        m.SubSocket = SubSocket
        m.PubSocket = PubSocket
        m.MessageBuilder = MessageBuilder
        m.SocketEventHandle = SocketEventHandle
        m.toggle_fake_events = lambda *a, **k: None
        m.set_fake_prefix = lambda *a, **k: None
        m.get_fake_prefix = lambda *a, **k: None
        m.delete_fake_prefix = lambda *a, **k: None
        m.wait_for_one_event = lambda *a, **k: None
        m.IpcError = RuntimeError
        m.MultiplePublishersError = RuntimeError


_install_msgq_stub()


# --- openpilot.common.transformations.transformations -> named raisers + LocalCoord ---
def _install_transformations_stub():
    modname = "openpilot.common.transformations.transformations"
    if modname in sys.modules:
        return
    m = _mod(modname)

    class LocalCoord:
        def __init__(self, *a, **k):
            raise NotImplementedError("LocalCoord is stubbed on Windows")

        @classmethod
        def from_geodetic(cls, *a, **k):
            raise NotImplementedError

        @classmethod
        def from_ecef(cls, *a, **k):
            raise NotImplementedError

    m.LocalCoord = LocalCoord

    # Any function name imported from here resolves to a loud raiser -- the controller suites
    # never call these; they only need the imports in calibrationd/orientation to resolve.
    def _mod_getattr(name):
        if name.startswith("__"):
            raise AttributeError(name)
        def f(*a, **k):
            raise NotImplementedError(f"transformations.{name} is stubbed on Windows")
        return f

    m.__getattr__ = _mod_getattr


_install_transformations_stub()


# --- openpilot.selfdrive.pandad.pandad_api_impl (Cython) -> loud raisers ---
def _install_pandad_stub():
    modname = "openpilot.selfdrive.pandad.pandad_api_impl"
    if modname in sys.modules:
        return
    m = _mod(modname)

    def _raiser(name):
        def f(*a, **k):
            raise NotImplementedError(f"pandad_api_impl.{name} is stubbed on the PC")
        return f

    m.can_list_to_can_capnp = _raiser("can_list_to_can_capnp")
    m.can_capnp_to_list = _raiser("can_capnp_to_list")


_install_pandad_stub()


# --- msgq.visionipc (Cython) -> constructible-but-inert classes ---
def _install_visionipc_stub():
    if "msgq.visionipc" in sys.modules:
        return
    m = _mod("msgq.visionipc")
    m.__path__ = []

    class _Inert:
        def __init__(self, *a, **k):
            pass

        def __getattr__(self, name):
            def f(*a, **k):
                raise NotImplementedError(f"visionipc.{name} is stubbed on the PC")
            return f

    class VisionStreamType:
        VISION_STREAM_ROAD = 0
        VISION_STREAM_DRIVER = 1
        VISION_STREAM_WIDE_ROAD = 2
        VISION_STREAM_MAP = 3

    m.VisionIpcServer = type("VisionIpcServer", (_Inert,), {})
    m.VisionIpcClient = type("VisionIpcClient", (_Inert,), {})
    m.VisionBuf = type("VisionBuf", (_Inert,), {})
    m.VisionStreamType = VisionStreamType
    m.get_endpoint_name = lambda *a, **k: ""
    sys.modules["msgq"].visionipc = m
    sub = _mod("msgq.visionipc.visionipc_pyx")
    for k in ("VisionIpcServer", "VisionIpcClient", "VisionBuf", "VisionStreamType", "get_endpoint_name"):
        setattr(sub, k, getattr(m, k))


_install_visionipc_stub()
