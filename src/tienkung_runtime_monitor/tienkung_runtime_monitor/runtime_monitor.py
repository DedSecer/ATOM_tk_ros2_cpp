#!/usr/bin/env python3
"""Read-only, standard-library runtime monitor for policy JSONL logs.

Only the configured log and optional URDF are read; no robot/ROS connection is
made. Binding beyond loopback exposes unauthenticated telemetry to that network.
Use a trusted network or an authenticated reverse proxy, never the public Internet.
"""

import argparse
from collections import deque
import ipaddress
import json
import math
import os
from pathlib import Path
import socket
import stat
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import parse_qs, urlsplit
import xml.etree.ElementTree as ET


MAX_URDF_BYTES = 4 * 1024 * 1024
STATIC_ROUTES = {
    "/": ("index.html", "text/html; charset=utf-8"),
    "/monitor.js": ("monitor.js", "text/javascript; charset=utf-8"),
    "/monitor.css": ("monitor.css", "text/css; charset=utf-8"),
    "/robot_view.js": ("robot_view.js", "text/javascript; charset=utf-8"),
}


def _finite_float(text):
    value = float(text)
    if not math.isfinite(value):
        raise ValueError("nonfinite number")
    return value


def _reject_constant(text):
    raise ValueError("nonstandard JSON constant: " + text)


def _vector(text, default):
    if text is None:
        return list(default)
    parts = text.split()
    if len(parts) != 3:
        raise ValueError("URDF vectors must contain exactly three numbers")
    return [_finite_float(value) for value in parts]


def _mesh_path(filename, urdf_path, asset_root):
    if not filename or "\\" in filename or "%" in filename:
        raise ValueError("invalid URDF mesh filename")
    parsed = urlsplit(filename)
    if parsed.query or parsed.fragment or parsed.scheme not in ("", "package"):
        raise ValueError("remote/file mesh URLs are forbidden")
    if parsed.scheme == "package":
        # Only two unambiguous local package layouts are supported. No ROS
        # package lookup, environment search, network, or guessed fallback.
        if not parsed.netloc or not parsed.path.startswith("/"):
            raise ValueError("invalid package mesh URL")
        relative = Path(parsed.path[1:])
        if parsed.netloc == asset_root.name:
            candidate = asset_root / relative
        else:
            candidate = asset_root / parsed.netloc / relative
    else:
        if parsed.netloc or Path(filename).is_absolute():
            raise ValueError("mesh filename must be relative")
        relative = Path(filename)
        candidate = urdf_path.parent / relative
    if ".." in relative.parts or candidate.suffix.lower() != ".stl":
        raise ValueError("mesh must be a confined local STL path")
    resolved = candidate.resolve(strict=True)
    try:
        resolved.relative_to(asset_root)
    except ValueError as exc:
        raise ValueError("mesh escapes asset root") from exc
    info = resolved.stat()
    if not stat.S_ISREG(info.st_mode) or info.st_size > 32 * 1024 * 1024:
        raise ValueError("mesh must be a regular file of at most 32 MiB")
    return resolved


def _visuals(root, urdf_path, asset_root, mesh_files):
    visuals = []
    for link in root.findall("link"):
        for visual in link.findall("visual"):
            if len(visual.findall("origin")) > 1 or len(visual.findall("geometry")) != 1:
                raise ValueError("invalid URDF visual")
            origin = visual.find("origin")
            geometry_element = visual.find("geometry")
            if len(geometry_element) != 1:
                raise ValueError("visual geometry must contain one shape")
            shape = geometry_element[0]
            geometry = {"type": shape.tag}
            if shape.tag == "mesh":
                mesh = _mesh_path(shape.get("filename"), urdf_path, asset_root)
                if mesh not in mesh_files:
                    mesh_files.append(mesh)
                geometry.update(url="/meshes/" + str(mesh_files.index(mesh)),
                                scale=_vector(shape.get("scale"), (1, 1, 1)))
                if any(value == 0 for value in geometry["scale"]):
                    raise ValueError("mesh scale cannot be zero")
            elif shape.tag == "box":
                if shape.get("size") is None:
                    raise ValueError("box size is required")
                geometry["size"] = _vector(shape.get("size"), ())
                if any(value <= 0 for value in geometry["size"]):
                    raise ValueError("box size must be positive")
            elif shape.tag in ("cylinder", "sphere"):
                for key in (("radius", "length") if shape.tag == "cylinder" else ("radius",)):
                    geometry[key] = _finite_float(shape.get(key, "nan"))
                    if geometry[key] <= 0:
                        raise ValueError("shape dimensions must be positive")
            else:
                raise ValueError("unsupported URDF visual geometry")
            visuals.append({
                "link": link.get("name").strip(),
                "origin_xyz": _vector(None if origin is None else origin.get("xyz"), (0, 0, 0)),
                "origin_rpy": _vector(None if origin is None else origin.get("rpy"), (0, 0, 0)),
                "geometry": geometry,
            })
    return visuals


def load_urdf(path, max_bytes=MAX_URDF_BYTES, *, asset_root=None, mesh_files=None):
    """Read validated kinematics and visuals; an absent optional file is None.

    Invalid existing files fail closed. DTDs, entity expansion, remote resources,
    arbitrary asset paths, and xacro execution are never supported. mesh_files,
    when supplied, receives the explicitly referenced, confined local STL paths.
    """
    if path is None:
        return None
    try:
        with open(path, "rb") as stream:
            if not stat.S_ISREG(os.fstat(stream.fileno()).st_mode):
                raise ValueError("URDF must be a regular file")
            data = stream.read(max_bytes + 1)
    except FileNotFoundError:
        return None
    if len(data) > max_bytes:
        raise ValueError("URDF exceeds size limit")
    # Decode before checking declarations so UTF-16 cannot bypass the check.
    try:
        text = data.decode("utf-8-sig")
    except UnicodeError as exc:
        raise ValueError("URDF must be UTF-8 XML") from exc
    if "<!DOCTYPE" in text.upper() or "<!ENTITY" in text.upper() or "\x00" in text:
        raise ValueError("URDF DTDs and entities are forbidden")
    try:
        root = ET.fromstring(text)
    except ET.ParseError as exc:
        raise ValueError("malformed URDF XML") from exc
    if root.tag != "robot":
        raise ValueError("URDF root must be robot")
    links = []
    for link in root.findall("link"):
        name = link.get("name", "").strip()
        if not name or name in links:
            raise ValueError("URDF links require unique nonempty names")
        links.append(name)
    if not links:
        raise ValueError("URDF has no links")
    link_names = set(links)
    joints = []
    joint_names = set()
    parents = {}
    children = {name: [] for name in links}
    supported = {"fixed", "revolute", "continuous", "prismatic", "floating", "planar"}
    for element in root.findall("joint"):
        name = element.get("name", "").strip()
        kind = element.get("type")
        if not name or name in joint_names or kind not in supported:
            raise ValueError("invalid URDF joint name or type")
        joint_names.add(name)
        for tag in ("parent", "child"):
            if len(element.findall(tag)) != 1:
                raise ValueError("each joint needs one parent and one child")
        parent = element.find("parent").get("link")
        child = element.find("child").get("link")
        if parent not in link_names or child not in link_names or child in parents:
            raise ValueError("unknown link or multiple parents in URDF")
        parents[child] = parent
        children[parent].append(child)
        for tag in ("origin", "axis", "mimic"):
            if len(element.findall(tag)) > 1:
                raise ValueError("duplicate joint element: " + tag)
        origin = element.find("origin")
        axis_element = element.find("axis")
        if axis_element is not None and axis_element.get("xyz") is None:
            raise ValueError("URDF axis is missing xyz")
        axis = _vector(None if axis_element is None else axis_element.get("xyz"), (1, 0, 0))
        norm = math.hypot(*axis)
        if not math.isfinite(norm) or (norm < 1e-12 and kind not in ("fixed", "floating")):
            raise ValueError("movable URDF axis must be finite and nonzero")
        # Exporters commonly emit a zero axis for fixed joints; it is unused.
        if norm >= 1e-12:
            axis = [value / norm for value in axis]
        joint = {
            "name": name, "type": kind, "parent": parent, "child": child,
            "origin_xyz": _vector(None if origin is None else origin.get("xyz"), (0, 0, 0)),
            "origin_rpy": _vector(None if origin is None else origin.get("rpy"), (0, 0, 0)),
            "axis": axis,
        }
        mimic = element.find("mimic")
        if mimic is not None:
            joint["mimic"] = {
                "joint": mimic.get("joint"),
                "multiplier": _finite_float(mimic.get("multiplier", "1")),
                "offset": _finite_float(mimic.get("offset", "0")),
            }
        joints.append(joint)
    roots = [name for name in links if name not in parents]
    # Iterative traversal avoids recursion limits on large, valid robots.
    pending = list(roots)
    visited = set()
    while pending:
        name = pending.pop()
        if name in visited:
            raise ValueError("cycle in URDF")
        visited.add(name)
        pending.extend(children[name])
    if len(visited) != len(links):
        raise ValueError("cycle in URDF")
    mimic_edges = {joint["name"]: joint["mimic"]["joint"]
                   for joint in joints if "mimic" in joint}
    checked = set()
    for name in mimic_edges:
        chain = set()
        while name in mimic_edges and name not in checked:
            if name in chain:
                raise ValueError("cycle in URDF mimic joints")
            chain.add(name)
            name = mimic_edges[name]
            if name not in joint_names:
                raise ValueError("unknown URDF mimic joint")
        checked.update(chain)
    resolved_urdf = Path(path).resolve()
    allowed_root = Path(asset_root).resolve() if asset_root is not None else resolved_urdf.parent
    files = []
    visuals = _visuals(root, resolved_urdf, allowed_root, files)
    if mesh_files is not None:
        mesh_files.extend(files)
    return {"links": links, "joints": joints, "root_links": roots, "visuals": visuals}


class RuntimeMonitor:
    """One writer thread tails logs; snapshot readers only copy bounded state."""

    def __init__(self, *, log_file=None, log_dir=None, pattern="policy_log_*.jsonl",
                 history_limit=1500, response_limit=1500,
                 max_tail_bytes=8 * 1024 * 1024, max_line_bytes=1024 * 1024,
                 max_history_bytes=32 * 1024 * 1024, include_obs=False,
                 poll_interval=0.1, urdf=None):
        if (log_file is None) == (log_dir is None):
            raise ValueError("choose exactly one of log_file or log_dir")
        if not pattern or Path(pattern).name != pattern or "**" in pattern:
            raise ValueError("pattern must be a filename glob, not a path")
        for value in (history_limit, response_limit, max_tail_bytes, max_line_bytes,
                      max_history_bytes):
            if not isinstance(value, int) or value < 1:
                raise ValueError("resource limits must be positive integers")
        if not math.isfinite(poll_interval) or poll_interval <= 0:
            raise ValueError("poll_interval must be finite and positive")
        self.log_file = Path(log_file).absolute() if log_file is not None else None
        self.log_dir = Path(log_dir).absolute() if log_dir is not None else None
        self.pattern = pattern
        self.history_limit = history_limit
        self.response_limit = min(response_limit, history_limit)
        self.max_tail_bytes = max_tail_bytes
        self.max_line_bytes = max_line_bytes
        self.max_history_bytes = max_history_bytes
        self.include_obs = include_obs
        self.poll_interval = poll_interval
        self.urdf = urdf
        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._thread = None
        self._stream = None
        self._identity = None
        self._offset = 0
        self._anchor = b""
        self._prefix = b""
        self._partial = b""
        self._discard_line = False
        self._frames = deque()
        self._history_bytes = 0
        self._last_sample_time = None
        self.generation = 0
        self._source_mtime = None
        self.logger_stats = None
        self.metadata = None
        self.source = None
        self.last_seq = 0
        self.parse_errors = 0
        self.skipped_lines = 0
        self.skipped_bytes = 0
        self.last_error = None

    def _candidate(self):
        if self.log_file is not None:
            return self.log_file
        newest = None
        newest_key = None
        # scandir keeps directory discovery memory bounded even for many runs.
        import fnmatch
        with os.scandir(self.log_dir) as entries:
            for entry in entries:
                if not fnmatch.fnmatchcase(entry.name, self.pattern):
                    continue
                try:
                    info = entry.stat(follow_symlinks=False)
                except FileNotFoundError:
                    continue
                if not stat.S_ISREG(info.st_mode):
                    continue
                key = (info.st_mtime_ns, entry.name)
                if newest_key is None or key > newest_key:
                    newest_key = key
                    newest = Path(entry.path)
        return newest

    def _decode(self, line):
        try:
            value = json.loads(line, parse_constant=_reject_constant, parse_float=_finite_float)
            if not isinstance(value, dict):
                raise ValueError("JSONL record must be an object")
            return value
        except (ValueError, UnicodeError, RecursionError):
            self.parse_errors += 1
            return None

    def _record(self, line, metadata_only=False):
        if not line.strip():
            if not metadata_only:
                self.skipped_lines += 1
            return
        value = self._decode(line)
        if value is None:
            return
        if value.get("type") == "metadata":
            order = value.get("joint_order_policy")
            if (value.get("schema_version") != 2 or not isinstance(order, list)
                    or not all(isinstance(name, str) for name in order)
                    or not isinstance(value.get("config"), dict)):
                self.parse_errors += 1
                return
            self.metadata = value
            return
        if value.get("type") == "logger_stats":
            counters = (value.get(key) for key in
                        ("dropped_records", "written_records", "write_errors"))
            if (value.get("schema_version") != 2 or
                    any(type(count) is not int or count < 0 for count in counters)):
                self.parse_errors += 1
                return
            self.logger_stats = value
            return
        if metadata_only:
            return
        timestamp = value.get("timestamp")
        if (not isinstance(timestamp, (int, float)) or isinstance(timestamp, bool)
                or not isinstance(value.get("mode"), (str, int))):
            self.parse_errors += 1
            return
        try:
            if not math.isfinite(timestamp):
                raise ValueError("nonfinite timestamp")
        except (OverflowError, ValueError):
            self.parse_errors += 1
            return
        if not self.include_obs:
            value.pop("obs", None)
        self.last_seq += 1
        value["monitor_seq"] = self.last_seq
        size = len(json.dumps(value, ensure_ascii=True, separators=(",", ":")))
        if size > self.max_history_bytes:
            self.skipped_lines += 1
            return
        self._frames.append((value, size))
        self._history_bytes += size
        while (len(self._frames) > self.history_limit
               or self._history_bytes > self.max_history_bytes):
            _, removed_size = self._frames.popleft()
            self._history_bytes -= removed_size
        self._last_sample_time = time.monotonic()

    def _reset(self, path, stream, info):
        if self._stream is not None:
            self._stream.close()
        self._stream = stream
        self._identity = (info.st_dev, info.st_ino)
        self.source = str(path)
        self.generation += 1
        self.logger_stats = None
        self.metadata = None
        self._frames.clear()
        self._history_bytes = 0
        self._last_sample_time = None
        self._partial = b""
        self._discard_line = False
        self._offset = 0
        self._anchor = b""
        self._prefix = b""
        if info.st_size > self.max_tail_bytes:
            # Metadata is the first line, outside the tail window on large logs.
            first_line = stream.readline(self.max_line_bytes + 1)
            if first_line.endswith(b"\n") and len(first_line) <= self.max_line_bytes:
                self._record(first_line, metadata_only=True)
            self._seek_tail(info.st_size)

    def _seek_tail(self, size):
        start = max(0, size - self.max_tail_bytes)
        self.skipped_bytes += max(0, start - self._offset)
        self._partial = b""
        self._discard_line = False
        if start:
            self._stream.seek(start - 1)
            self._discard_line = self._stream.read(1) != b"\n"
            if self._discard_line:
                self.skipped_lines += 1
        self._stream.seek(start)
        self._offset = start

    def _consume(self, data):
        for part in data.splitlines(keepends=True):
            # JSONL uses LF only; splitlines also recognizes CR, so joining a CR
            # fragment below is harmless and never completes a line prematurely.
            complete = part.endswith(b"\n")
            if self._discard_line:
                if complete:
                    self._discard_line = False
                continue
            if len(self._partial) + len(part) > self.max_line_bytes:
                self._partial = b""
                self.skipped_lines += 1
                self._discard_line = not complete
                continue
            self._partial += part
            if complete:
                self._record(self._partial)
                self._partial = b""

    def poll_once(self):
        """Perform one bounded tail pass (also useful for deterministic tests)."""
        with self._lock:
            try:
                path = self._candidate()
                if path is None:
                    self.last_error = "waiting for log file"
                    return
                info = path.stat()
                if not stat.S_ISREG(info.st_mode):
                    self.last_error = "log source is not a regular file"
                    return
                changed = (self._stream is None or self.source != str(path)
                           or self._identity != (info.st_dev, info.st_ino))
                if not changed:
                    changed = info.st_size < self._offset
                    if not changed and self._anchor:
                        self._stream.seek(self._offset - len(self._anchor))
                        changed = self._stream.read(len(self._anchor)) != self._anchor
                    if not changed and self._prefix:
                        self._stream.seek(0)
                        changed = self._stream.read(len(self._prefix)) != self._prefix
                if changed:
                    # Nonblocking open ensures a raced-in FIFO cannot stall the
                    # sole tail thread. fstat also closes the stat/open race.
                    fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
                    stream = os.fdopen(fd, "rb", buffering=0)
                    current = os.fstat(stream.fileno())
                    if not stat.S_ISREG(current.st_mode):
                        stream.close()
                        self.last_error = "log source is not a regular file"
                        return
                    self._reset(path, stream, current)
                    info = current
                self._source_mtime = info.st_mtime
                if info.st_size - self._offset > self.max_tail_bytes:
                    self._seek_tail(info.st_size)
                self._stream.seek(self._offset)
                data = self._stream.read(min(self.max_tail_bytes, max(0, info.st_size - self._offset)))
                self._offset += len(data)
                self._consume(data)
                self._stream.seek(max(0, self._offset - 64))
                self._anchor = self._stream.read(min(64, self._offset))
                self._stream.seek(0)
                self._prefix = self._stream.read(min(256, self._offset))
                self.last_error = None
            except OSError as exc:
                self.last_error = "log unavailable: " + str(exc)

    def _run(self):
        while not self._stop.is_set():
            self.poll_once()
            self._stop.wait(self.poll_interval)

    def start(self):
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                return
            self._stop.clear()
            self._thread = threading.Thread(target=self._run, name="runtime-log-tail", daemon=True)
            self._thread.start()

    def close(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join()
        with self._lock:
            if self._stream is not None:
                self._stream.close()
                self._stream = None

    def snapshot(self, after=0, include_frames=True):
        with self._lock:
            frames = [frame for frame, _ in self._frames if frame["monitor_seq"] > after]
            selected = frames[-self.response_limit:] if include_frames else []
            oldest_seq = self._frames[0][0]["monitor_seq"] if self._frames else None
            return {
                "generation": self.generation, "metadata": self.metadata,
                "frames": selected, "last_seq": self.last_seq,
                "source": self.source,
                "logger_stats": self.logger_stats,
                "source_age_sec": (None if self._source_mtime is None else
                                   max(0.0, time.time() - self._source_mtime)),
                # Filesystem time is wall time; sample timestamps may be ROS
                # simulation time and must never be compared with wall time.
                "age_sec": (None if self._last_sample_time is None else
                            max(0.0, time.monotonic() - self._last_sample_time,
                                time.time() - self._source_mtime if self._source_mtime is not None else 0.0)),
                "parse_errors": self.parse_errors, "skipped_lines": self.skipped_lines,
                "skipped_bytes": self.skipped_bytes, "urdf": self.urdf,
                "history_limit": self.history_limit, "response_limit": self.response_limit,
                "history_size": len(self._frames), "oldest_seq": oldest_seq,
                "truncated": (len(frames) > self.response_limit or
                              (after > 0 and oldest_seq is not None and after < oldest_seq - 1)),
                "last_error": self.last_error,
            }


class MonitorServer(HTTPServer):
    """Fixed-route server, with one background log reader and bounded requests."""

    allow_reuse_address = True

    def __init__(self, address, monitor, assets_dir=None, *, mesh_files=(), asset_root=None):
        # Bind only numeric addresses; a hostname could change DNS resolution.
        address_ip = ipaddress.ip_address(address[0])
        if address_ip.version == 6:
            self.address_family = socket.AF_INET6
        self.monitor = monitor
        self.mesh_files = tuple(Path(path).resolve() for path in mesh_files)
        self.asset_root = Path(asset_root).resolve() if asset_root is not None else None
        if self.mesh_files and self.asset_root is None:
            raise ValueError("mesh files require an asset root")
        self.assets_dir = (Path(assets_dir) if assets_dir is not None else
                           Path(__file__).resolve().parent / "monitor")
        super().__init__(address, MonitorHandler)
        self.allowed_hosts = {address_ip.compressed.lower()}
        if address_ip.is_loopback:
            self.allowed_hosts.update({"localhost", "127.0.0.1", "::1"})

    def get_request(self):
        connection, address = super().get_request()
        connection.settimeout(3.0)
        return connection, address


class MonitorHandler(BaseHTTPRequestHandler):
    server_version = "RuntimeMonitor"
    sys_version = ""

    def log_message(self, format_string, *args):
        # Polling is intentionally quiet; never reflect untrusted request text.
        pass

    def end_headers(self):
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("X-Frame-Options", "DENY")
        self.send_header("Referrer-Policy", "no-referrer")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Security-Policy", "default-src 'self'; script-src 'self'; "
                         "style-src 'self'; connect-src 'self'; img-src 'self' data:; "
                         "object-src 'none'; base-uri 'none'; frame-ancestors 'none'")
        self.send_header("Cross-Origin-Resource-Policy", "same-origin")
        super().end_headers()

    def _reply(self, status, payload, content_type="application/json; charset=utf-8"):
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        try:
            self.wfile.write(payload)
        except (BrokenPipeError, ConnectionResetError, TimeoutError):
            pass

    def _error(self, status, message):
        self._reply(status, json.dumps({"error": message}).encode("utf-8"))

    def _host_valid(self):
        hosts = self.headers.get_all("Host", [])
        if len(hosts) != 1:
            return False
        host = hosts[0]
        if any(char.isspace() for char in host) or any(char in host for char in "/\\@?#,"):
            return False
        try:
            parsed = urlsplit("http://" + host)
            name = parsed.hostname
            port = parsed.port if parsed.port is not None else 80
        except ValueError:
            return False
        # Wildcard bind accepts numeric local destination addresses only, not an
        # arbitrary hostname resolved by a rebinding attacker.
        allowed = set(self.server.allowed_hosts)
        allowed.add(self.connection.getsockname()[0].lower())
        return name in allowed and port == self.server.server_port

    def do_GET(self):
        if not self._host_valid():
            self._error(403, "Host not allowed")
            return
        if self.headers.get("Sec-Fetch-Site", "").lower() == "cross-site":
            self._error(403, "Cross-site request denied")
            return
        try:
            parsed = urlsplit(self.path)
        except ValueError:
            self._error(400, "Invalid request target")
            return
        if parsed.scheme or parsed.netloc or parsed.fragment or not self.path.startswith("/"):
            self._error(400, "Invalid request target")
            return
        if parsed.path in ("/api/snapshot", "/api/health"):
            try:
                query = parse_qs(parsed.query, keep_blank_values=True, max_num_fields=4)
                if set(query) - {"after", "since"} or ("after" in query and "since" in query):
                    raise ValueError("invalid query")
                values = query.get("after", query.get("since", ["0"]))
                if len(values) != 1 or not values[0].isascii() or not values[0].isdigit():
                    raise ValueError("invalid sequence")
                after = int(values[0])
                if after > 2 ** 63 - 1:
                    raise ValueError("sequence too large")
            except ValueError:
                self._error(400, "Expected a nonnegative after sequence")
                return
            snapshot = self.server.monitor.snapshot(after, include_frames=parsed.path == "/api/snapshot")
            if parsed.path == "/api/health":
                snapshot = {key: snapshot[key] for key in (
                    "generation", "last_seq", "source", "age_sec", "source_age_sec", "parse_errors",
                    "skipped_lines", "skipped_bytes", "history_size", "last_error")}
                snapshot["status"] = "ok" if snapshot["source"] and not snapshot["last_error"] else "waiting"
            self._reply(200, json.dumps(snapshot, allow_nan=False, separators=(",", ":")).encode("utf-8"))
            return
        if parsed.path.startswith("/meshes/"):
            index = parsed.path[len("/meshes/"):]
            if (not index.isascii() or not index.isdigit() or len(index) > 8
                    or str(int(index)) != index or int(index) >= len(self.server.mesh_files)):
                self._error(404, "Not found")
                return
            mesh = self.server.mesh_files[int(index)]
            try:
                # Revalidate on every request so replacing an asset with an
                # escaping symlink after startup does not expose another file.
                resolved = mesh.resolve(strict=True)
                resolved.relative_to(self.server.asset_root)
                if resolved != mesh:
                    raise ValueError("mesh path changed")
                fd = os.open(mesh, os.O_RDONLY | os.O_NONBLOCK | getattr(os, "O_NOFOLLOW", 0))
                with os.fdopen(fd, "rb") as stream:
                    info = os.fstat(stream.fileno())
                    if not stat.S_ISREG(info.st_mode) or info.st_size > 32 * 1024 * 1024:
                        raise ValueError("mesh is not a bounded regular file")
                    data = stream.read(32 * 1024 * 1024 + 1)
                if len(data) > 32 * 1024 * 1024:
                    raise ValueError("mesh exceeds size limit")
            except (OSError, ValueError):
                self._error(404, "Mesh unavailable")
                return
            self._reply(200, data, "model/stl")
            return
        route = STATIC_ROUTES.get(parsed.path)
        if route is None:
            self._error(404, "Not found")
            return
        filename, content_type = route
        try:
            asset = self.server.assets_dir / filename
            if asset.is_symlink() or not asset.is_file():
                raise FileNotFoundError()
            with asset.open("rb") as stream:
                data = stream.read(2 * 1024 * 1024 + 1)
            if len(data) > 2 * 1024 * 1024:
                self._error(503, "Static asset exceeds size limit")
                return
        except OSError:
            self._error(404, "Static asset unavailable")
            return
        self._reply(200, data, content_type)

    def _method_not_allowed(self):
        self._error(405, "Read-only service: GET only")

    do_POST = do_PUT = do_PATCH = do_DELETE = do_OPTIONS = do_HEAD = _method_not_allowed


def _positive_int(value):
    parsed = int(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("must be positive")
    return parsed


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--log-file", help="Explicit JSONL path (may not exist yet)")
    source.add_argument("--log-dir", help="Directory to watch for the newest matching run")
    parser.add_argument("--pattern", default="policy_log_*.jsonl")
    parser.add_argument("--bind", default="0.0.0.0", help="Numeric listen address; defaults to all IPv4 interfaces, without authentication")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--urdf", help="Optional local UTF-8 URDF (no xacro execution)")
    parser.add_argument("--asset-root", help="Allowed local mesh directory (default: URDF parent)")
    parser.add_argument("--history-frames", type=_positive_int, default=1500)
    parser.add_argument("--response-frames", type=_positive_int, default=1500)
    parser.add_argument("--max-tail-bytes", type=_positive_int, default=8 * 1024 * 1024)
    parser.add_argument("--max-line-bytes", type=_positive_int, default=1024 * 1024)
    parser.add_argument("--max-history-bytes", type=_positive_int, default=32 * 1024 * 1024)
    parser.add_argument("--include-obs", action="store_true", help="Include full observations in live responses")
    args = parser.parse_args(argv)
    if not 0 <= args.port <= 65535:
        parser.error("port must be between 0 and 65535")
    if args.asset_root and not args.urdf:
        parser.error("--asset-root requires --urdf")
    try:
        mesh_files = []
        urdf = load_urdf(args.urdf, asset_root=args.asset_root, mesh_files=mesh_files)
        asset_root = (Path(args.asset_root).resolve() if args.asset_root else
                      (Path(args.urdf).resolve().parent if args.urdf else None))
        monitor = RuntimeMonitor(
            log_file=args.log_file, log_dir=args.log_dir, pattern=args.pattern,
            history_limit=args.history_frames, response_limit=args.response_frames,
            max_tail_bytes=args.max_tail_bytes, max_line_bytes=args.max_line_bytes,
            max_history_bytes=args.max_history_bytes, include_obs=args.include_obs,
            urdf=urdf)
        server = MonitorServer((args.bind, args.port), monitor,
                               mesh_files=mesh_files, asset_root=asset_root)
    except (OSError, ValueError) as exc:
        parser.error(str(exc))
    if not ipaddress.ip_address(args.bind).is_loopback:
        print("WARNING: non-loopback monitor exposes unauthenticated read-only telemetry; "
              "use a trusted network or authenticated proxy.", file=sys.stderr)
    monitor.start()
    print("Runtime monitor listening on {}:{}".format(args.bind, server.server_port), file=sys.stderr)
    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        monitor.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
