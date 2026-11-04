"""The Gamemaster's Vision tab: the arena camera and the AprilTags it sees.

The arena camera looks down on the table. Tags 1 to 4 sit at the arena's
corners and every other tag is on a robot. This finds the tags in each
frame the way arena_perception's apriltag_node does (DICT_APRILTAG_25h9,
same detector settings), maps them to arena centimetres from the corner
tags, and draws what it found over the picture. The page shows the picture
live (an MJPEG stream), a table of tags, where each robot is on a plan of
the arena, and the settings, which can be changed while it runs.

Where the tags come from:

- The Gamemaster opens the camera itself (Start on the tab). It then
  publishes /arena/tags as apriltag_node does, a JSON list of
  {"id", "x", "y", "x_cm", "y_cm"} with "yaw" (degrees) added, so
  anything reading that topic gets the same.
- Or apriltag_node runs and holds the camera. The tab then shows what it
  publishes on /arena/tags, with no picture, since that node only shows
  its frames in its own window.
- "test" as the source draws a made-up arena with a robot tag going round,
  to try the tab with no camera.

Needs OpenCV with its aruco module (python3-opencv on Ubuntu). Without it
the tab says so, and still shows /arena/tags.
"""

import json
import math
import os
import re
import threading
import time

try:
    import cv2
    import numpy as np
except ImportError:  # the tab says so
    cv2 = np = None

PAGE_NAME = "vision.html"
TOPIC = "/arena/tags"
CORNER_TAGS = (1, 2, 3, 4)
# A tag not seen for this long drops off the table and the plan.
STALE = 1.0
BOUNDARY = "frame"

DEFAULTS = {
    # A device number, a /dev/video path, an http:// or rtsp:// URL, or
    # "test" for a made-up arena.
    "source": "0",
    "width": 1280,
    "height": 720,
    "fps": 30,
    # The arena's side in cm, between the corner points.
    "arena_cm": 112.0,
    # How far in px the arena's corners are pushed out from the corner
    # tags' centres, as apriltag_node's resize_parameter.
    "corner_push": 40.0,
    "thresh_min": 3,
    "thresh_max": 53,
    "thresh_step": 4,
    "min_perimeter": 0.02,
    # What is drawn over the picture.
    "draw_tags": True,
    "draw_arena": True,
    "draw_grid": False,
    "jpeg_quality": 75,
}
LIMITS = {
    "width": (160, 3840), "height": (120, 2160), "fps": (1, 60),
    "arena_cm": (10.0, 1000.0), "corner_push": (-400.0, 400.0),
    "thresh_min": (3, 99), "thresh_max": (3, 199), "thresh_step": (1, 50),
    "min_perimeter": (0.001, 1.0), "jpeg_quality": (10, 95),
}


def clean_settings(asked, settings):
    """settings with the keys asked changed, checked. Raises ValueError
    naming the first bad one."""
    out = dict(settings)
    for key, value in asked.items():
        if key not in DEFAULTS:
            raise ValueError(f"no setting {key}")
        kind = type(DEFAULTS[key])
        if kind is bool:
            if not isinstance(value, bool):
                raise ValueError(f"{key} is true or false")
        elif kind is str:
            value = str(value).strip()
            if not value:
                raise ValueError(f"{key} is empty")
        else:
            try:
                value = kind(value)
            except (TypeError, ValueError):
                raise ValueError(f"{key} is a number") from None
            low, high = LIMITS[key]
            if not low <= value <= high:
                raise ValueError(f"{key} is from {low} to {high}")
        out[key] = value
    if out["thresh_max"] < out["thresh_min"]:
        raise ValueError("thresh_max is under thresh_min")
    return out


def corner_points(centres, push):
    """The arena's corners in px, from the corner tags' centres pushed out
    by push, as apriltag_node does (1 bottom right, 2 top right, 3 top
    left, 4 bottom left on its plan). None unless all four are seen."""
    if not all(tag in centres for tag in CORNER_TAGS):
        return None
    signs = {1: (1, 1), 2: (1, -1), 3: (-1, -1), 4: (-1, 1)}
    return [(centres[tag][0] + signs[tag][0] * push,
             centres[tag][1] + signs[tag][1] * push) for tag in CORNER_TAGS]


def arena_matrix(corners, side):
    """The px to cm perspective matrix from the four corner points."""
    src = np.float32(corners)
    dst = np.float32([[side, side], [side, 0], [0, 0], [0, side]])
    return cv2.getPerspectiveTransform(src, dst)


def to_cm(matrix, points):
    """px points to cm, through matrix."""
    array = np.array([points], dtype=np.float32)
    return [tuple(map(float, p))
            for p in cv2.perspectiveTransform(array, matrix)[0]]


def tags_from_corners(corners_px, ids, settings):
    """The detections in apriltag_node's form, from the detector's corners
    and ids. A tag's yaw is the way its top edge points, in degrees on the
    arena plan (0 along +x, counted towards +y), once the corner tags are
    seen. Also gives the arena's corner points, or None."""
    tags, centres = [], {}
    for points, tag in zip(corners_px, ids):
        points = [(float(x), float(y)) for x, y in points]
        cx = int(sum(p[0] for p in points) / 4)
        cy = int(sum(p[1] for p in points) / 4)
        tags.append({"id": int(tag), "x": cx, "y": cy, "_corners": points})
        if int(tag) in CORNER_TAGS:
            centres[int(tag)] = (cx, cy)
    corners = corner_points(centres, settings["corner_push"])
    matrix = arena_matrix(corners, settings["arena_cm"]) if corners else None
    for tag in tags:
        points = tag.pop("_corners")
        if matrix is None:
            continue
        (x, y), = to_cm(matrix, [(tag["x"], tag["y"])])
        tag["x_cm"], tag["y_cm"] = round(x, 2), round(y, 2)
        (ax, ay), (bx, by) = to_cm(matrix, points[:2])
        tag["yaw"] = round(math.degrees(math.atan2(by - ay, bx - ax)), 1)
    return tags, corners


class Detector:
    """OpenCV's AprilTag finder with the settings, for any OpenCV 4."""

    def __init__(self, settings):
        aruco = cv2.aruco
        self.dictionary = aruco.getPredefinedDictionary(
            aruco.DICT_APRILTAG_25h9)
        params = (aruco.DetectorParameters() if hasattr(
            aruco, "DetectorParameters") else aruco.DetectorParameters_create())
        params.cornerRefinementMethod = aruco.CORNER_REFINE_APRILTAG
        params.adaptiveThreshWinSizeMin = settings["thresh_min"]
        params.adaptiveThreshWinSizeMax = settings["thresh_max"]
        params.adaptiveThreshWinSizeStep = settings["thresh_step"]
        params.minMarkerPerimeterRate = settings["min_perimeter"]
        self.params = params
        self.detector = (aruco.ArucoDetector(self.dictionary, params)
                         if hasattr(aruco, "ArucoDetector") else None)

    def find(self, grey):
        """(corners, ids): each tag's four px corners and its id."""
        if self.detector is not None:
            corners, ids, _ = self.detector.detectMarkers(grey)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                grey, self.dictionary, parameters=self.params)
        if ids is None:
            return [], []
        return [c[0] for c in corners], [int(i) for i in ids.flatten()]


def draw(frame, tags, corners, settings):
    """Draws the tags, the arena and its grid over frame."""
    if settings["draw_arena"] and corners:
        poly = np.array(corners, dtype=np.int32)
        cv2.polylines(frame, [poly], True, (255, 0, 0), 3)
        if settings["draw_grid"]:
            back = np.linalg.inv(arena_matrix(corners, settings["arena_cm"]))
            side = settings["arena_cm"]
            for step in range(1, 4):
                v = side * step / 4
                for a, b in (((v, 0), (v, side)), ((0, v), (side, v))):
                    ends = cv2.perspectiveTransform(
                        np.array([[a, b]], dtype=np.float32), back)[0]
                    cv2.line(frame, tuple(map(int, ends[0])),
                             tuple(map(int, ends[1])), (255, 160, 0), 1)
    if not settings["draw_tags"]:
        return
    for tag in tags:
        robot = tag["id"] not in CORNER_TAGS
        colour = (0, 200, 255) if robot else (0, 0, 255)
        centre = (tag["x"], tag["y"])
        cv2.circle(frame, centre, 6, colour, -1)
        label = f"ID {tag['id']}"
        if "x_cm" in tag:
            label += f" ({tag['x_cm']:.0f}, {tag['y_cm']:.0f}) cm"
        cv2.putText(frame, label, (centre[0] + 10, centre[1]),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4)
        cv2.putText(frame, label, (centre[0] + 10, centre[1]),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, colour, 2)


class TestCamera:
    """A made-up arena seen from above: the four corner tags and robot tag
    5 going round the middle, for trying the tab with no camera."""

    def __init__(self, width, height):
        self.width, self.height = width, height
        self.start = time.monotonic()
        dictionary = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_APRILTAG_25h9)
        make = getattr(cv2.aruco, "generateImageMarker", None) or \
            cv2.aruco.drawMarker
        self.size = max(40, min(width, height) // 10)
        self.tags = {tag: make(dictionary, tag, self.size)
                     for tag in (1, 2, 3, 4, 5)}

    def isOpened(self):
        return True

    def read(self):
        frame = np.full((self.height, self.width, 3), 200, np.uint8)
        margin = self.size
        right = self.width - margin - self.size
        bottom = self.height - margin - self.size
        spots = {1: (right, bottom), 2: (right, margin), 3: (margin, margin),
                 4: (margin, bottom)}
        t = time.monotonic() - self.start
        radius = min(self.width, self.height) / 4
        cx = self.width / 2 + radius * math.cos(t / 2) - self.size / 2
        cy = self.height / 2 + radius * math.sin(t / 2) - self.size / 2
        spots[5] = (int(cx), int(cy))
        # Tag 5 faces the way it goes, to the nearest quarter turn.
        turns = int(((t / 2 + math.pi / 2) % (2 * math.pi)) // (math.pi / 2))
        images = dict(self.tags)
        images[5] = np.rot90(self.tags[5], -turns)
        for tag, (x, y) in spots.items():
            pad = self.size // 5
            frame[y - pad:y + self.size + pad,
                  x - pad:x + self.size + pad] = 255
            frame[y:y + self.size, x:x + self.size] = \
                images[tag][:, :, None]
        time.sleep(1 / 30)
        return True, frame

    def release(self):
        pass


def list_cameras(root="/sys/class/video4linux"):
    """The cameras connected, as [{"source", "name"}]: on Linux each
    /dev/video device that gives pictures (a camera's other nodes, for
    metadata, have an index over 0), named as the camera names itself.
    Elsewhere, the first few device numbers that open."""
    if os.path.isdir(root):
        found = []
        for entry in os.listdir(root):
            match = re.fullmatch(r"video(\d+)", entry)
            if not match:
                continue

            def read(name):
                try:
                    with open(os.path.join(root, entry, name)) as file:
                        return file.read().strip()
                except OSError:
                    return ""
            if read("index") not in ("", "0"):
                continue
            number = match.group(1)
            found.append({"source": number,
                          "name": f"{read('name') or 'Camera'} "
                                  f"(/dev/video{number})"})
        return sorted(found, key=lambda c: int(c["source"]))
    if cv2 is None:
        return []
    found = []
    for number in range(4):
        camera = cv2.VideoCapture(number)
        if camera.isOpened():
            found.append({"source": str(number), "name": f"Camera {number}"})
        camera.release()
    return found


def open_camera(settings):
    """A cv2.VideoCapture (or TestCamera) for the source, or raises
    RuntimeError saying why not."""
    source = settings["source"]
    if source == "test":
        return TestCamera(settings["width"], settings["height"])
    if source.isdigit():
        camera = cv2.VideoCapture(int(source), getattr(cv2, "CAP_V4L2", 0))
    else:
        camera = cv2.VideoCapture(source)
    if not camera.isOpened():
        raise RuntimeError(
            f"could not open camera {source}. Is apriltag_node or another "
            "program using it?")
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, settings["width"])
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, settings["height"])
    camera.set(cv2.CAP_PROP_FPS, settings["fps"])
    return camera


class Vision:
    """The camera, the tags and the settings behind the Vision tab. Safe
    to use from the web server's threads and the ROS thread at once."""

    def __init__(self, book=None, opener=None):
        self.book = book
        self.opener = opener or open_camera
        self.settings = dict(DEFAULTS)
        self.lock = threading.Lock()
        self.new_frame = threading.Condition(self.lock)
        self.jpeg = None
        self.frame_number = 0
        self.tags = []
        self.tags_at = 0.0
        self.tags_from = None  # "camera" or "topic"
        self.corners = None
        self.fps = 0.0
        self.error = None
        self.thread = None
        self.running = threading.Event()
        self.publish = None
        self.topic_count = 0

    def say(self, text, level="info"):
        if self.book is not None:
            self.book.add("vision", text, level)

    # ROS side

    def attach(self, node):
        """Reads and, while the camera runs here, publishes /arena/tags on
        node."""
        from std_msgs.msg import String
        publisher = node.create_publisher(String, TOPIC, 10)

        def publish(tags):
            publisher.publish(String(data=json.dumps(tags)))
        self.publish = publish
        node.create_subscription(String, TOPIC, self.take_topic, 10)

    def take_topic(self, message):
        """A /arena/tags message, from apriltag_node or from this app."""
        if self.running.is_set():
            return  # ours
        try:
            tags = json.loads(message.data)
        except ValueError:
            return
        if not isinstance(tags, list):
            return
        with self.lock:
            self.tags = [t for t in tags if isinstance(t, dict) and "id" in t]
            self.tags_at = time.monotonic()
            self.tags_from = "topic"
            self.topic_count += 1

    # Camera side

    def start(self):
        if cv2 is None:
            raise RuntimeError("OpenCV is not installed: sudo apt install "
                               "python3-opencv")
        if self.running.is_set():
            return
        camera = self.opener(self.settings)
        self.running.set()
        self.error = None
        self.thread = threading.Thread(target=self.loop, args=(camera,),
                                       daemon=True)
        self.thread.start()
        self.say(f"camera {self.settings['source']} started")

    def stop(self):
        if not self.running.is_set():
            return
        self.running.clear()
        if self.thread is not None:
            self.thread.join(timeout=2)
        self.say("camera stopped")

    def loop(self, camera):
        settings, detector = None, None
        last, frames = time.monotonic(), 0
        try:
            while self.running.is_set():
                ok, frame = camera.read()
                if not ok:
                    raise RuntimeError("the camera stopped giving frames")
                if settings is not self.settings:
                    settings = self.settings
                    detector = Detector(settings)
                self.process(frame, detector, settings)
                frames += 1
                now = time.monotonic()
                if now - last >= 1.0:
                    self.fps = round(frames / (now - last), 1)
                    last, frames = now, 0
        except Exception as error:  # shown on the tab
            self.error = str(error)
            self.say(f"camera: {error}", "error")
        finally:
            camera.release()
            self.running.clear()
            self.fps = 0.0
            with self.new_frame:
                self.new_frame.notify_all()

    def process(self, frame, detector, settings):
        """Finds the tags in frame, publishes them and keeps the drawn
        picture."""
        grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners_px, ids = detector.find(grey)
        tags, corners = tags_from_corners(corners_px, ids, settings)
        draw(frame, tags, corners, settings)
        ok, jpeg = cv2.imencode(
            ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, settings["jpeg_quality"]])
        if self.publish is not None:
            self.publish(tags)
        with self.new_frame:
            self.tags, self.corners = tags, corners
            self.tags_at = time.monotonic()
            self.tags_from = "camera"
            if ok:
                self.jpeg = jpeg.tobytes()
                self.frame_number += 1
            self.new_frame.notify_all()

    def wait_frame(self, after, timeout=2.0):
        """The next JPEG after frame number after: (number, jpeg), or
        (after, None) when none came."""
        with self.new_frame:
            self.new_frame.wait_for(
                lambda: self.frame_number != after or not self.running.is_set(),
                timeout)
            if self.frame_number == after:
                return after, None
            return self.frame_number, self.jpeg

    # The page

    def state(self):
        with self.lock:
            age = time.monotonic() - self.tags_at if self.tags_at else None
            tags = self.tags if age is not None and age < STALE else []
            return {
                "opencv": cv2.__version__ if cv2 is not None else None,
                "ros": self.publish is not None,
                "running": self.running.is_set(),
                "source": self.settings["source"],
                "fps": self.fps,
                "error": self.error,
                "tags_from": self.tags_from if tags else None,
                "tags_age": None if age is None else round(age, 2),
                "topic_messages": self.topic_count,
                "arena_found": all(any(t.get("id") == c for t in tags)
                                   for c in CORNER_TAGS),
                "tags": tags,
                "settings": dict(self.settings),
            }

    def change(self, asked):
        """Changes the settings asked. A new source restarts the camera."""
        settings = clean_settings(asked, self.settings)
        restart = self.running.is_set() and any(
            settings[k] != self.settings[k]
            for k in ("source", "width", "height", "fps"))
        if restart:
            self.stop()
        self.settings = settings
        if restart:
            self.start()

    def cameras(self):
        """The cameras to pick from: those connected, the one in use (it
        may be a URL, or held open so not listed), and the test one."""
        cameras = list_cameras() if not self.running.is_set() or \
            os.path.isdir("/sys/class/video4linux") else []
        source = self.settings["source"]
        if source != "test" and all(c["source"] != source for c in cameras):
            name = f"Camera {source}, not found" if source.isdigit() \
                else source
            cameras.append({"source": source, "name": name})
        return cameras + [{"source": "test", "name": "Test camera "
                           "(made-up arena)"}]

    def get(self, path):
        """GET routes: (status, body), or None for a path not the tab's."""
        if path == "/api/vision":
            return 200, self.state()
        if path == "/api/vision/cameras":
            return 200, {"cameras": self.cameras()}
        if path == "/vision/frame.jpg":
            if self.jpeg is None:
                return 404, {"error": "no picture yet"}
            return 200, self.jpeg, "image/jpeg"
        return None

    def post(self, path, read_body):
        """POST routes: start, stop and the settings."""
        try:
            if path == "/api/vision/start":
                self.start()
            elif path == "/api/vision/stop":
                self.stop()
            elif path == "/api/vision":
                self.change(read_body())
            else:
                return None
        except (RuntimeError, ValueError) as error:
            return 400, {"error": str(error)}
        return 200, self.state()

    def stream(self, write):
        """Writes the picture as an MJPEG stream through write(bytes) until
        the camera stops or write raises."""
        number = 0
        while self.running.is_set():
            number, jpeg = self.wait_frame(number)
            if jpeg is None:
                continue
            write(b"--" + BOUNDARY.encode() + b"\r\n"
                  b"Content-Type: image/jpeg\r\n"
                  + f"Content-Length: {len(jpeg)}\r\n\r\n".encode()
                  + jpeg + b"\r\n")
