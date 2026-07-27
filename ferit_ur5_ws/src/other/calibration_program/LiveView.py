import os
import threading
import time
import traceback

import cv2


class LiveView:
    """Continuously displays the camera stream in its own thread.

    OpenCV's HighGUI is not thread-safe, so this class is the *only* place in
    the program that is allowed to call cv2.imshow / cv2.waitKey.  Command
    handlers that want to show a still image (e.g. "image show 3") hand it over
    through show_still() instead of opening their own window, which keeps every
    HighGUI call on a single thread.

    While running it shows the latest frame from the camera; press 'a' in the
    window to toggle the live ArUco overlay, and press space to capture the
    current frame (equivalent to typing "image capture").  A still image,
    when requested, pops up in a separate window and is dismissed by pressing
    any key.
    """

    LIVE_WINDOW = "calibration - live camera"
    STILL_WINDOW = "calibration - image"
    CAPTURE_DEBOUNCE_S = 0.5  # ignore repeat key events from a held-down space

    def __init__(self, cameraReader, arucoDetector, marker_size=0.15, show_aruco=True, on_capture=None):
        self.cameraReader = cameraReader
        self.arucoDetector = arucoDetector
        self.marker_size = marker_size
        self.show_aruco = show_aruco
        self.on_capture = on_capture

        self._running = False
        self._thread = None
        self._lock = threading.Lock()
        self._still = None  # (image, title) waiting to be shown, or None
        self._has_display = 'DISPLAY' in os.environ
        self._last_capture_time = 0.0

    # -- lifecycle ---------------------------------------------------------
    def start(self):
        if not self._has_display:
            print("No display detected (DISPLAY not set); live view disabled.")
            return
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._run, name="LiveView", daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    # -- public API used by command handlers -------------------------------
    def show_still(self, image, title="image"):
        """Queue a still image to be displayed by the view thread."""
        if not self._has_display:
            print("No display!")
            return
        if image is None:
            return
        with self._lock:
            self._still = (image.copy(), title)

    def set_aruco(self, enabled):
        self.show_aruco = bool(enabled)

    def set_on_capture(self, callback):
        self.on_capture = callback

    # -- internals (run on the view thread) --------------------------------
    def _render_live(self):
        frame = self.cameraReader.lastimage
        # CameraReader starts with lastimage = False until the first frame
        if frame is None or frame is False:
            return None
        frame = frame.copy()
        if self.show_aruco:
            try:
                markers = self.arucoDetector.detector.detect(
                    frame, self.arucoDetector.camparam, self.marker_size)
                for marker in markers:
                    marker.draw(frame, (0, 255, 0), 2)
                    marker.draw3dAxis(frame, self.arucoDetector.camparam, 0.05)
            except Exception:
                # never let a bad frame kill the stream
                pass
        return frame

    def _run(self):
        still_open = False
        try:
            while self._running:
                live = self._render_live()
                if live is not None:
                    cv2.imshow(self.LIVE_WINDOW, live)

                with self._lock:
                    still = self._still
                if still is not None:
                    image, title = still
                    cv2.imshow(self.STILL_WINDOW, image)
                    cv2.setWindowTitle(self.STILL_WINDOW, self.STILL_WINDOW + " - " + str(title))
                    still_open = True

                key = cv2.waitKey(30) & 0xFF
                if key != 255:  # 255 == no key pressed
                    if still_open:
                        with self._lock:
                            self._still = None
                        cv2.destroyWindow(self.STILL_WINDOW)
                        still_open = False
                    elif key in (ord('a'), ord('A')):
                        self.show_aruco = not self.show_aruco
                    elif key == ord(' '):
                        self._trigger_capture()
        finally:
            cv2.destroyAllWindows()

    def _trigger_capture(self):
        if self.on_capture is None:
            return
        now = time.monotonic()
        if now - self._last_capture_time < self.CAPTURE_DEBOUNCE_S:
            return  # debounce a held-down space bar
        self._last_capture_time = now
        try:
            self.on_capture()
        except Exception:
            traceback.print_exc()
