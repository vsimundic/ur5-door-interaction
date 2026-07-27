import math
import cv2
import numpy as np

# ArUco marker detection backed by OpenCV (cv2.aruco) instead of the external
# "aruco" (Salinas) python bindings.  The public interface is kept compatible
# with the old code so the rest of the program is unchanged:
#
#   detector = ArucoDetector(camera_params_path, aruco_dict_path)
#   markers  = detector.detector.detect(image, detector.camparam, marker_size)
#   for m in markers:
#       m.id, m.getCenter(), m.getTransformMatrix()
#       m.calculateExtrinsics(marker_size, detector.camparam)  # -> m.Rvec, m.Tvec
#       m.draw(image, color, thickness)
#       m.draw3dAxis(image, detector.camparam, length)


class CameraParameters:
    """Camera intrinsics, replacing aruco.CameraParameters."""

    def __init__(self, camera_matrix, dist_coeffs, image_size=None):
        self.CameraMatrix = np.asarray(camera_matrix, dtype=np.float64).reshape(3, 3)
        self.Distorsion = np.asarray(dist_coeffs, dtype=np.float64).reshape(-1)
        self.CamSize = image_size

    def isValid(self):
        return self.CameraMatrix is not None and self.CameraMatrix.shape == (3, 3)


class Marker:
    """A single detected marker, replacing aruco.Marker.

    The transform matrix is the pose of the marker expressed in the camera
    frame (C_T_M); its translation column is the marker centre in camera
    coordinates (metres) - exactly what the calibration relies on.
    """

    def __init__(self, marker_id, corners):
        self.id = int(marker_id)
        # image corners in detectMarkers order: TL, TR, BR, BL
        self.corners = np.asarray(corners, dtype=np.float32).reshape(4, 2)
        self.Rvec = None
        self.Tvec = None
        self._T = None

    def getCenter(self):
        c = self.corners.mean(axis=0)
        return (float(c[0]), float(c[1]))

    def calculateExtrinsics(self, marker_size, camparam):
        half = marker_size / 2.0
        # object points matching OpenCV/aruco convention: X right, Y up, Z out
        obj = np.array([[-half,  half, 0.0],
                        [ half,  half, 0.0],
                        [ half, -half, 0.0],
                        [-half, -half, 0.0]], dtype=np.float32)
        ok, rvec, tvec = cv2.solvePnP(obj, self.corners,
                                      camparam.CameraMatrix, camparam.Distorsion,
                                      flags=cv2.SOLVEPNP_IPPE_SQUARE)
        self.Rvec = rvec.reshape(3, 1)
        self.Tvec = tvec.reshape(3, 1)
        R, _ = cv2.Rodrigues(self.Rvec)
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = self.Tvec[:, 0]
        self._T = T
        return self._T

    def getTransformMatrix(self):
        if self._T is None:
            raise RuntimeError("calculateExtrinsics must be called before getTransformMatrix")
        return self._T

    def draw(self, image, color=(0, 0, 255), thickness=2):
        border = tuple(int(c) for c in np.asarray(color).reshape(-1)[:3])
        cv2.aruco.drawDetectedMarkers(image, [self.corners.reshape(1, 4, 2)],
                                      np.array([[self.id]], dtype=np.int32), border)
        return image

    def draw3dAxis(self, image, camparam, length=0.1):
        if self.Rvec is None or self.Tvec is None:
            raise RuntimeError("calculateExtrinsics must be called before draw3dAxis")
        # drawFrameAxes is the modern name; drawAxis is the pre-4.7 aruco name
        if hasattr(cv2, "drawFrameAxes"):
            cv2.drawFrameAxes(image, camparam.CameraMatrix, camparam.Distorsion,
                              self.Rvec, self.Tvec, length)
        else:
            cv2.aruco.drawAxis(image, camparam.CameraMatrix, camparam.Distorsion,
                               self.Rvec, self.Tvec, length)
        return image


class _MarkerDetector:
    """Detector, replacing aruco.MarkerDetector."""

    def __init__(self, dictionary):
        self.dictionary = dictionary
        self.params = cv2.aruco.DetectorParameters_create()

    def detect(self, image, camparam, marker_size):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary,
                                                  parameters=self.params)
        markers = []
        if ids is None:
            return markers
        for corner, marker_id in zip(corners, ids.flatten()):
            marker = Marker(marker_id, corner)
            marker.calculateExtrinsics(marker_size, camparam)
            markers.append(marker)
        return markers


class ArucoDetector:
    def __init__(self, camera_params_path: str, aruco_dict_path: str):
        camera_mat, dist_coeffs, image_size = self._read_camera_params(camera_params_path)
        self.camparam = CameraParameters(camera_mat, dist_coeffs, image_size)
        dictionary = self._load_dictionary(aruco_dict_path)
        self.detector = _MarkerDetector(dictionary)

    @staticmethod
    def _read_camera_params(path):
        fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
        camera_mat = fs.getNode('camera_matrix').mat()
        dist_coeffs = fs.getNode('distortion_coefficients').mat()
        w, h = fs.getNode('image_width'), fs.getNode('image_height')
        image_size = None
        if not w.empty() and not h.empty():
            image_size = (int(w.real()), int(h.real()))
        fs.release()
        if camera_mat is None:
            raise ValueError("Could not read 'camera_matrix' from %s" % path)
        return camera_mat, dist_coeffs, image_size

    @staticmethod
    def _load_dictionary(aruco_dict_path):
        """Build a cv2.aruco dictionary from an aruco-library ".dict" file so
        marker IDs stay identical to the previous setup."""
        nbits, codes = ArucoDetector._parse_dict_file(aruco_dict_path)
        marker_size = int(round(math.sqrt(nbits)))
        if marker_size * marker_size != nbits:
            raise ValueError("nbits=%d is not a square number of bits" % nbits)

        n = len(codes)
        bytes_list = np.zeros((n, (nbits + 7) // 8, 4), dtype=np.uint8)
        for i in range(n):
            grid = codes[i].reshape(marker_size, marker_size)
            bytes_list[i] = cv2.aruco.Dictionary_getByteListFromBits(grid)

        dictionary = cv2.aruco.Dictionary_create(1, marker_size)
        dictionary.markerSize = marker_size
        dictionary.bytesList = bytes_list
        dictionary.maxCorrectionBits = ArucoDetector._max_correction_bits(codes, marker_size)
        return dictionary

    @staticmethod
    def _parse_dict_file(path):
        nbits = None
        codes = []
        with open(path) as f:
            for line in f:
                line = line.strip()
                if line.startswith('nbits'):
                    nbits = int(line.split()[1])
                elif nbits is not None and len(line) == nbits and set(line) <= {'0', '1'}:
                    codes.append([int(c) for c in line])
        if nbits is None or not codes:
            raise ValueError("Could not parse aruco dictionary file %s" % path)
        return nbits, np.array(codes, dtype=np.uint8)

    @staticmethod
    def _max_correction_bits(codes, marker_size):
        """Error-correction budget = (minHamming - 1) // 2, computed over every
        pair of markers and every 90-degree rotation, matching OpenCV."""
        grids = codes.reshape(-1, marker_size, marker_size)
        rots = np.zeros((len(grids), 4), dtype=np.uint64)
        for i, grid in enumerate(grids):
            rotated = grid
            for k in range(4):
                rots[i, k] = int(''.join(map(str, rotated.flatten())), 2)
                rotated = np.rot90(rotated)
        ident = rots[:, 0]
        min_dist = marker_size * marker_size
        eye = np.eye(len(grids), dtype=bool)
        for k in range(4):
            xor = (ident[:, None] ^ rots[None, :, k]).astype(np.uint64)
            hamming = np.unpackbits(xor.view(np.uint8).reshape(len(grids), len(grids), 8),
                                    axis=2).sum(2)
            if k == 0:
                hamming = hamming[~eye]
            min_dist = min(min_dist, int(hamming.min()))
        return (min_dist - 1) // 2
