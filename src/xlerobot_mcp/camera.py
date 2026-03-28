"""Camera providers for XLeRobot visual observation."""

from __future__ import annotations

import base64
import mimetypes
import re
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Literal, Optional

NavigationMode = Literal["normal", "precision"]

DEFAULT_CAMERA_WIDTH = 640
DEFAULT_CAMERA_HEIGHT = 480
DEFAULT_CAMERA_FPS = 30
DEFAULT_CAMERA_FOV_DEG = 120.0

_MOCK_IMAGE_PNG_BASE64 = (
    "iVBORw0KGgoAAAANSUhEUgAAAEAAAABACAIAAAAlC+aJAAAApElEQVR4nO3PQQ0AIBDAsAP/nuGN"
    "AvZoFSzZOjNnyNi/A3gV0CqgVUCrgFYBrQJaBbQKaBXQKqBVQKuAVgGtAloFtApoFdAqoFVAq4BWAa0C"
    "WgW0CmgV0CqgVUCrgFYBrQJaBbQKaBXQKqBVQKuAVgGtAloFtApoFdAqoFVAq4BWgf0AcgYB5MZ5CqIA"
    "AAAASUVORK5CYII="
)


class CameraError(RuntimeError):
    """Base camera error."""


def _timestamp() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _parse_index_or_path(value: str) -> int | str:
    stripped = value.strip()
    if re.fullmatch(r"-?\d+", stripped):
        return int(stripped)
    return stripped


def _import_cv2() -> Any:
    try:
        import cv2
    except ImportError as exc:  # pragma: no cover - depends on optional dependency
        raise CameraError(
            "OpenCV is required for live camera capture. "
            "Install it with `uv sync --extra hardware` or `pip install opencv-python-headless`."
        ) from exc
    return cv2


@dataclass
class CameraSnapshot:
    image_bytes: bytes
    mime_type: str
    source: str
    timestamp: str
    width: int | None
    height: int | None
    camera_fov_deg: float
    center_angle_deg: float
    navigation_mode: NavigationMode
    overlay_applied: bool


class CameraProvider:
    def capture_image(
        self,
        *,
        camera_fov_deg: float = DEFAULT_CAMERA_FOV_DEG,
        center_angle_deg: float = 0.0,
        navigation_mode: NavigationMode = "normal",
        include_debug_overlay: bool = True,
    ) -> CameraSnapshot:
        raise NotImplementedError

    def get_status(self) -> dict[str, Any]:
        raise NotImplementedError

    def release(self) -> None:
        return None


class UnavailableCameraProvider(CameraProvider):
    def __init__(self, reason: str = "No camera is configured.") -> None:
        self.reason = reason

    def capture_image(
        self,
        *,
        camera_fov_deg: float = DEFAULT_CAMERA_FOV_DEG,
        center_angle_deg: float = 0.0,
        navigation_mode: NavigationMode = "normal",
        include_debug_overlay: bool = True,
    ) -> CameraSnapshot:
        raise CameraError(
            f"{self.reason} Start the server with --camera-index-or-path 0 "
            "or --camera-mock-image /path/to/frame.jpg."
        )

    def get_status(self) -> dict[str, Any]:
        return {
            "configured": False,
            "source": None,
            "live": False,
            "reason": self.reason,
        }


class StaticImageCameraProvider(CameraProvider):
    def __init__(self, image_path: str | Path, *, source_label: str = "file") -> None:
        self.image_path = Path(image_path).expanduser()
        if not self.image_path.exists():
            raise CameraError(f"Camera image path does not exist: {self.image_path}")
        self.source_label = source_label

    def capture_image(
        self,
        *,
        camera_fov_deg: float = DEFAULT_CAMERA_FOV_DEG,
        center_angle_deg: float = 0.0,
        navigation_mode: NavigationMode = "normal",
        include_debug_overlay: bool = True,
    ) -> CameraSnapshot:
        mime_type, _ = mimetypes.guess_type(str(self.image_path))
        if mime_type is None:
            mime_type = "image/png"
        return CameraSnapshot(
            image_bytes=self.image_path.read_bytes(),
            mime_type=mime_type,
            source=f"{self.source_label}:{self.image_path}",
            timestamp=_timestamp(),
            width=None,
            height=None,
            camera_fov_deg=float(camera_fov_deg),
            center_angle_deg=float(center_angle_deg),
            navigation_mode=navigation_mode,
            overlay_applied=False,
        )

    def get_status(self) -> dict[str, Any]:
        return {
            "configured": True,
            "source": str(self.image_path),
            "live": False,
            "provider": self.source_label,
        }


class MockCameraProvider(CameraProvider):
    def __init__(self) -> None:
        self._image_bytes = base64.b64decode(_MOCK_IMAGE_PNG_BASE64)

    def capture_image(
        self,
        *,
        camera_fov_deg: float = DEFAULT_CAMERA_FOV_DEG,
        center_angle_deg: float = 0.0,
        navigation_mode: NavigationMode = "normal",
        include_debug_overlay: bool = True,
    ) -> CameraSnapshot:
        return CameraSnapshot(
            image_bytes=self._image_bytes,
            mime_type="image/png",
            source="mock",
            timestamp=_timestamp(),
            width=64,
            height=64,
            camera_fov_deg=float(camera_fov_deg),
            center_angle_deg=float(center_angle_deg),
            navigation_mode=navigation_mode,
            overlay_applied=False,
        )

    def get_status(self) -> dict[str, Any]:
        return {
            "configured": True,
            "source": "mock",
            "live": False,
            "provider": "mock",
        }


class OpenCVCameraProvider(CameraProvider):
    def __init__(
        self,
        index_or_path: str | int,
        *,
        width: int = DEFAULT_CAMERA_WIDTH,
        height: int = DEFAULT_CAMERA_HEIGHT,
        fps: int = DEFAULT_CAMERA_FPS,
    ) -> None:
        self.cv2 = _import_cv2()
        self.index_or_path = index_or_path
        self.width = int(width)
        self.height = int(height)
        self.fps = int(fps)
        self.capture = self.cv2.VideoCapture(index_or_path)
        self._configure_capture()
        if not self.capture.isOpened():
            raise CameraError(f"Failed to open camera source: {index_or_path}")

    def _configure_capture(self) -> None:
        self.capture.set(self.cv2.CAP_PROP_BUFFERSIZE, 1)
        self.capture.set(self.cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.capture.set(self.cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.capture.set(self.cv2.CAP_PROP_FPS, self.fps)

    def _reopen(self) -> None:
        self.release()
        self.capture = self.cv2.VideoCapture(self.index_or_path)
        self._configure_capture()

    def _apply_overlay(
        self,
        frame: Any,
        *,
        camera_fov_deg: float,
        center_angle_deg: float,
        navigation_mode: NavigationMode,
    ) -> Any:
        overlay = frame.copy()
        height, width = overlay.shape[:2]
        center_x = width // 2
        self.cv2.line(overlay, (center_x, 0), (center_x, height), (0, 255, 0), 2)
        self.cv2.putText(
            overlay,
            f"mode={navigation_mode} fov={camera_fov_deg:.1f}deg center={center_angle_deg:.1f}deg",
            (16, 28),
            self.cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2,
            self.cv2.LINE_AA,
        )
        return overlay

    def capture_image(
        self,
        *,
        camera_fov_deg: float = DEFAULT_CAMERA_FOV_DEG,
        center_angle_deg: float = 0.0,
        navigation_mode: NavigationMode = "normal",
        include_debug_overlay: bool = True,
    ) -> CameraSnapshot:
        if not self.capture.isOpened():
            self._reopen()
        self.capture.grab()
        ok, frame = self.capture.read()
        if not ok or frame is None:
            self._reopen()
            self.capture.grab()
            ok, frame = self.capture.read()
        if not ok or frame is None:
            raise CameraError(f"Failed to read frame from camera source: {self.index_or_path}")

        if include_debug_overlay:
            frame = self._apply_overlay(
                frame,
                camera_fov_deg=float(camera_fov_deg),
                center_angle_deg=float(center_angle_deg),
                navigation_mode=navigation_mode,
            )

        ok, buffer = self.cv2.imencode(".jpg", frame)
        if not ok:
            raise CameraError("Failed to encode camera frame as JPEG.")

        height, width = frame.shape[:2]
        return CameraSnapshot(
            image_bytes=buffer.tobytes(),
            mime_type="image/jpeg",
            source=f"opencv:{self.index_or_path}",
            timestamp=_timestamp(),
            width=int(width),
            height=int(height),
            camera_fov_deg=float(camera_fov_deg),
            center_angle_deg=float(center_angle_deg),
            navigation_mode=navigation_mode,
            overlay_applied=bool(include_debug_overlay),
        )

    def get_status(self) -> dict[str, Any]:
        return {
            "configured": True,
            "source": str(self.index_or_path),
            "live": True,
            "provider": "opencv",
            "width": self.width,
            "height": self.height,
            "fps": self.fps,
            "is_open": bool(self.capture.isOpened()),
        }

    def release(self) -> None:
        if getattr(self, "capture", None) is not None:
            self.capture.release()


def build_camera_provider(
    *,
    camera_index_or_path: Optional[str] = None,
    camera_width: int = DEFAULT_CAMERA_WIDTH,
    camera_height: int = DEFAULT_CAMERA_HEIGHT,
    camera_fps: int = DEFAULT_CAMERA_FPS,
    camera_mock_image: Optional[str] = None,
) -> CameraProvider:
    if camera_index_or_path:
        return OpenCVCameraProvider(
            _parse_index_or_path(camera_index_or_path),
            width=camera_width,
            height=camera_height,
            fps=camera_fps,
        )
    if camera_mock_image:
        return StaticImageCameraProvider(camera_mock_image, source_label="mock-file")
    return UnavailableCameraProvider()
