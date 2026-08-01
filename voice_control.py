"""
Voice control for SO-101 using sounddevice + whisper.
Push-to-talk: press → stream audio to buffer, release → transcribe + execute.
"""
import logging
from collections.abc import Callable

import numpy as np

log = logging.getLogger(__name__)

try:
    import sounddevice as sd
    _SD_AVAILABLE = True
except ImportError:
    _SD_AVAILABLE = False

try:
    import whisper
    _WHISPER_AVAILABLE = True
except ImportError:
    _WHISPER_AVAILABLE = False

_COMMAND_MAP = {
    "home":         {"cmd": "home"},
    "go home":      {"cmd": "home"},
    "open":         {"cmd": "gripper", "value": 100},
    "open gripper": {"cmd": "gripper", "value": 100},
    "close":        {"cmd": "gripper", "value": 0},
    "close gripper":{"cmd": "gripper", "value": 0},
    "stop":         {"cmd": "estop"},
    "estop":        {"cmd": "estop"},
    "record":       {"cmd": "record"},
    "start record": {"cmd": "record"},
    "stop record":  {"cmd": "stop_record"},
    "zero":         {"cmd": "zero"},
    "wave":         {"cmd": "wave"},
}

class VoiceController:
    """Push-to-talk: start_recording() buffers audio, stop_and_process() transcribes + executes."""

    def __init__(self, sample_rate: int | None = None):
        if sample_rate is None and _SD_AVAILABLE:
            try:
                sample_rate = int(sd.query_devices(kind='input')['default_samplerate'])
            except Exception:
                sample_rate = 16000
        elif sample_rate is None:
            sample_rate = 16000
        self._sample_rate = sample_rate
        self._model: object | None = None
        self._command_callback: Callable | None = None
        self._recording = False
        self._stream: object | None = None
        self._buffer: list[np.ndarray] = []

    def set_command_callback(self, cb: Callable):
        self._command_callback = cb

    @property
    def active(self) -> bool:
        return self._recording

    def start(self) -> bool:
        """Load whisper model (lazy). Does NOT start recording."""
        if not _SD_AVAILABLE:
            log.error("sounddevice not installed"); return False
        if _WHISPER_AVAILABLE and self._model is None:
            log.info("Loading whisper model (tiny)...")
            try:
                self._model = whisper.load_model("tiny")
                log.info("Whisper model loaded")
            except Exception as e:
                log.warning("Whisper load failed: %s", e)
                self._model = None
        log.info("Voice controller ready")
        return True

    def stop(self):
        """Full stop — clean up stream + model."""
        self._recording = False
        if self._stream:
            self._stream.stop()
            self._stream.close()
            self._stream = None
        self._model = None
        log.info("Voice controller stopped")

    def start_recording(self):
        """Begin streaming audio into an internal buffer."""
        if not _SD_AVAILABLE:
            log.error("sounddevice not available"); return
        if self._recording:
            return
        self._buffer = []
        self._recording = True

        def callback(indata, frames, time_info, status):
            if status:
                log.warning("Audio status: %s", status)
            if self._recording:
                self._buffer.append(indata.copy())

        try:
            self._stream = sd.InputStream(
                samplerate=self._sample_rate, channels=1,
                dtype=np.float32, callback=callback)
            self._stream.start()
            log.info("Recording started (PTT)")
        except Exception as e:
            log.error("Failed to start audio stream: %s", e)
            self._recording = False

    def stop_and_process(self) -> str | None:
        """Stop recording, transcribe buffer, execute command. Returns recognized text."""
        self._recording = False
        if self._stream:
            self._stream.stop()
            self._stream.close()
            self._stream = None

        if not self._buffer:
            log.info("No audio captured")
            return None

        audio = np.concatenate(self._buffer).flatten()
        self._buffer = []

        text = self._transcribe(audio)
        if text:
            log.info("Heard: '%s'", text)
            self._execute(text)
        else:
            log.info("No speech detected")
        return text

    def _transcribe(self, audio: np.ndarray) -> str | None:
        if self._model:
            try:
                result = self._model.transcribe(audio, language="en")
                return result.get("text", "").strip().lower()
            except Exception as e:
                log.warning("Whisper error: %s", e)
        return None

    def _execute(self, text: str):
        text = text.lower().strip()
        for phrase, action in _COMMAND_MAP.items():
            if phrase in text:
                if self._command_callback:
                    self._command_callback(action)
                return
        log.info("No command matched: '%s'", text)
