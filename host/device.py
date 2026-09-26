"""Cancellable subprocess calls for device setup (no GUI dependency)."""
import subprocess
import time


def select_adb_serial(devices, requested=None):
    """Pin every command and port forward to one authorized ADB transport."""
    if requested:
        if requested not in devices:
            raise RuntimeError(f'Seçilen ADB bağlantısı hazır değil: {requested}')
        return requested
    if not devices:
        raise RuntimeError('Yetkili telefon bulunamadı; USB veya kablosuz hata ayıklama bağlantısını kontrol edin.')
    if len(devices)>1:
        raise RuntimeError('Birden fazla ADB bağlantısı var; PHONE_ADB_SERIAL ile USB veya Wi-Fi bağlantısını seçin.')
    return devices[0]


def timing_args(wide_fps=15):
    """Opt-in supported-rate experiment. Never changes the tele camera settings."""
    if wide_fps == 15:
        return []
    if wide_fps == 30:
        return ['--ei','fpsMin21','30','--ei','fpsMax21','30',
                '--el','frameDurationNs21','33333334']
    raise ValueError('Only15 or30 FPS profiles are supported')


def run_command(command, stop, timeout=10.):
    if stop.is_set():
        raise InterruptedError('Device setup cancelled')
    process = subprocess.Popen(command, stdin=subprocess.DEVNULL,
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    deadline = time.monotonic() + timeout
    try:
        while True:
            if stop.is_set():
                raise InterruptedError('Device setup cancelled')
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError('Device command timed out')
            try:
                stdout, stderr = process.communicate(timeout=min(.1, remaining))
                result = subprocess.CompletedProcess(command, process.returncode, stdout, stderr)
                result.check_returncode()
                return result
            except subprocess.TimeoutExpired:
                pass
    finally:
        if process.poll() is None:
            process.terminate()
            try:
                process.communicate(timeout=1.)
            except subprocess.TimeoutExpired:
                process.kill()
                process.communicate()
