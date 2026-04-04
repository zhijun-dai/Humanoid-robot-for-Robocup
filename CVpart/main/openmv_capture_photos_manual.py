import sensor  # type: ignore
import time
import os
import pyb  # type: ignore
from pyb import LED  # type: ignore

# -------------------- User settings --------------------
PIXFORMAT = sensor.RGB565
FRAMESIZE = sensor.QVGA
MAX_IMAGES = 80
JPEG_QUALITY = 95
# Set one fixed save path here.
# Examples:
#   "calib_photos_manual"            -> save under current filesystem root/workdir
#   "/flash/calib_photos_manual"     -> force internal flash
#   "/sd/calib_photos_manual"        -> force SD card
SAVE_DIR = "calib_photos_manual"
DEBOUNCE_MS = 250
WARMUP_MS = 2000
# ------------------------------------------------------


def ensure_dir(path):
    parts = [p for p in path.split("/") if p]
    cur = ""
    for p in parts:
        cur += "/" + p
        try:
            os.mkdir(cur)
        except OSError:
            pass


def read_switch(sw):
    if sw is None:
        return False
    try:
        return bool(sw())
    except Exception:
        try:
            return bool(sw.value())
        except Exception:
            return False


sensor.reset()
sensor.set_pixformat(PIXFORMAT)
sensor.set_framesize(FRAMESIZE)
sensor.skip_frames(time=WARMUP_MS)

# For calibration photos, disable auto controls for consistency.
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)

save_dir = SAVE_DIR
ensure_dir(save_dir)

led_g = LED(2)
led_b = LED(3)
usb = pyb.USB_VCP()

sw = None
try:
    sw = pyb.Switch()
except Exception:
    sw = None

print("Manual capture start")
print("save_dir:", save_dir)
print("Trigger method:")
print("1) Press board user button (if available)")
print("2) Or press Enter in IDE terminal")
print("3) Ctrl+C to stop")

count = 0
last_trigger_ms = time.ticks_ms()
clock = time.clock()

while True:
    clock.tick()
    _ = sensor.snapshot()

    now = time.ticks_ms()
    trigger = False

    if read_switch(sw):
        trigger = True

    if usb.any():
        _ = usb.read()  # clear input buffer
        trigger = True

    if trigger and (time.ticks_diff(now, last_trigger_ms) >= DEBOUNCE_MS):
        filename = "%s/img_%03d.jpg" % (save_dir, count)
        img = sensor.snapshot()
        img.save(filename, quality=JPEG_QUALITY)
        print("saved:", filename)

        led_g.on()
        time.sleep_ms(80)
        led_g.off()

        count += 1
        last_trigger_ms = now

        if count >= MAX_IMAGES:
            print("Reached MAX_IMAGES:", MAX_IMAGES)
            break

    if (count % 5) == 0:
        led_b.toggle()

print("Manual capture done, total:", count)
print("FPS:", clock.fps())
