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
# 'run_once': capture exactly one image then exit (recommended for OpenMV IDE)
# 'button_or_serial': keep running and capture on board button/serial Enter
TRIGGER_MODE = "run_once"
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
    is_abs = path.startswith("/")
    cur = "/" if is_abs else ""
    for p in parts:
        if is_abs:
            if cur == "/":
                cur = "/" + p
            else:
                cur = cur + "/" + p
        else:
            if cur:
                cur = cur + "/" + p
            else:
                cur = p
        try:
            os.mkdir(cur)
        except OSError:
            pass


def can_write(dir_path):
    test_file = dir_path + "/.omv_write_test.tmp"
    try:
        f = open(test_file, "w")
        f.write("ok")
        f.close()
        os.remove(test_file)
        return True
    except Exception:
        return False


def resolve_save_dir(user_path):
    candidates = [user_path]
    if not user_path.startswith("/"):
        candidates.append("/flash/" + user_path)
        candidates.append("/sd/" + user_path)

    for d in candidates:
        try:
            ensure_dir(d)
            if can_write(d):
                return d
        except Exception:
            pass

    raise OSError(
        "No writable path. Set SAVE_DIR to '/flash/calib_photos_manual' or '/sd/calib_photos_manual'."
    )


def next_image_index(save_dir):
    try:
        names = os.listdir(save_dir)
    except Exception:
        return 0

    idx = 0
    while True:
        target = "img_%03d.jpg" % idx
        exists = False
        for n in names:
            if n == target:
                exists = True
                break
        if not exists:
            return idx
        idx += 1


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

save_dir = resolve_save_dir(SAVE_DIR)

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

if TRIGGER_MODE == "run_once":
    print("Mode: run_once")
    idx = next_image_index(save_dir)
    filename = "%s/img_%03d.jpg" % (save_dir, idx)
    img = sensor.snapshot()
    img.save(filename, quality=JPEG_QUALITY)
    print("saved:", filename)
    print("Done. Move target and click Run again for next photo.")
    led_g.on()
    time.sleep_ms(120)
    led_g.off()
else:
    print("Mode: button_or_serial")
    print("Trigger method:")
    print("1) Press board user button (if available)")
    print("2) Or press Enter in IDE terminal")
    print("3) Ctrl+C to stop")

if TRIGGER_MODE != "run_once":
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
            idx = next_image_index(save_dir)
            filename = "%s/img_%03d.jpg" % (save_dir, idx)
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
