# post-build: save a flashable (merged, flash at 0x0) image as <env>_<timestamp>.bin in the repo root
Import("env")
import datetime, glob, os, subprocess

def save_bin(target, source, env):
    root = env.subst("$PROJECT_DIR")
    name = env.subst("$PIOENV")
    out = os.path.join(root, "%s_%s.bin" % (name, datetime.datetime.now().strftime("%Y%m%d-%H%M%S")))
    esptool = os.path.join(env.PioPlatform().get_package_dir("tool-esptoolpy"), "esptool.py")
    images = []
    for img in env.Flatten(env.get("FLASH_EXTRA_IMAGES", [])):
        images.append(env.subst(img))
    images += [env.subst("$ESP32_APP_OFFSET"), str(target[0])]
    mcu = env.BoardConfig().get("build.mcu", "esp32")
    cmd = [env.subst("$PYTHONEXE"), esptool, "--chip", mcu, "merge_bin", "-o", out,
           "--flash_mode", "keep", "--flash_freq", "keep", "--flash_size", "keep"] + images
    subprocess.check_call(cmd)
    # keep only the newest image per env
    for old in glob.glob(os.path.join(root, name + "_????????-??????.bin")):
        if old != out:
            os.remove(old)
    print("Saved " + out)

env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", save_bin)
