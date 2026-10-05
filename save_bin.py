# post-build: save a flashable (merged, flash at 0x0) image as <env>_<timestamp>.bin in the repo root
Import("env")
import datetime, glob, os, shutil, subprocess

def save_bin(target, source, env):
    root = env.subst("$PROJECT_DIR")
    name = env.subst("$PIOENV")
    stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    out = os.path.join(root, "%s_%s.bin" % (name, stamp))
    esptool = os.path.join(env.PioPlatform().get_package_dir("tool-esptoolpy"), "esptool.py")
    images = []
    for img in env.Flatten(env.get("FLASH_EXTRA_IMAGES", [])):
        images.append(env.subst(img))
    images += [env.subst("$ESP32_APP_OFFSET"), str(target[0])]
    mcu = env.BoardConfig().get("build.mcu", "esp32")
    cmd = [env.subst("$PYTHONEXE"), esptool, "--chip", mcu, "merge_bin", "-o", out,
           "--flash_mode", "keep", "--flash_freq", "keep", "--flash_size", "keep"] + images
    subprocess.check_call(cmd)
    if name == "s3cam-rover-tank":
        framework = env.PioPlatform().get_package_dir("framework-arduinoespressif32")
        webflash_images = {
            "bootloader": ("$BUILD_DIR/bootloader.bin", "0x0"),
            "partitions": ("$BUILD_DIR/partitions.bin", "0x8000"),
            "boot_app0": (os.path.join(framework, "tools/partitions/boot_app0.bin"), "0xe000"),
            "firmware": (str(target[0]), env.subst("$ESP32_APP_OFFSET")),
        }
        for image_name, (image_path, _) in webflash_images.items():
            src = env.subst(image_path)
            dst = os.path.join(root, "%s_%s_%s.bin" % (name, stamp, image_name))
            shutil.copyfile(src, dst)
        print("Webflasher offsets: " + ", ".join(
            "%s=%s" % (image_name, offset) for image_name, (_, offset) in webflash_images.items()))
    # keep only the newest image per env
    for old in glob.glob(os.path.join(root, name + "_????????-??????.bin")):
        if old != out:
            os.remove(old)
    if name == "s3cam-rover-tank":
        for old in glob.glob(os.path.join(root, name + "_????????-??????_*.bin")):
            if not old.startswith(os.path.join(root, name + "_" + stamp + "_")):
                os.remove(old)
    print("Saved " + out)

env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", save_bin)
