Import("env")
import os
import re

USB_MANUFACTURER = "Reiser_Lab"
USB_PRODUCT = "G6_Arena"


def _char_array(s):
    return "{" + ",".join("'" + c + "'" for c in s) + "}"


def patch_usb_desc(*args, **kwargs):
    path = os.path.join(
        env.PioPlatform().get_package_dir("framework-arduinoteensy"),
        "cores", "teensy4", "usb_desc.h",
    )
    with open(path) as f:
        content = f.read()

    # Target only the USB_SERIAL block (flat #if/#elif chain; ends at first #elif)
    m = re.search(r"(#if defined\(USB_SERIAL\)\n)(.*?)(?=\n#elif)", content, re.DOTALL)
    if not m:
        print("WARNING patch_usb_strings.py: USB_SERIAL block not found in usb_desc.h")
        return

    block = m.group(2)
    patched = block
    patched = re.sub(r"(#define MANUFACTURER_NAME\s+)\{[^}]+\}", r"\g<1>" + _char_array(USB_MANUFACTURER), patched)
    patched = re.sub(r"(#define MANUFACTURER_NAME_LEN\s+)\d+", r"\g<1>" + str(len(USB_MANUFACTURER)), patched)
    patched = re.sub(r"(#define PRODUCT_NAME\s+)\{[^}]+\}", r"\g<1>" + _char_array(USB_PRODUCT), patched)
    patched = re.sub(r"(#define PRODUCT_NAME_LEN\s+)\d+", r"\g<1>" + str(len(USB_PRODUCT)), patched)

    if patched != block:
        new_content = content[: m.start(2)] + patched + content[m.end(2) :]
        with open(path, "w") as f:
            f.write(new_content)
        print(
            f"patch_usb_strings: patched usb_desc.h "
            f"(manufacturer='{USB_MANUFACTURER}', product='{USB_PRODUCT}')"
        )


env.AddPreAction("$BUILD_DIR/FrameworkArduino/usb_desc.c.o", patch_usb_desc)
