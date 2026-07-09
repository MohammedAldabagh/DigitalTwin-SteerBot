"""
G29 Steering Wheel Detection Tool

Run this to check if your Logitech G29 wheel is properly connected.
It scans all USB devices and shows you what it finds.

Useful for troubleshooting when the wheel isn't being recognized.
"""
import hid

print("Scanning HID devices...\n")

found = False

for d in hid.enumerate():
    if d["vendor_id"] == 0x046d:  # Logitech vendor ID
        print("Found Logitech device:")
        for k, v in d.items():
            print(f"  {k}: {v}")
        print()
        found = True

if not found:
    print("No Logitech devices found. Is the G29 connected?")
