import hid

VID = 0x046d  # Logitech
PID = 0xc24f  # G29

print("=== hid.enumerate(VID, PID) ===")
devices = hid.enumerate(VID, PID)
if not devices:
    print("Kein Gerät mit dieser VID/PID gefunden.")
else:
    for d in devices:
        print("----")
        for k, v in d.items():
            print(f"{k}: {v}")

print("\n=== Versuche open(VID, PID) ===")
dev = hid.device()
try:
    dev.open(VID, PID)
    print("✅ open(VID, PID) OK")
except Exception as e:
    print(f"❌ open(VID, PID) fehlgeschlagen: {e}")
    dev = None

if dev is not None:
    try:
        print("=== Sende Test-Write (Force-Off) ===")
        dev.write([0x00, 0xF3, 0, 0, 0, 0, 0, 0])
        print("✅ write() hat nicht gecrasht")
    except Exception as e:
        print(f"❌ write() fehlgeschlagen: {e}")
    dev.close()

