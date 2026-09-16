import os
import serial.tools.list_ports as list_ports


def _interface_index(port):
    # The interface number is the tail of the USB location, e.g. "...:1.0" for
    # interface 0 (the CLI VCP) and "...:1.2" for the second CDC's data interface.
    location = port.location or ""
    tail = location.rsplit(".", 1)[-1]
    try:
        return int(tail)
    except ValueError:
        return 99


def _cli_port(group):
    # A Flipper in dual-CDC mode exposes two serial ports; the CLI is on
    # interface 0. Pick it (fall back to the first device name).
    if len(group) == 1:
        return group[0]
    return sorted(group, key=lambda p: (_interface_index(p), p.device))[0]


# Returns a valid port or None, if it cannot be found
def resolve_port(logger, portname: str = "auto"):
    if portname != "auto":
        return portname
    # Try guessing. A single Flipper can present more than one serial port (the
    # firmware's dual-CDC mode), so group by serial number to count devices, not
    # ports, and use each device's CLI interface.
    by_serial = {}
    for port in list_ports.grep("flip_"):
        by_serial.setdefault(port.serial_number, []).append(port)
    if len(by_serial) == 1:
        flipper = _cli_port(next(iter(by_serial.values())))
        logger.info(f"Using {flipper.serial_number} on {flipper.device}")
        return flipper.device
    elif len(by_serial) == 0:
        logger.error("Failed to find connected Flipper")
    elif len(by_serial) > 1:
        logger.error("More than one Flipper is attached")
    env_path = os.environ.get("FLIPPER_PATH")
    if env_path:
        if os.path.exists(env_path):
            logger.info(f"Using FLIPPER_PATH from environment: {env_path}")
            return env_path
