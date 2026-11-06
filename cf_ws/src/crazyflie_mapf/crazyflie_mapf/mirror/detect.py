"""Find which configured Crazyflies answer on the radio, before the real server opens the dongles."""

import re

URI = re.compile(r"^radio://(\d+)/(\d+)/(250K|1M|2M)/([0-9A-Fa-f]{10})$")


def parse(uri):
    match = URI.match(uri)
    if not match:
        raise ValueError(f"mirror only supports radio://<dongle>/<channel>/<rate>/<10 hex address>: {uri}")
    dongle, channel, rate, address = match.groups()
    return int(dongle), int(channel), rate, bytes.fromhex(address)


def reachable(uris, attempts=5, log=print):
    """Return the subset of {name: uri} whose Crazyflie acknowledges a null packet."""
    try:
        from cflib.drivers.crazyradio import Crazyradio
    except ImportError as exc:
        log(f"cflib을 불러올 수 없어 실기체를 찾지 않습니다: {exc}")
        return {}
    rates = {"250K": Crazyradio.DR_250KPS, "1M": Crazyradio.DR_1MPS, "2M": Crazyradio.DR_2MPS}
    by_dongle = {}
    for name, uri in uris.items():
        dongle, channel, rate, address = parse(uri)
        by_dongle.setdefault(dongle, []).append((name, channel, rates[rate], address))
    found = {}
    for dongle, robots in sorted(by_dongle.items()):
        try:
            radio = Crazyradio(devid=dongle)
        except Exception as exc:  # no dongle, permissions, or already opened
            log(f"Crazyradio {dongle}를 열 수 없습니다 ({exc}). 해당 동글의 기체는 시뮬레이션만 합니다.")
            continue
        try:
            radio.set_arc(3)
            for name, channel, rate, address in robots:
                radio.set_channel(channel)
                radio.set_data_rate(rate)
                radio.set_address(address)
                for _ in range(attempts):
                    ack = radio.send_packet((0xFF,))
                    if ack is not None and ack.ack:
                        found[name] = uris[name]
                        break
        finally:
            radio.close()
    return found
