"""List the Crazyflies that answer on the radio right now, matched to config names.

    python3 -m crazyflie_mapf.mirror.scan [--robots hardware/robots.yaml] [--no-info]

1. Each configured URI is pinged directly (reliable), and every channel/rate is scanned
   several times for each configured address and the factory default E7E7E7E7E7 (one pass
   can miss an ack), so a drone on another channel is still found.
2. Each responding drone is connected read-only for battery voltage and detected decks.
Motors are never commanded. The Crazyradio must not be in use by a running server.
"""

import argparse
import time

from ..common.trajectory import config_directory, read_mapping

DEFAULT_ADDRESS = 0xE7E7E7E7E7
DECKS = {"bcActiveMarker": "ActiveMarker", "bcFlow2": "Flow2", "bcLighthouse4": "Lighthouse",
         "bcMultiranger": "Multiranger", "bcAI": "AI", "bcLoco": "Loco"}
POWER = {0: "battery", 1: "charging", 2: "charged", 3: "LOW POWER", 4: "shutdown"}


def configured(path):
    robots = read_mapping(path).get("robots", {})
    result = {}
    for name, robot in robots.items():
        uri = robot.get("uri", "")
        try:
            result[int(uri.rsplit("/", 1)[1], 16)] = (name, uri, robot.get("enabled"))
        except (IndexError, ValueError):
            continue
    return result


def scan(addresses, rounds):
    """Channel scan repeated `rounds` times: {uri: number of rounds it answered}. One pass can miss an ack."""
    import cflib.crtp
    cflib.crtp.init_drivers()
    hits = {}
    for _ in range(rounds):
        for address in addresses:
            for uri, _ in cflib.crtp.scan_interfaces(address):
                if not uri.startswith("radio://"):
                    continue
                if uri.count("/") == 4:  # default address comes back without it
                    uri = f"{uri}/{address:010X}"
                hits[uri] = hits.get(uri, 0) + 1
    return hits


def ping(uris, attempts):
    """Direct null-packet ping per configured URI: {uri: acks out of attempts}. Reliable for known URIs."""
    from cflib.drivers.crazyradio import Crazyradio
    from .detect import parse
    rates = {"250K": Crazyradio.DR_250KPS, "1M": Crazyradio.DR_1MPS, "2M": Crazyradio.DR_2MPS}
    acks = {}
    radios = {}
    try:
        for uri in uris:
            dongle, channel, rate, address = parse(uri)
            radio = radios.get(dongle) or radios.setdefault(dongle, Crazyradio(devid=dongle))
            radio.set_arc(3)
            radio.set_channel(channel)
            radio.set_data_rate(rates[rate])
            radio.set_address(address)
            acks[uri] = sum(1 for _ in range(attempts) if (lambda a: a is not None and a.ack)(radio.send_packet((0xFF,))))
    finally:
        for radio in radios.values():
            radio.close()
    return acks


def info(uri):
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.log import LogConfig
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
    from cflib.crazyflie.syncLogger import SyncLogger
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache="/tmp/cfcache")) as scf:
        param = scf.cf.param
        groups = param.toc.toc
        decks = [label for key, label in DECKS.items() if key in groups.get("deck", {}) and param.get_value(f"deck.{key}") == "1"]
        conf = LogConfig("scan", period_in_ms=100)
        conf.add_variable("pm.vbat", "float")
        conf.add_variable("pm.state", "uint8_t")
        with SyncLogger(scf, conf) as logger:
            for _, data, _ in logger:
                return data["pm.vbat"], POWER.get(data["pm.state"], data["pm.state"]), decks
    return None


def info_with_timeout(uri, seconds):
    """cflib's connect has no timeout and can block forever on a flaky link."""
    import threading
    result = {}

    def run():
        try:
            result["value"] = info(uri)
        except Exception as exc:
            result["error"] = exc
    thread = threading.Thread(target=run, daemon=True)
    thread.start()
    thread.join(seconds)
    if thread.is_alive():
        raise TimeoutError(f"{seconds:.0f}초 안에 응답 없음 (배터리 부족·링크 불안정일 수 있음)")
    if "error" in result:
        raise result["error"]
    return result["value"]


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--robots", default=str(config_directory() / "hardware" / "robots.yaml"))
    parser.add_argument("--no-info", action="store_true", help="skip battery/deck readout")
    parser.add_argument("--attempts", type=int, default=20, help="direct pings per configured URI")
    parser.add_argument("--rounds", type=int, default=3, help="full channel scans")
    parser.add_argument("--info-timeout", type=float, default=10.0, help="seconds per battery/deck readout")
    parser.add_argument("--decks", nargs="+", metavar="URI",
                        help="only print JSON {uri: [decks] | error} for these URIs (used by mirror.launch.py)")
    opt = parser.parse_args(args)

    if opt.decks:
        import json
        import cflib.crtp
        cflib.crtp.init_drivers()
        result = {}
        for uri in opt.decks:
            try:
                result[uri] = info_with_timeout(uri, opt.info_timeout)[2]
            except Exception as exc:
                result[uri] = str(exc)
        print("DECKS " + json.dumps(result, ensure_ascii=False), flush=True)
        return 0

    known = configured(opt.robots)
    addresses = sorted(set(known) | {DEFAULT_ADDRESS})
    started = time.time()
    print(f"스캔 중... 설정 기체 {len(known)}대 직접 확인 후 전 채널 스캔 {opt.rounds}회 (약 20초)", flush=True)
    try:
        acks = ping([uri for _, uri, _ in known.values()], opt.attempts)
        hits = scan(addresses, opt.rounds)
    except Exception as exc:  # no dongle, permission, or the server holds it
        print(f"스캔 실패: {exc}\n동글이 꽂혀 있는지, 실기체 서버(mirror/hardware)가 동글을 쓰고 있지 않은지 확인하세요.")
        return 1
    print(f"설정 파일: {opt.robots}")
    if not opt.no_info:
        print("응답한 기체의 배터리·덱 정보를 읽는 중...", flush=True)
    print(f"확인: 설정 주소 {len(known)}개 직접 {opt.attempts}회 + 전 채널 스캔 {opt.rounds}회 ({time.time() - started:.1f}s)\n")
    rows = []  # (name, uri, evidence)
    for address, (name, uri, _) in sorted(known.items()):
        scanned = hits.pop(uri, 0)
        if acks.get(uri) or scanned:
            rows.append((name, uri, f"직접 {acks.get(uri, 0)}/{opt.attempts}, 스캔 {scanned}/{opt.rounds}"))
    # A drone already answering at its configured URI is also heard on overlapping channels
    # (a 2M link is 2 MHz wide): those extra hits are the same drone, not another one.
    for uri in list(hits):
        address = int(uri.rsplit("/", 1)[1], 16)
        if address in known and acks.get(known[address][1]):
            hits.pop(uri)
    # A single scan ack on an unexpected channel is often noise: confirm it with direct pings.
    confirm = ping(sorted(hits), opt.attempts) if hits else {}
    noise = [uri for uri in hits if confirm.get(uri, 0) < max(3, opt.attempts // 4)]
    for uri in noise:
        hits.pop(uri)
    for uri, count in sorted(hits.items()):
        address = int(uri.rsplit("/", 1)[1], 16)
        count = f"{count}/{opt.rounds}, 직접 {confirm[uri]}/{opt.attempts}"
        if address in known:
            name, config_uri, _ = known[address]
            rows.append((name, uri, f"스캔 {count}  ! 설정은 {config_uri} (채널/속도 다름)"))
        else:
            rows.append(("(설정에 없음)", uri, f"스캔 {count}"))
    if noise:
        print(f"(스캔에서 한 번 잡혔지만 직접 확인에 응답하지 않은 {len(noise)}건은 잡음으로 제외)\n")
    if not rows:
        print("응답하는 Crazyflie가 없습니다. 기체 전원과 주소/채널을 확인하세요.")
        return 0
    failed = set()
    for name, uri, evidence in rows:
        line = f"{name:14s} {uri}   응답 {evidence}"
        if not opt.no_info:
            try:
                vbat, power, decks = info_with_timeout(uri, opt.info_timeout)
                warn = "  ! 충전 필요" if vbat < 3.8 else ""
                line += f"\n{'':14s} 배터리 {vbat:.2f} V ({power}){warn}, 덱: {', '.join(decks) if decks else '없음'}"
            except Exception as exc:
                failed.add(name)
                line += f"\n{'':14s} 연결 실패: {exc}"
        print(line, flush=True)
    names = list(dict.fromkeys(name for name, _, _ in rows if not name.startswith("(") and name not in failed))
    if failed:
        print(f"\n연결 실패로 제외: {', '.join(sorted(failed))} (충전·재부팅 후 다시 scan)")
    if names:
        print(f"\nmirror에 쓰려면: docker/mapf.sh mirror real:={','.join(names)}")
    return 0


if __name__ == "__main__":
    import os
    import sys
    code = main()
    sys.stdout.flush()
    # A timed-out cflib link leaves non-daemon threads behind; do not wait for them.
    os._exit(code or 0)
