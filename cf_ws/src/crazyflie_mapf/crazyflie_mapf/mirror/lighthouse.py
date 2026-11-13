"""Upload a Lighthouse system file (base-station geometry + calibration) to Crazyflies.

    python3 -m crazyflie_mapf.mirror.lighthouse --upload lighthouse_system.yaml radio://0/80/2M/E7E7E7E7E1 ...

The file is the one cfclient's Lighthouse tab saves ("Save system config"). The data is written to
each Crazyflie's RAM and persisted to its flash. Motors are never commanded. The Crazyradio must
not be in use by a running server. Prints one line: UPLOAD {uri: "ok" | error}.
"""

import argparse
import json
import threading


def upload(uri, system_file, timeout):
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
    from cflib.localization import LighthouseConfigWriter
    done, result = threading.Event(), {}

    def stored(success):
        result['ok'] = success
        done.set()
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='/tmp/cfcache')) as scf:
        LighthouseConfigWriter(scf.cf).write_and_store_config_from_file(stored, system_file)
        if not done.wait(timeout):
            raise TimeoutError(f'{timeout:.0f}초 안에 저장 완료 응답 없음')
    if not result['ok']:
        raise RuntimeError('기체가 일부 기지국 데이터 저장에 실패했다고 응답')


def upload_with_timeout(uri, system_file, timeout):
    """cflib's connect has no timeout and can block forever on a flaky link."""
    result = {}

    def run():
        try:
            upload(uri, system_file, timeout)
            result['value'] = 'ok'
        except Exception as exc:
            result['value'] = str(exc) or type(exc).__name__
    thread = threading.Thread(target=run, daemon=True)
    thread.start()
    thread.join(timeout + 5)
    return result.get('value', f'{timeout:.0f}초 안에 연결 안 됨 (배터리 부족·링크 불안정일 수 있음)')


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--upload', required=True, metavar='FILE', help='cfclient Lighthouse system config (.yaml)')
    parser.add_argument('--timeout', type=float, default=20.0, help='seconds per robot')
    parser.add_argument('uris', nargs='+')
    opt = parser.parse_args(args)
    import cflib.crtp
    from cflib.localization import LighthouseConfigFileManager
    LighthouseConfigFileManager.read(opt.upload)  # fail early on a malformed file
    cflib.crtp.init_drivers()
    print('UPLOAD ' + json.dumps({uri: upload_with_timeout(uri, opt.upload, opt.timeout) for uri in opt.uris},
                                 ensure_ascii=False), flush=True)
    return 0


if __name__ == '__main__':
    import os
    import sys
    code = main()
    sys.stdout.flush()
    os._exit(code or 0)  # a timed-out cflib link leaves non-daemon threads behind
