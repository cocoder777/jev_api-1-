"""Send the sample trackingMode message to the local UDP decision service."""

import argparse
import json
import socket
import time
from typing import Any, Dict

from decision_service import (
    ALGORITHM_TASK_ID,
    ALGORITHM_STATUS_ID,
    SPEED_COURSE_ID,
    analyze_task,
    build_json_packet,
    decode_speed_course_packet,
    parse_header,
)


DEFAULT_TARGET_HOST = "127.0.0.1"
DEFAULT_TARGET_PORT = 18014
DEFAULT_LOCAL_HOST = "127.0.0.1"
DEFAULT_LOCAL_PORT = 7612


def sample_information() -> Dict[str, Any]:
    """Return the tracking task shown in the captured UDP packet."""

    return {
        "simId": "USV-07",
        "id": "b0bb189b-dc19-419c-8cd7-09e66646a25a",
        "code": "trackingMode",
        "params": {
            "platCode": 1010,
            "planRouteNum": 0,
            "currentRoute": 0,
            "planWaypointNum": 3,
            "planWaypointList": [
                {
                    "routeLon": 120.59707595635746,
                    "routeLat": 30.587895210827043,
                    "routeSpeed": 51.444444444444445,
                    "routeRadius": 20.0,
                },
                {
                    "routeLon": 120.58059646416967,
                    "routeLat": 30.43171972235737,
                    "routeSpeed": 51.444444444444445,
                    "routeRadius": 20.0,
                },
                {
                    "routeLon": 120.89919997979501,
                    "routeLat": 30.609172419875108,
                    "routeSpeed": 51.444444444444445,
                    "routeRadius": 20.0,
                },
            ],
        },
    }


def build_packet(information: Dict[str, Any]) -> bytes:
    """Build the 16-byte-header UDP packet visible in the screenshot."""

    return build_json_packet(information, ALGORITHM_TASK_ID)


def send_once(args: argparse.Namespace) -> None:
    packet = build_packet(sample_information())
    target = (args.target_host, args.target_port)

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as client:
        client.bind((args.local_host, args.local_port))
        client.settimeout(args.timeout)
        local = client.getsockname()
        print("本地模拟端口：{}:{}".format(*local))
        print("目标服务端口：{}:{}".format(*target))
        print("发送长度：{} 字节".format(len(packet)))
        print("发送文本：{}".format(packet[16:].decode("utf-8")))
        print("发送 HEX：{}".format("-".join("{:02X}".format(b) for b in packet)))

        client.sendto(packet, target)
        final_status_received = False
        while not final_status_received:
            try:
                response, remote = client.recvfrom(65_507)
            except socket.timeout:
                print("等待回复超时：请确认 decision_service.py 已启动且端口一致。")
                return

            print("收到回复：{}:{}，{} 字节".format(remote[0], remote[1], len(response)))
            header = parse_header(response)
            secondary_id = header["secondaryInfoUnitId"]
            if secondary_id == SPEED_COURSE_ID:
                print(
                    "航速航向：{}".format(
                        json.dumps(
                            decode_speed_course_packet(response), ensure_ascii=False
                        )
                    )
                )
            elif secondary_id == ALGORITHM_STATUS_ID:
                status = analyze_task(response)
                print("任务状态：{}".format(json.dumps(status, ensure_ascii=False)))
                final_status_received = status.get("status") in (2, 3)
            else:
                print("未知回复标识：0x{:04X}".format(secondary_id))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="模拟 USV UDP 状态信息发送端")
    parser.add_argument("--target-host", default=DEFAULT_TARGET_HOST)
    parser.add_argument("--target-port", type=int, default=DEFAULT_TARGET_PORT)
    parser.add_argument("--local-host", default=DEFAULT_LOCAL_HOST)
    parser.add_argument("--local-port", type=int, default=DEFAULT_LOCAL_PORT)
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument(
        "--interval",
        type=float,
        default=0.0,
        help="大于 0 时按指定秒数循环发送；默认为只发送一次",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    try:
        while True:
            send_once(args)
            if args.interval <= 0:
                break
            time.sleep(args.interval)
    except KeyboardInterrupt:
        print("模拟发送已停止。")
    except OSError as exc:
        raise SystemExit("UDP 模拟发送失败：{}".format(exc))


if __name__ == "__main__":
    main()
