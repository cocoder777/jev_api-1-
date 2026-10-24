"""Multicast subscription lifecycle tests without network traffic."""

import socket
import unittest
from unittest.mock import MagicMock, Mock, call, patch

from server import decision_service


class MulticastSubscriptionTests(unittest.TestCase):
    group = "239.255.0.10"
    port = 18010
    interface = "192.168.3.100"
    datagram = (b"situation", ("192.168.3.201", 18010))

    def make_socket(self):
        sub = MagicMock()
        sub.__enter__.return_value = sub
        return sub

    def test_keeps_membership_and_ignores_invalid_datagrams(self):
        sub = self.make_socket()
        sub.recvfrom.side_effect = [self.datagram, self.datagram, KeyboardInterrupt]
        situations = Mock()
        state = {"platformId": 7}
        with (
            patch.object(decision_service.socket, "socket", return_value=sub) as factory,
            patch.object(decision_service, "decode_situation_packet",
                         side_effect=[ValueError("invalid packet"), state]),
            patch.object(decision_service.time, "sleep") as sleep,
            self.assertRaises(KeyboardInterrupt),
        ):
            decision_service.subscribe_situations(
                situations, self.group, self.port, self.interface,
            )
        factory.assert_called_once()
        sub.bind.assert_called_once_with(("", self.port))
        membership = socket.inet_aton(self.group) + socket.inet_aton(self.interface)
        self.assertEqual(sub.setsockopt.call_args_list, [
            call(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1),
            call(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, membership),
        ])
        situations.update.assert_called_once_with(state)
        sleep.assert_not_called()

    def test_rejoins_after_socket_failures_and_resumes_receiving(self):
        for failure in ("bind", "join", "receive"):
            with self.subTest(failure=failure):
                failed = self.make_socket()
                recovered = self.make_socket()
                error = OSError("network unavailable")
                if failure == "bind":
                    failed.bind.side_effect = error
                elif failure == "join":
                    failed.setsockopt.side_effect = [None, error]
                else:
                    failed.recvfrom.side_effect = error
                recovered.recvfrom.side_effect = [self.datagram, KeyboardInterrupt]
                situations = Mock()
                state = {"platformId": 7}

                def check_closed_before_retry(_):
                    failed.__exit__.assert_called_once()

                with (
                    patch.object(decision_service.socket, "socket",
                                 side_effect=[failed, recovered]) as factory,
                    patch.object(decision_service, "decode_situation_packet", return_value=state),
                    patch.object(decision_service.time, "sleep",
                                 side_effect=check_closed_before_retry) as sleep,
                    self.assertRaises(KeyboardInterrupt),
                ):
                    decision_service.subscribe_situations(
                        situations, self.group, self.port, self.interface,
                    )
                self.assertEqual(factory.call_count, 2)
                sleep.assert_called_once_with(decision_service.MULTICAST_RETRY_INTERVAL)
                membership = socket.inet_aton(self.group) + socket.inet_aton(self.interface)
                recovered.setsockopt.assert_any_call(
                    socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, membership,
                )
                situations.update.assert_called_once_with(state)

    def test_receipt_logs_are_throttled_but_every_packet_updates_the_store(self):
        sub = self.make_socket()
        sub.recvfrom.side_effect = [self.datagram] * 5 + [KeyboardInterrupt]
        situations = Mock()
        states = [dict(platformId=platform) for platform in (7, 8, 7, 9, 7)]
        with (
            patch.object(decision_service.socket, 'socket', return_value=sub),
            patch.object(decision_service, 'decode_situation_packet', side_effect=states),
            patch.object(decision_service.time, 'monotonic', side_effect=[0., 1., 4.99, 5., 5.01]),
            patch.object(decision_service.LOGGER, 'info') as info,
            self.assertRaises(KeyboardInterrupt),
        ):
            decision_service.subscribe_situations(situations, self.group, self.port, self.interface)
        self.assertEqual(situations.update.call_args_list, [call(state) for state in states])
        receipts = [args for args in info.call_args_list if args.args[0].startswith('received situation')]
        self.assertEqual(len(receipts), 2)
        self.assertEqual(receipts[0].args[1:3], (1, [7]))
        self.assertEqual(receipts[1].args[1:3], (3, [7, 8, 9]))


if __name__ == "__main__":
    unittest.main()
