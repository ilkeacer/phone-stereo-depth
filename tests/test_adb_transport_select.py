import unittest

from host.device import select_adb_serial


class AdbTransportSelectTests(unittest.TestCase):
    def test_one_authorized_connection_is_selected(self):
        self.assertEqual(select_adb_serial(['usb-serial']), 'usb-serial')

    def test_wifi_can_be_pinned_while_usb_is_also_attached(self):
        self.assertEqual(select_adb_serial(['usb-serial','192.168.1.2:5555'],
                                           '192.168.1.2:5555'),'192.168.1.2:5555')

    def test_multiple_connections_without_selection_are_rejected(self):
        with self.assertRaisesRegex(RuntimeError,'PHONE_ADB_SERIAL'):
            select_adb_serial(['usb-serial','192.168.1.2:5555'])

    def test_unknown_requested_connection_is_rejected(self):
        with self.assertRaisesRegex(RuntimeError,'hazır değil'):
            select_adb_serial(['usb-serial'],'192.168.1.2:5555')


if __name__=='__main__':unittest.main()
