import contextlib
from io import StringIO
from pathlib import Path
import socket
import tempfile
import threading
import unittest
from unittest.mock import patch

import yaml

from host.ros_arcore_ai import ai_rviz_config,cancellable_probe_lines,main


class RosAiTests(unittest.TestCase):
    def test_user_can_start_after_more_than_five_minutes(self):
        class Client:
            def __enter__(self):return self
            def __exit__(self,*args):pass
            def settimeout(self,value):pass
            def recv(self,size):return next(self.chunks)
            chunks=iter((b'{"type":"pose"}\n',b''))
        stop=threading.Event()
        now=[0];attempts=[0]
        def connect_after_delay(*args,**kwargs):
            now[0]+=301;attempts[0]+=1
            if attempts[0]==1:raise ConnectionRefusedError()
            return Client()
        # The successful connection is at simulated second 602, after expiry.
        with patch('host.ros_arcore_ai.time.monotonic',side_effect=lambda:now[0]), \
             patch('host.ros_arcore_ai.socket.create_connection',
                   side_effect=connect_after_delay) as connect, \
             patch.object(stop,'wait',return_value=False):
            received=list(cancellable_probe_lines(8766,stop))
        self.assertEqual(received,['{"type":"pose"}\n'])
        self.assertEqual(connect.call_count,2)
        self.assertEqual(now[0],602)

    def test_optional_startup_timeout_remains_available(self):
        with patch('host.ros_arcore_ai.time.monotonic',side_effect=[0,4]):
            with self.assertRaisesRegex(TimeoutError,'did not start'):
                list(cancellable_probe_lines(8766,threading.Event(),startup_seconds=3))

    def test_waiting_for_user_is_cancellable_before_first_packet(self):
        stop=threading.Event()
        with patch('host.ros_arcore_ai.socket.create_connection',side_effect=ConnectionRefusedError()), \
             patch.object(stop,'wait',side_effect=lambda seconds:stop.set()):
            self.assertEqual(list(cancellable_probe_lines(8766,stop)),[])

    def test_active_stream_still_rejects_twenty_seconds_without_data(self):
        class Client:
            def __enter__(self):return self
            def __exit__(self,*args):pass
            def settimeout(self,value):pass
            def recv(self,size):
                if not hasattr(self,'sent'):
                    self.sent=True
                    return b'{"type":"pose"}\n'
                raise socket.timeout()
        with patch('host.ros_arcore_ai.socket.create_connection',return_value=Client()), \
             patch('host.ros_arcore_ai.time.monotonic',side_effect=[0,0,21]):
            with self.assertRaisesRegex(TimeoutError,'idle for 20 seconds'):
                list(cancellable_probe_lines(8766,threading.Event()))

    def test_rviz_has_durable_ai_cloud_path_and_current_pose(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'ai.rviz';ai_rviz_config(p)
            cfg=yaml.safe_load(p.read_text())
            self.assertEqual(cfg['Visualization Manager']['Global Options']['Fixed Frame'],'arcore_map')
            shown={d.get('Topic',{}).get('Value'):d for d in cfg['Visualization Manager']['Displays'] if d.get('Enabled')}
            for topic in ('/phone/arcore_ai_map','/phone/arcore_ai_path','/phone/arcore_ai_pose'):
                self.assertIn(topic,shown)
                self.assertEqual(shown[topic]['Topic']['Durability Policy'],'Transient Local')

    def test_fifo_cannot_select_live_transport(self):
        with patch('sys.argv',['ros_arcore_ai','--queue-policy','fifo','--output','work/not-created',
                               '--repository','work/local-model','--checkpoint','data/local-weights']), \
             patch('host.ros_arcore_ai.load_model') as model,contextlib.redirect_stderr(StringIO()):
            with self.assertRaises(SystemExit) as exc:main()
            self.assertEqual(exc.exception.code,2);model.assert_not_called()

    def test_socket_reader_can_cancel_while_phone_sends_no_more_packets(self):
        stop=threading.Event();ready=threading.Event();finished=threading.Event()
        with socket.socket() as listener:
            listener.bind(('127.0.0.1',0));listener.listen(1)
            port=listener.getsockname()[1]
            def serve():
                client,_=listener.accept()
                with client:
                    client.sendall(b'{"type":"pose"}\n')
                    ready.wait(3)
            server=threading.Thread(target=serve,daemon=True);server.start()
            received=[]
            def read():
                for line in cancellable_probe_lines(port,stop,startup_seconds=3):
                    received.append(line);ready.set();stop.set()
                finished.set()
            reader=threading.Thread(target=read,daemon=True);reader.start()
            self.assertTrue(finished.wait(3));reader.join(1);server.join(1)
            self.assertEqual(received,['{"type":"pose"}\n'])


if __name__=='__main__':unittest.main()
