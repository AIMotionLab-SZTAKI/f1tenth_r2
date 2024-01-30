from external_dependencies.aimotion_f1tenth_utils.aimotion_f1tenth_utils.communicaton.TCPServer import TCPServer
from PyQt5.QtCore import QThread

class TCP_Server_process(QThread):
    def __init__(self, host, port, message_callback=None):
            super(TCP_Server_process, self).__init__()
            self.tcp_server = TCPServer(
            host = host,
            port= port,
            message_callback= message_callback
        )
    def run(self):
        self.tcp_server.start()   
