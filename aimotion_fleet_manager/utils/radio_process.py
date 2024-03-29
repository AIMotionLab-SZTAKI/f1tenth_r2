from PyQt5.QtCore import QThread, QObject, pyqtSignal, pyqtSlot
from crazymocap.radio_streamer import RadioStreamer

class Radio_Worker(QThread):
    def __init__(self, ip, channel, devid, obj_name):
        
        super(Radio_Worker, self).__init__()
        
        self.streamer = RadioStreamer(ip= ip, devid=devid, channel= channel, object_name= obj_name)
        self.running = False
    
    def stop(self):
        self.running = False
    
    def run(self):
        self.running = True
        while self.running:
            self.streamer.send_pose()



    