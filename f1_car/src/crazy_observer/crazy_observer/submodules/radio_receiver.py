import array
from .crazyradio import Crazyradio
import traceback
import time


class RadioReciever:
    """Class to encapsulate the receiving crazyradio."""
    def __init__(self, devid, mode=Crazyradio.MODE_PRX, channel=100, data_rate=Crazyradio.DR_250KPS):
        # initialize radio
        self.radio = Crazyradio(devid=devid) # if you only have 1 dongle in your PC, this must be 0
        self.radio.set_channel(channel)
        self.radio.set_data_rate(data_rate)
        self.radio.set_mode(mode)
        # make sure radio gets released before the program quits

    def receive(self):
        buffer = array.array("f", [0.0, 0.0, 0.0, 0.0, 0.0])
        # radio.receive can return the following:
        # - None, if an exception occurred
        # - An array.array object, if we passed to it an int of the bytes that should be read
        # - An int, signaling how many bytes were read, if we passed to it a buffer array (where it writes what it read)
        rec = self.radio.receive(buffer)  # in our case, we expect an array of 4 floats, so we pass a buffer
        if isinstance(rec, int) and rec == buffer.itemsize * len(buffer):
            return buffer
        else:
            return None

    def close(self):
        # For some weird reason, close() doesn't *actually* close the radio, and the PRX radio still sends back
        # acknowledgements after it gets closed. To avoid this, set it to PTX mode before closing.
        self.radio.set_mode(self.radio.MODE_PTX)
        self.radio.close()

if __name__=="__main__":
    print("Starting RX")
    start_time = time.time()
    receiver = RadioReciever(devid=0)
    try:
        while True:
            rec = receiver.receive()
            if rec is not None:
                print(f"{time.time() - start_time}: received {rec}")

    except Exception as exc:
        print(f"Exception: {exc!r}. TRACEBACK:\n")
        print(traceback.format_exc())
        receiver.radio.close()
    input("Press Enter to exit...")
