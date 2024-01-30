import socket
def get_ip_address():

    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip_address =  s.getsockname()[0]
        s.close()
    except:
        hostname = socket.gethostname()
        ip_address = socket.gethostbyname(hostname)
    return ip_address