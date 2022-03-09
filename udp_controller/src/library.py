from select import select
import time


def receive_data(sock, length=512, timeout=0.00005):
    while True:
        read_fds, _, _ = select([sock], [], [], timeout)
        if sock in read_fds:
            print('Incoming data')
            data, _ = sock.recvfrom(length)
            return data
        time.sleep(0.0025)
