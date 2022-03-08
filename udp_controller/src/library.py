from select import select
from socket import socket


def receive_data(sock: socket, length: int = 512, timeout: float = 0.00005):
    while True:
        read_fds, _, _ = select([sock], [], [], timeout)
        if sock in read_fds:
            return sock.recvfrom(length)
