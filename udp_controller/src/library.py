from select import select


def receive_data(sock, length=512, timeout=0.00005):
    while True:
        read_fds, _, _ = select([sock], [], [], timeout)
        if sock in read_fds:
            data, _ = sock.recvfrom(length)
            return data
