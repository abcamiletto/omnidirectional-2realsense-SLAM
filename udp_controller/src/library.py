from select import select


def receive_data(sock, length=512, timeout=0.0005):
    read_fds, _, _ = select([sock], [], [], timeout)
    data = None
    if sock in read_fds:
        print('Incoming data')
        data, _ = sock.recvfrom(length)
    return data
