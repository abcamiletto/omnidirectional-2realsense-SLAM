from collections import namedtuple

IP_Config = namedtuple("IP_Config", ["localIP", "localPort", "remoteIP", "remotePort"])

dummy_IP_Config = IP_Config("192.168.0.10",9999,"192.168.0.100",11111)
avatar_IP_Config = IP_Config("10.24.4.13", 9999, "10.24.4.100", 11115)
