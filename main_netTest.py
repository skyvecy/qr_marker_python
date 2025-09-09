import Module.Network as net

def init():
    network = net.Network(9001, 9999)
    network.init_server()

if __name__ == "__main__":
    init()
