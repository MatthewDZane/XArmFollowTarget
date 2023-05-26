import ast
import socket
import threading
import carb

class XArmSocket():
    def __init__(self) -> None:
        # sending position data to arm  
        self.txsocket = None
        self.txconn = None
        
        # tracking info
        self.rxsocket = None
        self.rxconn = None

        self.dx = 0
        self.dy = 0

        # threads
        self.txsocket_thread = None
        self.rxsocket_thread = None

    def start_txsocket(self):
        self.txsocket_thread = threading.Thread(target=self.setup_txsocket)
        self.txsocket_thread.start()

    def start_rxsocket(self):
        self.rxsocket_thread = threading.Thread(target=self.setup_rxsocket)
        self.rxsocket_thread.start()

    def setup_txsocket(self):
        if self.txsocket is None:
            self.txsocket = socket.socket()
            # allow socket to reuse address
            self.txsocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            txport = 12345
            self.txsocket.bind(('', txport))
        
        # https://docs.python.org/3/library/socket.html#socket.socket.listen
        self.txsocket.listen(5) # number of unaccepted connections allow (backlog)
        
        while True:
            self.txconn, self.txaddr = self.txsocket.accept()
            print("accepted tx connection from:",str(self.txaddr[0]), ":", str(self.txaddr[1]))

    def setup_rxsocket(self):
        if self.rxsocket is None:
            self.rxsocket = socket.socket()
            # allow socket to reuse address
            self.rxsocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            rxport = 12346
            self.rxsocket.bind(('', rxport))
        
        # https://docs.python.org/3/library/socket.html#socket.socket.listen
        self.rxsocket.listen(5) # number of unaccepted connections allow (backlog)
        
        while True:
            self.rxconn, self.rxaddr = self.rxsocket.accept()
            print("accepted rx connection from:",str(self.rxaddr[0]), ":", str(self.rxaddr[1]))

            while True:
                data = self.rxconn.recv(1024)
                if data:
                    message = data.decode()
                    # carb.log_error("received:" + str(type(message)) + message)
                    print("received:", type(message), message)
                    x, y, z, dx, dy = ast.literal_eval(message)
                    print("received:", x, y, z, dx, dy)
                    weight = 0.1
                    self.dx = weight*dx
                    self.dy = weight*dy
                else:
                    self.dx = None
                    self.dy = None


    def shut_down_socket(self):
        if self.txconn:
            try:
                # self._conn.send("Done".encode())
                self.txsocket.shutdown(socket.SHUT_RDWR)
                self.txsocket.close()
                self.txsocket = None
            except socket.error as e:
                pass
        if self.rxconn:
            try:
                # self._conn.send("Done".encode())
                self.rxsocket.shutdown(socket.SHUT_RDWR)
                self.rxsocket.close()
                self.rxsocket = None
            except socket.error as e:
                pass