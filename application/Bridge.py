
import socket

"""
Socket for communication with bridge to Arduino
"""

class Bridge(object):
    OK = 0
    DISCONNEXION = 1
    QUIT = 2
    ERROR = 3

    def __init__(self):
        self.my_socket = None
        self.host = 'nb-arnault4'
        self.port = 5000
        self.message = 0

    def connect(self):
        print("testing for connexion...")

        # 1) création du socket :
        try:
            self.my_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        except socket.error:
            return self.ERROR

        # 2) envoi d'une requête de connexion au serveur :
        try:
            self.my_socket.connect((self.host, self.port))
        except socket.error:
            return self.ERROR

        print("Connexion établie avec le serveur.")

        # 3) Dialogue avec le serveur :

        return self.OK

    """
    Sending data to the Bridge to Arduino
    """

    def send_data(self, x1, y1, x2, y2):
        if self.my_socket is None:
            status = self.connect()
            if status != self.OK:
                return status

        try:
            self.my_socket.send("{}|{}|{}|{}#".format(x1, y1, x2, y2).encode("utf-8"))
            self.message += 1
        except socket.error:
            print("Disconnexion from server")
            self.my_socket = None
            return self.DISCONNEXION

        try:
            # time.sleep(0.0001)
            for i in range(100):
                pass
            pass
        except KeyboardInterrupt:
            self.my_socket = None
            return self.QUIT

        return self.OK


