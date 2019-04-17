# Définition d'un client réseau rudimentaire
# Ce client dialogue avec un serveur ad hoc

import socket, sys
import random
import time

HOST = 'nb-arnault4'
PORT = 5000

message = 0

OK = 0
DISCONNEXION = 1
QUIT = 2

def set_connexion():

    while True:
        print("Waiting for connexion...")

        # 1) création du socket :
        try:
            mySocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        except socket.error:
            time.sleep(1)
            continue

        # 2) envoi d'une requête de connexion au serveur :
        try:
            mySocket.connect((HOST, PORT))
        except socket.error:
            time.sleep(1)
            continue

        print("Connexion établie avec le serveur.")
        break

    # 3) Dialogue avec le serveur :

    return mySocket


def send_data(to_socket, x, y):
    global message

    try:
        to_socket.send("({}){}|{}#".format(message, x, y).encode("utf-8"))
        message += 1
    except socket.error:
        print("Disconnexion from server")
        return DISCONNEXION

    try:
        # time.sleep(0.0001)
        for i in range(1000):
            pass
        pass
    except KeyboardInterrupt:
        return QUIT

    return OK


while True:

    mySocket = set_connexion()

    quit = False
    status = OK

    while True:
        x = int(random.random()*256)
        y = int(random.random()*256)

        status = send_data(mySocket, x, y)

        if status == QUIT:
            break
        if status == DISCONNEXION:
            break

    if status == QUIT:
        break

# 4) Fermeture de la connexion :
print("Connexion interrompue.")
mySocket.close()
