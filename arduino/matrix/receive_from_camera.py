# Définition d'un serveur réseau rudimentaire
# Ce serveur attend la connexion d'un client, pour entamer un dialogue avec lui

import socket, sys
import re
import serial
import random
import math
import sys, os, time

HOST = 'nb-arnault4'
PORT = 5000

# speed = 9600
speed = 115200

use_socket = True

if use_socket:
    # 1) création du socket :
    mySocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # 2) liaison du socket à une adresse précise :
    try:
        mySocket.bind((HOST, PORT))
    except socket.error:
        print("La liaison du socket à l'adresse choisie a échoué.")
        sys.exit()

previous = None

with serial.Serial('COM5', speed, timeout=.1) as arduino:

    while True:
        if use_socket:
            # 3) Attente de la requête de connexion d'un client :
            print("Serveur prêt, en attente de requêtes ...")
            mySocket.listen(5)

            # 4) Etablissement de la connexion :
            connexion, adresse = mySocket.accept()
            print("Client connecté, adresse IP %s, port %s" % (adresse[0], adresse[1]))

        messages = 1
        number = 0


        while True:
            try:
                if use_socket:
                    msgClient = connexion.recv(100).decode("utf-8")
                else:
                    msgClient = "123|456|789#"
            except socket.error:
                print("disconnexion from client")
                break

            if len(msgClient) > 0:
                m = re.match(r"([\d]+)[|]([\d]+)[|]([\d]+)[|]([\d]+)[#]", msgClient)
                if not m is None:
                    x1 = int(m[1])
                    y1 = int(m[2])
                    x2 = int(m[3])
                    y2 = int(m[4])
                    """
                    if not previous is None:
                        if number != (previous + 1):
                            print("Mismatch", number, previous, x, y)
    
                    previous = number
                    """

                if (number % 1) == 0:
                    # print("send >>> {}|{}|{}|{}|{}#".format(number, x1, y1, x2, y2))

                    arduino.write("{}|{}|{}|{}|{}#".format(number, x1, y1, x2, y2).encode("utf-8"))

                    data = arduino.readline()
                    if data:
                        print("received >>>", data.strip())

                number += 1
                """
                if messages >= 999:
                    messages = 0
                else:
                    messages += 1
                """
            else:
                break

            time.sleep(0.001)


# 6) Fermeture de la connexion :
connexion.send("Au revoir !")
print("Connexion interrompue.")
connexion.close()

