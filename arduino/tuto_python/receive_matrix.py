
import serial
import re

old = ""
number = 0

first = True

speed = 9600
# speed = 115200

with serial.Serial('COM5', speed, timeout=.1) as ser:

    """
    ser.write("start#".encode("utf-8"))

    while(True):
        data = ser.read(100).decode("utf-8")
        print(data)
    """

    while(True):
        data = ser.read(100).decode("utf-8")
        print(data)

        if first:
            first = False

            close = data.find(">")
            open = data.find("<")
            if close > 0 and open > 0 and close < open:
                data = data[open:]

        buffer = old + data

        print("\ndata[\n", buffer, "]\n")
        line = buffer.split("\r\n")
        size = 0
        for w in line:
            s = re.match(r"[\s]*[<].*", w)
            if s is None:
                # print("### no < ({})".format(w))
                continue
            elif w.endswith(">"):
                number += 1
                size += len(w) + 2
                print("{}>>>#{}# [{}]".format(number, buffer[size:size+10], w.strip()))
                print("{}>>> [{}]".format(number, w.strip()))
            else:
                # print("### no > [{}]".format(w))
                break;
        old = buffer[size:]

        # print("old=[{}]".format(old))

