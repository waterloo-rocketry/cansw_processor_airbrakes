s = "FA 05 01 00 0E 00 78 D8 F1 E0 2C 00 00 00 F5 01 01 BE BB A0 B7 3E DB 27 36 3F 70 8A 71 3A D6 A8 04 3D A7 F6 1C C1 B7 CB C5 39 C3 C8 8A 3A 6E 23 9B 3B E8 1E"

#print(s.replace("[RX] - ", " ").replace(" ", " 0x").replace("\n ", "\n")[1:]) #if you are copying multiple liness

#single line
s = " " + s
print(s.replace(" ", " 0x").replace("\n ", "\n")[1:])