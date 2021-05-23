import socket

c = socket.socket()
host = '127.0.0.1'
port = 2024
i=1
try:
    c.connect((host, port))
except socket.error as e:
    print(str(e))
print("Connection established")
quer = input("Enter the number of queries:")
c.send(str.encode(quer))

while i<=4:
    Inp = input("Enter a list of 8 space separated integers consisting of 0 and 1:")
    c.send(str.encode(Inp))
    i += 1

c.close()
