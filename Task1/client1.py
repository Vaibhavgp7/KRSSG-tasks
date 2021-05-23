import socket

c = socket.socket()
hst = '127.0.0.1'
prt = 2100

try:
    c.connect((hst, prt))
except socket.error as e:
    print(str(e))

print("Connected")

input_p = input("Enter the number of players : ")
c.send(str.encode(input_p))
input_r = input("Enter the number of rounds to be played : ")
while True:
    if (int(input_r))%(int(input_p)) == 0:
        print("Number of rounds should not be divisible by number of players")
        print("Enter new input")
        input_r = input("Enter the number of rounds to be played : ")
    else:
        c.send(str.encode(input_r))
        break

c.close()
        



