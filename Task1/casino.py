import socket
import os
import random
import time
from _thread import *
import threading


cards = list(range(1,53))
value = cards
for i in range(len(cards)):
    if (i+1)%13 != 0:
        value[i] = cards[i]%13
    else :
        value[i] = 13
        
s = socket.socket()
hst = '127.0.0.1'
prt = 2100
try:
    s.bind((hst, prt))
except socket.error as e:
    print(str(e))

print('Socket 1 is listening..')
s.listen()

conn, addr = s.accept()
print('Connected to: ' + addr[0] + ':' + str(addr[1]))
re = conn.recv(2048)
player = int(re.decode('utf-8'))
print("Number of players = {}".format(player))
re = conn.recv(2048)
rounds = int(re.decode('utf-8'))
print("Number of rounds to be played = {}".format(rounds))

s.close()

ServerSocket = socket.socket()
host = '127.0.0.1'
port = 2004
total = []
rec = []
response  = threadname = []
i=1
total = [0]*player
try:
    ServerSocket.bind((host, port))
except socket.error as e:
    print(str(e))
print('Socket 2 is listening..')
ServerSocket.listen()


def client(connection):
    global response
    response=[]
    global a
    global randomlist
    response.insert(0,randomlist[a])
    response.insert(1,randomlist[a+1])
    response.insert(2,randomlist[a+2])
    a +=3
    response = str(response)
    connection.send(str.encode(response))
    data = connection.recv(2048)
    received= data.decode('utf-8')
    num = int(received)
    rec.append(num)
for m in range(player):
    connection,addr = ServerSocket.accept()
    print('Connected to: ' + addr[0] + ':' + str(addr[1]))
    print('Player Number: {}'.format(m+1))
    threadname.insert(m,connection)

while i<=rounds:
    randomlist = random.sample(range(1,53),(3*player))
    print(randomlist)
    a = 0  
    for j in range(player):
        start_new_thread( client,(threadname[j], ))
        time.sleep(0.5)
    
    print(rec)
    maxim = max(rec)
    if rec.count(maxim)>1:
        for d in range(player):
            if rec[d]==maxim:
                print("The winner of round {} is Player number {}".format(i,d+1))
                total[d] +=1
    else:
        index = rec.index(maxim) + 1
        print("The winner of round {} is Player number {}".format(i,index))
        total[index-1] += 1
    i += 1
    rec.clear()
else:
    maximum = max(total)
    if total.count(maximum)>1:
        for a in range(player):
            if total[a]==maximum:
                print("The winner of game of cards is Player number {}".format(a+1))
    else:
        index = total.index(maximum) + 1
        print("The winner of game of cards is Player number {}".format(index))

ServerSocket.close()
