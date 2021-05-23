import socket
import os
import time
from _thread import *
s = socket.socket()
host = '127.0.0.1'
port = 2024

try:
    s.bind((host, port))
except socket.error as e:
    print(str(e))

print('Socket is listening..')
s.listen()

currstate = temp = [0,0,0,0,0,0,0,0]
states = ["A1B1","B1B2","A2B2","A1A2","C1D1","D1D2","C2D2","C1C2","A1D2","B1C2","C1A2","D1B2"]
steps = 1


conn, addr = s.accept()
print('Connected to: ' + addr[0] + ':' + str(addr[1]))
rece = conn.recv(2048)
t = rece.decode('utf-8')
t = int(t)
    
def stateofA(sub):
    red = not (sub[0] | sub[1])
    if int(red) == 1:
        print("    A : off")
    else:
        if sub[0] == 1 and sub[1] == 1 :
            print("    A : go straight,go right")
        elif sub[0] == 1 and sub[1] == 0:
            print("    A : go straight")
        else:
            print("    A : go right")
def stateofB(sub):
    red = not (sub[2] | sub[3])
    if int(red) == 1:
        print("    B : off")
    else:
        if sub[2] == 1 and sub[3] == 1 :
            print("    B : go straight,go right")
        elif sub[2] == 1 and sub[3] == 0:
            print("    B : go straight")
        else:
            print("    B : go right")
def stateofC(sub):
    red = not (sub[4] | sub[5])
    if int(red) == 1:
        print("    C : off")
    else:
        if sub[4] == 1 and sub[5] == 1 :
            print("    C : go straight,go right")
        elif sub[4] == 1 and sub[5] == 0:
            print("    C : go straight")
        else:
            print("    C : go right")
def stateofD(sub):
    red = not (sub[6] | sub[7])
    if int(red) == 1:
        print("    D : off")
    else:
        if sub[6] == 1 and sub[7] == 1 :
            print("    D : go straight,go right")
        elif sub[6] == 1 and sub[7] == 0:
            print("    D : go straight")
        else:
            print("    D : go right")
    
    
def currentstate1(j):
    a1 = [0,2,1,0,4,6,5,4,0,2,4,6]
    a2 = [2,3,3,1,6,7,7,5,7,5,1,3]
    currstate = [0,0,0,0,0,0,0,0]
    currstate[a1[j]] = 1
    currstate[a2[j]] = 1
    return currstate
    
def currentstate2(j,lst1,lst2):
    a1 = [0,2,1,0,4,6,5,4,0,2,4,6]
    a2 = [2,3,3,1,6,7,7,5,7,5,1,3]
    currstate = [0,0,0,0,0,0,0,0]
    if lst1[j] == 0:
        currstate[a2[j]] = 1
    else:
        currstate[a1[j]] = 1
    return currstate
            
for i in range(t):
    rec = conn.recv(2048)
    q = rec.decode('utf-8')
    if len(q) != 15 :
        print("Invalid input")
        i -= 1
        continue
    input_string = list(map(int,q.split()))
    if (input_string.count(1) + input_string.count(0))!= 8:
        print("Invalid input")
        i -= 1
        continue
    queue = []
    for k in range(0,8):
        queue.append(temp[k] + input_string[k])
    
    lst1 = [queue[0],queue[2],queue[1],queue[0],queue[4],queue[6],queue[5],queue[4],queue[0],queue[2],queue[4],queue[6]]
    lst2 = [queue[2],queue[3],queue[3],queue[1],queue[6],queue[7],queue[7],queue[5],queue[7],queue[5],queue[1],queue[3]]
    maxlist = []
    for k in range(0,12):
        maxlist.append(lst1[k] + lst2[k])
    if max(maxlist)>=2:
        for j in range(0,12):
            if max(maxlist) == maxlist[j] :
                if lst1[j]>0 and lst2[j]>0:
                    index = j
                    sub = currentstate1(index)
                    break
                else:
                    index = j
                    sub = currentstate2(j,lst1,lst2)
                    break
    else:
        for j in range(0,12):
            if max(maxlist) == maxlist[j]:
                index = j
                sub = currentstate2(j,lst1,lst2)
                break
    print("Time step : {}".format(steps))
    print("    Current State : "+str(sub))
    stateofA(sub)
    stateofB(sub)
    stateofC(sub)
    stateofD(sub)
    print("    Initial queue : "+str(queue))
    tempo = []
    for k in range(0,8):
        tempo.append(queue[k] - sub[k])
    queue = tempo
    print("    Final queue   : "+str(queue))
    steps += 1
    temp = queue
s.close()

while queue != [0,0,0,0,0,0,0,0]:
    lst1 = [queue[0],queue[2],queue[1],queue[0],queue[4],queue[6],queue[5],queue[4],queue[0],queue[2],queue[4],queue[6]]
    lst2 = [queue[2],queue[3],queue[3],queue[1],queue[6],queue[7],queue[7],queue[5],queue[7],queue[5],queue[1],queue[3]]
    maxlist = []
    for k in range(0,12):
        maxlist.append(lst1[k] + lst2[k])
    if max(maxlist)>=2:
        for j in range(0,12):
            if max(maxlist) == maxlist[j] :
                if lst1[j]>0 and lst2[j]>0:
                    index = j
                    sub = currentstate1(index)
                    break
                else:
                    index = j
                    sub = currentstate2(j,lst1,lst2)
                    break
    else:
        for j in range(0,12):
            if max(maxlist) == maxlist[j]:
                index = j
                sub = currentstate2(j,lst1,lst2)
                break
    print("Time step : {}".format(steps))
    print("    Current State : "+str(sub))
    stateofA(sub)
    stateofB(sub)
    stateofC(sub)
    stateofD(sub)
    print("    Initial queue : "+str(queue))
    tempo = []
    for k in range(0,8):
        tempo.append(queue[k] - sub[k])
    queue = tempo
    print("    Final queue   : "+str(queue))
    steps += 1
    
    
    
                
            
            
            
               
               
               


    
               
               
