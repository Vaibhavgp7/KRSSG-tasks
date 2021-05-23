import socket
cards = list(range(1,53))
value = cards
for i in range(len(cards)):
    if (i+1)%13 != 0:
        value[i] = cards[i]%13
    else :
        value[i] = 13
   
ClientSocket = socket.socket()
host = '127.0.0.1'
port = 2004

print('Waiting for connection response')
try:
    ClientSocket.connect((host, port))
except socket.error as e:
    print(str(e))

    
while True:
    rece = ClientSocket.recv(1024)
    rece = rece.decode('utf-8')
    rece = eval(rece)
    print(rece)
    val = [value[rece[0]-1],value[rece[1]-1],value[rece[2]-1]]
    greatest = max(val)
    print(greatest)
    ClientSocket.send(str(greatest).encode('utf-8'))


ClientSocket.close()
