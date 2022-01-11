import socket
import threading
import datetime
 

def handle_client(client_soc, address):
    print(f"Connection from {address} has been established!")
    file=open('file.txt', 'w')
    print('New file Created at: '+str(datetime.datetime.now()))

    while True:      
           
        message = client_soc.recv(1024)
        file.write('['+str(datetime.datetime.now())+']')
        file.write(message.decode('utf-8'))

        file.flush()
        print('File Saved at: '+str(datetime.datetime.now()))
#         print(message)  #bytes to string message using utf-8
#     file.close()
#     print('File closed at: '+str(datetime.datetime.now()))
     
 #         if(message=='end\r'):
#             print("System Message: closing connection with ", str(address))
#             client_soc.close()
        
def main():
    #define socket object

    s = socket.socket()#socket.AF_INET, socket.SOCK_STREAM) # AF_INET---->IPV4, STREAM---->TCP

    #now bind the socket object

    s.bind(('192.168.137.1',8002))  #bind to a tuple, it has IP and a port number

    s.listen(5) # it will have a queue of 5 if overloaded

    while True:
        clientSocket, address = s.accept()  #Address is tuple of local host and clinet, this is also blocking so that reduandant threads arent made
        #print(f"Connection from {address} has been established!")
        #clientsocket.send(bytes("Welcome to the server!", "utf-8"))  #send info to client socket
 
        
        threading.Thread(target=handle_client, args=(clientSocket, address, )).start()
        
 
     
print('server is listening')
main()