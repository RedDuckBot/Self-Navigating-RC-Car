import cv2, socket, pickle
import numpy as np 

s = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
host_ip = "10.60.116.176"
port = 6669
s.bind((host_ip,port))

while True:
    payload = s.recvfrom(1000000)
    client_ip = payload[1][0]
    data = payload[0]

    data = pickle.loads(data)

    img = cv2.imdecode(data, cv2.IMREAD_COLOR)
    cv2.imshow("Im Server", img)

    if cv2.waitKey(5) & 0xFF == 113:
        break

cv2.destroyAllWindows()