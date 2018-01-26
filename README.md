Abstract:
This repo involves codes on application of Arduino UNO to GPS guided unmanned vehicle. The four-wheeled small scale (55 cm χ 25 cm) vehicle is capable of moving forward, rotating and consists of a GPS sensor, a magnetometer, a motor driver and four DC motors. The target coordinates (key points) are manually entered as a part of the control algorithm script (.ino file). The developed control algorithm makes a blind calculation of the distance and required rotation between consecutive key points. The initial coordinates of the vehicle, however, requires the only field-dependent calculation of the distance and target heading between the initial and the first key point. The whole target route is achieved by consecutive “rotation” and “forward movements (straight path)” between key points. The GPS sensor is used to determine the initial coordinates whereas magnetometer is used to determine the heading (and rotation) before (and during) each key point movement. When taxiing through blind-calculated route, the acquired GPS data is transmitted via Bluetooth for offline monitoring.

Published in: 2017 10th International Conference on Electrical and Electronics Engineering (ELECO)

If you use these codes please cite: 

T. Bozik and G. Gurkan, "An Arduino UNO application: GPS guided unmanned ground vehicle," 2017 10th International Conference on Electrical and Electronics Engineering (ELECO), Bursa, Turkey, 2017, pp. 852-855.

URL: http://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=8266283&isnumber=8266128
