In case of importing this project as standalone app copy next files to appropriate locations (if already not there):
board\OpenIMU300ZA.json  ->  C:\Users\<username\.platformio\platforms\aceinna_imu\boards
ldscript\stm32f40x.ld    ->  C:\Users\<username>\.platformio\platforms\aceinna_imu\ldscripts

In the file openimu.json defined serial messages and configuration parameters supported by provided application example.
This file required for communication between IMU and Aceinna Navigatio Studio via serial port. 
Copy openimu.json file to python-openimu directory for working with Aceinna Navigation Studio developers.aceinna.com.
In case of creation additional custom messages in IMU application this file needs to be updated as well to include new 
parameters and messages definitions to be automatically supported by Aceinna Navigation Studio.
     