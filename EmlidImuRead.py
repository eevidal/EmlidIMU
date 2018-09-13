import sys

sys.path.append('.')
import RTIMU
import os.path
import time
import math
import socket
import select

#import utm
#import pynmea2
from datetime import datetime


update_rate = 100.0 # Hz
last_update = 0.0
last_print = 0.0

#IP and Port for the computer ros
UDP_IP = '192.168.88.10'
UDP_PORT = 9999


BUFFER_SIZE = 1024

#create sockets
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


#Initial configuration of messages and debugging
print_messages = True
running = True
gga_msg_on = False
rmc_msg_on = False
aog_msg_on = True
save_to_file = False
started_file = False
debugging = False

antennaHeight = 1.5 #Height from ground to antenna in METERS

#Allow user to enter an IP for the AgOpen Program as part of the start 
print "Default IP: ", UDP_IP 
new_ip = raw_input("Enter Receiving computer IP (blank for Default IP):")
if len(new_ip)>5:
    print new_ip
    UDP_IP=new_ip
print "UDP_IP is :", UDP_IP, "   Port: ", UDP_PORT

#Kalman  Variables
Pc = 0.0
G = 0.0
P = 1.0
Xp = 0.0
Zp = 0.0
KalRoll = 0.0
varRoll = 0.1
varProcess = .00001  ##smaller is more filtering
Kalman = True



#Read in the RTIMULib.ini file
SETTINGS_FILE = "RTIMULib"

print("Using settings file " + SETTINGS_FILE + ".ini")
if not os.path.exists(SETTINGS_FILE + ".ini"):
  print("Settings file does not exist, will be created")

s = RTIMU.Settings(SETTINGS_FILE)
imu = RTIMU.RTIMU(s)

print("IMU Name: " + imu.IMUName())

if (not imu.IMUInit()):
    print("IMU Init Failed")
    sys.exit(1)
else:
    print("IMU Init Succeeded")

# this is a good time to set any fusion parameters
# Slerp power controls the fusion and can be between 0 and 1
# 0 means that only gyros are used, 1 means that only accels/compass are used
# In-between gives the fusion mix.
imu.setSlerpPower(0.02)
imu.setGyroEnable(True)
imu.setAccelEnable(True)
imu.setCompassEnable(True)


poll_interval = imu.IMUGetPollInterval()
print("Recommended Poll Interval: %dmS\n" % poll_interval)
#print("Update Rate:" ,update_rate, " %6.2f" %(1/update_rate))

print ("Configuring....")
time.sleep(1)
now = time.time()


#Print out initial IMU reads before starting the looping.  confirms IMU is working
i=0
while i<100:
    if imu.IMURead():
        print("i", i, "data", imu.getIMUData())
    i+=1


    
#Function to split comma separated NMEA sentence into a list of the individual data fields
def get_csv_chunks(CSV, chunks):
    split_csv = CSV.split(',')
    return_list=[]
    for fields in chunks:
        return_list.append(split_csv[fields])
    return (return_list)



def buildImuMesagge( roll, pitch, yaw, gyro, IMUStatus):
    csum = 0
    if(yaw < 0):
       heading = yaw+360
    if (yaw >= 0):
        heading = yaw
    yawrate = round(math.degrees(gyro[2]), 2)
    IMUStatus = str(IMUStatus)[:1]

    IMUMesagge = "IMU,"

    IMUMesagge = IMUMesagge+str(-1*roll)+","+str(pitch)+","+str(heading)+","+str(yawrate)+","+IMUStatus
    #PAOGI = PAOGI+str(round(math.degrees(fusionPose[0]),2))+","+str(round(math.degrees(fusionPose[1]),2))+","+str(heading)+","+str(yawrate)+","+IMUStatus+","
    
#compute and append checksum
    for c in IMUMesagge:
        csum ^= ord(c)
    csum=str(hex(csum))[-2:]
    csum= csum.upper()
    IMUMesagge = "$"+IMUMesagge+"*"+csum
    
    return(IMUMesagge)
    

if __name__ == "__main__":
    try:
        
        while running:
            
            if imu.IMURead():
                #print("In IMU read")
                now=time.time()
                data = imu.getIMUData()
                #print(data)
                fusionPose = data["fusionPose"]
                IMUStatus = data["fusionPoseValid"]
                gyro = data["gyro"]
                #yaw = math.degrees(fusionPose[2])-90  ## this is in to match BNO on my mounting box.
                yaw = round(math.degrees(fusionPose[2]),2)
                if(yaw < 0):
                   heading = yaw+360
                if (yaw >= 0):
                    heading = yaw
                roll = math.degrees(fusionPose[0])
                pitch = math.degrees(fusionPose[1])
                if(Kalman == True):
                    Pc = P + varProcess
                    G = Pc / (Pc+varRoll)
                    P = (1-G)*Pc
                    Xp = KalRoll
                    Zp = Xp
                    KalRoll = (G*(roll-Zp))+Xp
                    IMUMesagge = buildImuMesagge(KalRoll, pitch, yaw, gyro, IMUStatus) 
                    
                    message = IMUMesagge+'\r\n'
                    
                    sock.sendto(message, (UDP_IP, UDP_PORT))
                    if print_messages:
                        print "Sent Message:"
                        print message
                    output_string = message
                    if debugging:
                        print "Roll:", roll, " KalRoll:", KalRoll
                   
                    last_print = now
                    
#                    if (save_to_file == True and started_file == True):
#                        with open(file_name, 'a') as text_file:
#                            text_file.write (output_string[:-4]+ str(roll)+"\r\n")

#Bit of code to monitor keyboard input (works over SSH) and set variable based on what has been typed in
                            #Just type in teh codes and press enter while the system running.  You want' be able to easily see
                            #what is types in as teh messages will keep running over teh keyboard input.
                            
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                keystroke = sys.stdin.readline().rstrip()
                print "You entered", keystroke
                if keystroke == "m":
                    print_messages = not print_messages
                    print "Display messages on screen set to:", print_messages
                elif keystroke == "X":
                    print "Exiting Program"

                    running = False
                    break
                elif (keystroke == 's'):
                    if started_file == False:
                        now=datetime.now()
                        started_file = True
                        file_name = ("./datafiles/data_"+str(now.month)+ "_"+str(now.day)+"_"+str(now.hour)+"_"+str(now.minute)+".csv")
                        with open(file_name, 'w') as text_file:
                            text_file.write("PAOGI, UTCseconds, Lat, LatDir, Lon, LonDir, FixQual, Sats, HDOP, Alt, AltUnits, DGPS Age, Spd_KPH, Hdg_True, IMU_HDG_True, Roll_Kal, Pitch, Yaw_rate, IMU_Status, Roll"+ '\n')
                    save_to_file = not save_to_file
                    print "Save to file is set to: ", save_to_file
                elif (keystroke == 'debug'):
                    debugging = not debugging
                elif (keystroke.split("=")[0] == 'varp'):
                    varProcess = float(keystroke.split("=")[1])

                else:
                    print "you entered: ", keystroke, "  No action being taken"
#Brief sleep timer to reduce CPU load.  Runs at approximately 100 Hz
            time.sleep(0.009)
            
    except KeyboardInterrupt:
        print("interrupted!")

        sock.close()
    
print (data)
      
