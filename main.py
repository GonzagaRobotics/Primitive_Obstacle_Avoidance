import RPi.GPIO as GPIO
import serial
import time

triggerPin = 2
echoPin = 17

GPIO.setmode(GPIO.BCM)
GPIO.setup(triggerPin, GPIO.OUT)
GPIO.setup(echoPin, GPIO.IN)
ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
ser.flush()

def get_distance():
    GPIO.output(triggerPin, True)
    time.sleep(0.00001)  
    GPIO.output(triggerPin, False)
    start_time = time.time()
    stop_time = time.time()
    while GPIO.input(echoPin) == 0:
        start_time = time.time()  

    while GPIO.input(echoPin) == 1:
        stop_time = time.time()  
    elapsed_time = stop_time - start_time
    distance = (elapsed_time * .0343) / 2 
    return round(distance, 2)

def send_serial_stop( ):
    ser.write(f"FB:100, LR:100, EN:0".encode('utf-8'))
    #print(f"FB:100, LR:100, EN:0")

    return None

def main():
    print("Here")
    try:
        while True:
            distance = get_distance()
            if(distance<100):
                send_serial_stop()
            time.sleep(1) 
    except KeyboardInterrupt:
        print("\nExiting...")
        GPIO.cleanup()
        ser.close()

    return None

if __name__=="__main__":
    main()