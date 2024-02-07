# HARDWARE INTERFACING + JOINT STATE PUBLISHER
# Author: PRATHAM T
# GitHub: github.com/AiryAir

# Made by air!

import Jetson.GPIO as GPIO
import time
import threading
import rclpy
from std_msgs.msg import Int32
import signal
import sys

ENC1_CHA = 36
ENC1_CHB = 35
ENC2_CHA = 31
ENC2_CHB = 40
LEFT_PWM = 33
LEFT_IN1 = 12
LEFT_IN2 = 13
RIGHT_PWM = 32
RIGHT_IN3 = 15
RIGHT_IN4 = 16

GPIO.setmode(GPIO.BOARD)
GPIO.setup(ENC1_CHA, GPIO.IN)
GPIO.setup(ENC1_CHB, GPIO.IN)
GPIO.setup(ENC2_CHA, GPIO.IN)
GPIO.setup(ENC2_CHB, GPIO.IN)
GPIO.setup(LEFT_PWM, GPIO.OUT)
GPIO.setup(LEFT_IN1, GPIO.OUT)
GPIO.setup(LEFT_IN2, GPIO.OUT)
GPIO.setup(RIGHT_PWM, GPIO.OUT)
GPIO.setup(RIGHT_IN3, GPIO.OUT)
GPIO.setup(RIGHT_IN4, GPIO.OUT)

left_pwm = GPIO.PWM(LEFT_PWM, 100)
right_pwm = GPIO.PWM(RIGHT_PWM, 100)

left_pwm.start(0)
right_pwm.start(0)

exit_requested = False

# not working
def signal_handler(sig, frame):
    global exit_requested
    print("Ctrl+C pressed. Cleaning up GPIO and exiting.")
    exit_requested = True

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('encoder_and_pwm_control')

    left_ticks_pub = node.create_publisher(Int32, '/left_ticks', 10)
    right_ticks_pub = node.create_publisher(Int32, '/right_ticks', 10)

    def left_pwm_callback(msg):
        left_pwm.ChangeDutyCycle(msg.data * 100 / 255)
        if msg.data > 0:
            GPIO.output(LEFT_IN1, GPIO.HIGH)
            GPIO.output(LEFT_IN2, GPIO.LOW)
        else:
            GPIO.output(LEFT_IN1, GPIO.LOW)
            GPIO.output(LEFT_IN2, GPIO.HIGH)

    def right_pwm_callback(msg):
        right_pwm.ChangeDutyCycle(msg.data * 100 / 255)
        if msg.data > 0:
            GPIO.output(RIGHT_IN3, GPIO.HIGH)
            GPIO.output(RIGHT_IN4, GPIO.LOW)
        else:
            GPIO.output(RIGHT_IN3, GPIO.LOW)
            GPIO.output(RIGHT_IN4, GPIO.HIGH)

    left_pwm_sub = node.create_subscription(Int32, '/left_pwm', left_pwm_callback, 10)
    right_pwm_sub = node.create_subscription(Int32, '/right_pwm', right_pwm_callback, 10)

    def read_encoder(CHA, CHB):
        encoder_val = Int32()
        encoder_val.data = 0
        prev_CHA_state = GPIO.input(CHA)
        while rclpy.ok() and not exit_requested:
            CHA_state = GPIO.input(CHA)
            CHB_state = GPIO.input(CHB)
            if prev_CHA_state == GPIO.LOW and CHA_state == GPIO.HIGH:
                if CHB_state == GPIO.LOW:
                    encoder_val.data += 1
                else:
                    encoder_val.data -= 1
                left_ticks_pub.publish(encoder_val)
                right_ticks_pub.publish(encoder_val)
            prev_CHA_state = CHA_state
            time.sleep(0.001)
    
    thread1 = threading.Thread(target=read_encoder, args=(ENC1_CHA, ENC1_CHB))
    thread2 = threading.Thread(target=read_encoder, args=(ENC2_CHA, ENC2_CHB))
    thread1.start()
    thread2.start()

    signal.signal(signal.SIGINT, signal_handler)

    while rclpy.ok() and not exit_requested:
        rclpy.spin_once(node)

    left_pwm.stop()
    right_pwm.stop()
    GPIO.cleanup()

    thread1.join()
    thread2.join()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
