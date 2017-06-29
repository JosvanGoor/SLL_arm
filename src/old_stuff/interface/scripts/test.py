#!/usr/bin/env python
from os import system

import rospy
from std_msgs.msg import String, Int16

def main():
    running = True
    rospy.init_node('interface_test')
    text = rospy.Publisher('text_input', String)
    emotion = rospy.Publisher('emotion', Int16)
    try:
        while running:
            system('clear')
            print('1. Send text')
            print('2. Send emotion')
            choice = input('> ')
            system('clear')
            if choice == 1:
                data = raw_input('Enter text: ')
                text.publish(String(data))
            elif choice == 2:
                print('1. Neutral')
                print('2. Focused')
                print('3. Happy')
                print('4. Sad')
                print('5. Confused')
                print('6. Angry')
                data = input('Enter emotion: ')
                emotion.publish(Int16(data))
    except:
        pass

if __name__ == '__main__':
    main()
