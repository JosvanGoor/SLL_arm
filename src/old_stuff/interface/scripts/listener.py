#!/usr/bin/env python
import os
import time
import cjson
import roslib
roslib.load_manifest('borg_pioneer')
import rospy
from std_msgs.msg import String, Int16
import pygame
from pygame.locals import *
from utility import *
from borg_pioneer.srv import MemoryReadSrv

service_read = rospy.ServiceProxy('/memory_read', MemoryReadSrv)
rospy.wait_for_service('/memory_read')    

# This should use write_on_screen with OUTPUT_COLOR.

def read_from_memory(mem_name):
    result = service_read('get_last_observation', rospy.Time.now(), mem_name, '')
    result = cjson.decode(result.json)
    return result

MAX_FPS = 60

# Scaling to screen
WIDTH_RATIO = 0.65
HEIGHT_RATIO = 0.8

# Text
FONT_SIZE = 96
OUTLINE_COLOR = (31, 31, 31)
INPUT_COLOR = (255, 127, 255)
OUTPUT_COLOR = (255, 255, 79)
DISPLAY_DURATION = 5
_text_last_recieved = 0
_last_text_time = 0

# Emotion
_current_emotion = 1

# Human detection
_human_detected = False

# Time


text = rospy.Publisher('text_input', String)


@callback('text_input', String, 'render_group')
def text_callback(data, args):
    global _text_last_recieved
    text = data.data
    screen = args[0]
    font = args[1]
    text_area = args[2]

    write_on_screen(
        text,
        font,
        screen,
        OUTLINE_COLOR,
        INPUT_COLOR,
        text_area,
        HEIGHT_RATIO
    )

    _text_last_recieved = pygame.time.get_ticks()


@callback('emotion', Int16)
def emotion_callback(data, args):
    global _current_emotion
    _current_emotion = data.data


def detect_humans():
    pass


def main():
    print "Starting interface"
    rospy.init_node('interface')

    running = True
    pygame.init()
    display_info = pygame.display.Info()
    screen_size = (display_info.current_w, display_info.current_h)

    screen = pygame.display.set_mode(screen_size)
    pygame.display.set_caption('Interface')
    clock = pygame.time.Clock()

    font = pygame.font.Font(None, FONT_SIZE)
    text_area = pygame.Rect((0, screen_size[1] * HEIGHT_RATIO), screen_size)

    borg_path = os.path.expandvars('$BORG/ros/catkin_ws/src/interface/')
    
    # Retrieve emotion from memory
    _current_emotion = read_from_memory('interface_emotion')
    
    # Retrieve input text from memory
    #_text_last_recieved = read_from_memory('interface_text')

    # Load all animations into the map
    emotion_map = load_map(
        os.path.join(borg_path, 'gfx/emotions/map')
    )

    # Static background of face
    base = emotion_map[0]

    # Human symbol for human detection
    human = load_image(os.path.join(borg_path, 'gfx/human.png'))
    human_scale = (
        int(screen_size[0] * 0.125),
        int(screen_size[1] * 0.3)
    )
    human = pygame.transform.scale(human, human_scale)
    human_position = (
        int(screen_size[0] * 0.025),
        int(screen_size[1] * 0.05)
    )
    


    
    # Subscribe with the given argument lists
    callback.subscribe_all(
        text_callback=(screen, font, text_area)
    )
    
    _last_text_time = time.time()

    while (running):
        _current_emotion = read_from_memory('interface_emotion')

        delta_time = time.time()

        for event in pygame.event.get():
            if event.type == QUIT:
                running = False
            elif event.type == KEYDOWN:
                if event.key == K_ESCAPE:
                    running = False
        
        if _current_emotion == None:
            _current_emotion = 1
        
        emotion_map[_current_emotion].update(delta_time)

        base_image = scale_to_screen(
            base.get_sprite(),
            screen_size,
            WIDTH_RATIO,
            HEIGHT_RATIO
        )

        emotion_image = scale_to_screen(
            emotion_map[_current_emotion].get_sprite(),
            screen_size,
            WIDTH_RATIO,
            HEIGHT_RATIO
        )

        x = int((screen_size[0] - base_image.get_width()) / 2.0)
        screen.blit(base_image, (x, 0))
        screen.blit(emotion_image, (x, 0))

        # Read text from memory somewhere here...
        _mem_text = read_from_memory('interface_text')
        print _mem_text
        if _mem_text != None:
            if float(_mem_text['time']) > float(_last_text_time):
                _last_text_time = _mem_text['time']
                text.publish(String(_mem_text['text']))

        #detect_humans()
        #if _human_detected:
        #    screen.blit(human, human_position)

        if delta_time - _last_text_time > 4:
            screen.fill((0, 0, 0), text_area)

        pygame.display.flip()
        clock.tick(MAX_FPS)

    pygame.quit()

if __name__ == '__main__':
    main()
