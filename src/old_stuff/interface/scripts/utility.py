import pygame
import rospy

from animation import *

__all__ = [
    'load_image', 'load_map',
    'scale_to_screen', 'wrap_text',
    'render_outlined', 'write_on_screen',
    'callback'
]

# These shouldn't be in here (probably read from the map)
_base_frames = 1
_base_interval = 0

_blink_frames = 5
_blink_interval = 150
    

def load_image(filename, alpha=True):
    image = pygame.image.load(filename)
    if not alpha:
        return image.convert()
    return image.convert_alpha()


def load_map(filename):
    import os
    map_ = {}
    directory = os.path.dirname(filename)
    f = open(filename, 'r')
    lines = [line.strip() for line in f.readlines() if \
                (not line.startswith('#') and line.strip())]
    f.close()
    for line in lines:
        line = line.split('\t')
        index = int(line[0])
        emotion = None
        image_files = \
            [os.path.join(directory, path) for path in line[1].split(',')]
        # Not every emotion needs a blink animation
        if len(image_files) == 1:
            base_image = load_image(image_files[0])
            base_anim = Animation(
                base_image,
                _base_frames,
                _base_interval
            )
            emotion = Emotion(base_anim, None)
        else:
            base_image = load_image(image_files[0])
            blink_image = load_image(image_files[1])
            base_anim = Animation(
                base_image,
                _base_frames,
                _base_interval
            )
            blink_anim = Animation(
                blink_image,
                _blink_frames,
                _blink_interval
            )
            emotion = Emotion(base_anim, blink_anim)
        map_[index] = emotion
    return map_


def scale_to_screen(image, screen_size, width_ratio, height_ratio):
    width = int(screen_size[0] * width_ratio)
    height = int(screen_size[1] * height_ratio)
    return pygame.transform.scale(image, (width, height))


def wrap_text(text, font, screen_width):
    segment_length = 1
    while font.size(text[:segment_length])[0] < screen_width:
        segment_length += 1
        if segment_length >= len(text):
            return text
    # Try to split at a space - if none are found, just split the line anyway
    wrap_index = text.rfind(' ', 0, segment_length)
    if wrap_index == -1:
        wrap_index = segment_length
    final_text = text[:wrap_index] + '\n' + \
        wrap_text(text[wrap_index + 1:], font, screen_width)
    return final_text


def render_outlined(text, font, position, screen, outline_color, text_color):
    outline = font.render(text, 1, outline_color)
    # Move it around a little, creating the outline
    for i in range(-2, 3):
        for j in range(-2, 3):
            width = position[0] + i
            height = position[1] + j
            screen.blit(outline, (width, height))
    text_image = font.render(text, 1, text_color)
    screen.blit(text_image, position)


def write_on_screen(text, font, screen, outline_color, text_color, text_area,
                    height_ratio):
    lines = text.split('\n')
    line_count = 0
    line_height = font.get_linesize()
    screen_size = screen.get_size()

    y = text_area.y

    screen.fill((0, 0, 0), text_area)

    for line in lines:
        wrapped_lines = wrap_text(text, font, screen_size[0]).split('\n')
        for wrapped_line in wrapped_lines:
            text_size = font.size(wrapped_line)
            x = int((screen_size[0] - text_size[0]) / 2.0)
            y = int(text_area.y + line_count * line_height)
            render_outlined(
                wrapped_line,
                font,
                (x, y),
                screen,
                outline_color,
                text_color
            )
            line_count += 1


class callback(object):

    _callback_functions = {}

    def __init__(self, topic_name, data_class, *argument_groups):
        self._topic_name = topic_name
        self._data_class = data_class
        self._argument_groups = list(argument_groups)

    def __call__(self, function):
        if function not in self._callback_functions:
            self._argument_groups.insert(1, function.func_name)
            # Converting to set to remove possible duplicates
            self._argument_groups = list(set(self._argument_groups))
            self._callback_functions[self] = function

    @classmethod
    def subscribe_all(cls, **kwargs):
        # Construct the argument list for the callback functions
        for function in cls._callback_functions:
            callback_function = cls._callback_functions[function]
            callback_args = []
            for kwarg in kwargs:
                if kwarg in function._argument_groups:
                    argument = kwargs[kwarg]
                    if type(argument) in (list, tuple):
                        callback_args.extend(argument)
                    else:
                        callback_args.append(argument)
            rospy.Subscriber(
                function._topic_name,
                function._data_class,
                callback_function,
                callback_args
            )

