import os
import random
import pygame

__all__ = [
    'Animation', 'Emotion'
]

# Blink interval - shouldn't be here
_blink_low = 2
_blink_high = 12


class Animation(object):

    def __init__(self, sprite_sheet, sprite_count, interval):
        self._sprite_sheet = sprite_sheet
        self._sprite_count = sprite_count
        self._interval = interval
        self._sprite_index = 0
        self._last_update = 0

    def update(self, delta_time):
        if delta_time - self._last_update > self._interval:
            self._sprite_index = (self._sprite_index + 1) % self._sprite_count
            self._last_update = delta_time
        # Return True if animation ended
        return not self._sprite_index

    def get_sprite(self):
        if self._sprite_count < 2:
            return self._sprite_sheet
        sprite_width = self._sprite_sheet.get_width() / self._sprite_count
        x = self._sprite_index * sprite_width
        y = 0
        w = sprite_width
        h = self._sprite_sheet.get_height()
        rect = pygame.Rect(x, y, w, h)
        return self._sprite_sheet.subsurface(rect)

    def reset(self, delta_time):
        self._sprite_index = 0
        self._last_update = delta_time


class Emotion(object):

    def __init__(self, base, blink):
        self._base = base
        self._blink = blink
        self._current_animation = self._base
        self._next_blink = random.randrange(_blink_low, _blink_high)

    def update(self, delta_time):
        if self._current_animation == self._base:
            self._base.update(delta_time)
            if not self._blink:
                return
        elif self._current_animation == self._blink:
            stopped = self._blink.update(delta_time)
            if stopped:
                self._next_blink = random.randrange(_blink_low, _blink_high)
                self._base.reset(delta_time)
                self._current_animation = self._base
        if delta_time - self._blink._last_update >= self._next_blink * 1000:
            # No need to reset blink animaiton, since it goes from start to end
            # every time
            self._current_animation = self._blink

    def get_sprite(self):
        return self._current_animation.get_sprite()

