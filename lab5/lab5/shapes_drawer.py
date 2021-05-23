import math


class RectangleDrawer:
    def __init__(self, a, b):
        self.t = 1
        self.a = float(a)
        self.b = float(b)

    def get_next_point(self):
        if self.t == 1:
            self.t += 1
            return [self.a / 2, self.b / 2, 0.8]
        if self.t == 2:
            self.t += 1
            return [-1 * self.a / 2, self.b / 2, 0.8]
        if self.t == 3:
            self.t += 1
            return [-1 * self.a / 2, -1 * self.b / 2, 0.8]
        if self.t == 4:
            self.t = 1
            return [self.a / 2, -1 * self.b / 2, 0.8]


class ElipseDrawer:
    def __init__(self, a, b):
        self.t = 0
        self.a = float(a)
        self.b = float(b)

    def get_next_point(self):
        point = [self.a * math.cos(self.t), self.b * math.sin(self.t), 0.8]
        self.t += math.radians(0.5)
        self.t = self.normalize_angle(self.t)
        return point

    def normalize_angle(self, angle):
        if angle >= math.pi * 2:
            return angle - math.pi * 2
        else:
            return angle
