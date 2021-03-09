from tkinter import Tk


class KeyCapturer(object):
    LINEAR_VEL = float(5.0)
    ANGULAR_VEL = float(0.5)

    def __init__(self, pub):
        self.publisher = pub
        self.root = Tk()
        self.root.bind(self.publisher.get_parameter('forward').get_parameter_value().string_value,
                       self.publisher.go_forward)
        self.root.bind(self.publisher.get_parameter('backwards').get_parameter_value().string_value,
                       self.publisher.go_backwards)
        self.root.bind(self.publisher.get_parameter('right').get_parameter_value().string_value,
                       self.publisher.rotate_right)
        self.root.bind(self.publisher.get_parameter('left').get_parameter_value().string_value,
                       self.publisher.rotate_left)

    def run(self):
        self.root.mainloop()
