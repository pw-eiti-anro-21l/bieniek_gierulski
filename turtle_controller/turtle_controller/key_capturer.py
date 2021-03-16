from tkinter import Tk


class KeyCapturer(object):

    def __init__(self, pub):
        self.publisher = pub
        self.root = Tk()
        
        self.root.bind("<Key>", self.key_pressed)   # function will be called upon every key press

    def key_pressed(self, event):
        # get actual parameters values
        str_val1 = self.publisher.get_parameter('forward').get_parameter_value().string_value

        str_val2 = self.publisher.get_parameter('backwards').get_parameter_value().string_value

        str_val3 = self.publisher.get_parameter('right').get_parameter_value().string_value

        str_val4 = self.publisher.get_parameter('left').get_parameter_value().string_value
        
        char = event.char   # pressed key character
        
        if char != " ":     # accept only letters
            if char == str_val1:
                self.publisher.go_forward()

            if char == str_val2:
                self.publisher.go_backwards()

            if char == str_val3:
                self.publisher.rotate_right()

            if char == str_val4: 
                self.publisher.rotate_left()

    def run(self):
        self.root.mainloop()
