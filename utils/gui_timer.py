import tkinter
from tkinter import *
import cv2
from PIL import Image
from PIL import ImageTk


# This program is designed to count up from zero
class Stopwatch(Frame):
    def __init__(self, master=None):
        Frame.__init__(self, master)
        self.root = master
        screen_width = self.root.winfo_screenwidth()
        screen_height = self.root.winfo_screenheight()
        self.root.geometry("%dx%d+%d+%d" % (660, 500, screen_width / 2, screen_height / 2))
        # Frame initialization
        self.imageFrame = None
        # Button initialization
        self.quitButton = None
        self.startButton = None
        # Text initialization
        self.show = None
        self.imageGrid = [[]]*4

        # place widgets
        self.widgets()
        # start timer
        self.timer = [0, 0, 0]  # [minutes ,seconds, centiseconds]
        self.timeString = f'{self.timer[0]:02d}:{self.timer[1]:02d}:{self.timer[2]:02d}'
        self.running = False
        self.update_time()

    def widgets(self):
        self.root.grid_columnconfigure(0, weight=1)
        # setup frames
        timeFrame = LabelFrame(self.root, text='Recording in Progress')
        timeFrame.grid(row=0, sticky='NSEW')
        timeFrame.grid_columnconfigure(0, weight=1)
        timeFrame.grid_columnconfigure(1, weight=1)
        self.timeFrame = timeFrame

        # draw Timer
        self.imageGrid[0] = Label(timeFrame)
        self.imageGrid[0].grid(row=0, column=0, sticky='W')
        self.imageGrid[1] = Label(timeFrame)
        self.imageGrid[1].grid(row=0, column=1, sticky='E')
        self.imageGrid[2] = Label(timeFrame)
        self.imageGrid[2].grid(row=1, column=0, sticky='W')
        self.imageGrid[3] = Label(timeFrame)
        self.imageGrid[3].grid(row=1, column=1, sticky='E')

        # draw Timer
        self.show = Label(timeFrame, text='00:00:00', font=('Helvetica', 30))
        self.show.grid(row=2, column=0, columnspan=2, sticky='EW')
        # setup buttons
        self.startButton = Button(timeFrame, text='Start Recording', height=2, width=20, command=self.start)
        self.startButton.grid(row=3, column=0)
        self.quitButton = Button(timeFrame, text='End Recording', height=2, width=20, command=self.quit)
        self.quitButton.grid(row=3, column=1)


    def update_time(self):
        if self.running:  # Clock is running
            self.timer[2] += 1  # Count Down
            if self.timer[2] >= 100:  # 100 centiseconds --> 1 second
                self.timer[2] = 0  # reset to zero centiseconds
                self.timer[1] += 1  # add 1 second
            if self.timer[1] >= 60:  # 60 seconds --> 1 minute
                self.timer[0] += 1  # add 1 minute
                self.timer[1] = 0  # reset to 0 seconds
            self.timeString = f'{self.timer[0]:02d}:{self.timer[1]:02d}:{self.timer[2]:02d}'
            self.show.config(text=self.timeString)
        self.root.after(1000, self.update_time)


    def start(self):
        pass


    def quit(self):  # Quit the program
        self.root.destroy()


if __name__ == "__main__":
    root = tkinter.Tk()
    up = Stopwatch(root)
    root.mainloop()
