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
        self.root.geometry("%dx%d+%d+%d" % (500, 120, screen_width / 2, screen_height / 2))
        # Frame initialization
        self.imageFrame = None
        # Button initialization
        self.quitButton = None
        self.startButton = None
        # Text initialization
        self.show = None

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
        timeFrame.grid_columnconfigure(2, weight=1)
        timeFrame.grid_columnconfigure(4, weight=1)
        # draw Timer
        self.show = Label(timeFrame, text='00:00:00', font=('Helvetica', 30))
        self.show.grid(row=0, column=0, columnspan=5, sticky='EW')
        # setup buttons
        self.startButton = Button(timeFrame, text='Start Recording', height=2, width=20, command=self.start)
        self.startButton.grid(row=1, column=1)
        self.quitButton = Button(timeFrame, text='End Recording', height=2, width=20, command=self.quit)
        self.quitButton.grid(row=1, column=3)

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

    img = cv2.imread('/Volumes/shared_3/Project/functional-stn/Recording/220104-wt22/camera_034422070939/_Color_1641306006583.52099609375000.png')
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img_pil = Image.fromarray(img_rgb)
    imgtk = ImageTk.PhotoImage(image=img_pil)

    up = Stopwatch(root)
    root.mainloop()
