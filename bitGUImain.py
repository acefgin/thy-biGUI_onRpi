import tkinter as tk
import tkinter.font as tkFont
from tkinter.ttk import Progressbar
from tkinter import Tk, Canvas, Frame, Grid, Label, Button
import subprocess
from bitTest import *


import PIL.Image
import PIL.ImageTk
from PIL import ImageSequence

from multiprocessing import Process, Value, Array, Queue
import time

animateDELAY = 200
DELAY1 = 100

class gifAnimation():
    def __init__(self, master, path):
        self.master = master
        self.canvas = Canvas(master, width = 320, height = 376)
        self.canvas.grid(row = 0, column = 0)
        self.imSeq = [PIL.ImageTk.PhotoImage(img)
                        for img in ImageSequence.Iterator(PIL.Image.open(path))]
        self.image = self.canvas.create_image(160, 188, image = self.imSeq[0])
        self.timerDisplay = ""
        self.text = self.canvas.create_text(160, 188, fill = "white", text = self.timerDisplay, font=("Calibri", 32), justify = "center")

        self.animating = True
        self.frameNum = 0
        self.animate()

    def animate(self):
        self.canvas.itemconfig(self.image, image = self.imSeq[self.frameNum])
        self.canvas.itemconfig(self.text, text = self.timerDisplay)
        if not self.animating:
            return
        self.frameNum = (self.frameNum + 1) % len(self.imSeq)
        self.master.after(animateDELAY, lambda: self.animate())

    def updateGIF(self, path):
        self.imSeq = [PIL.ImageTk.PhotoImage(img)
                        for img in ImageSequence.Iterator(PIL.Image.open(path))]
        self.canvas.itemconfig(self.image, image = self.imSeq[0])
        self.frameNum = 0

    def displayText(self, text):
        self.timerDisplay = text


class bitGUIapp():
    def __init__(self, master, timerStatus, results):
        self.master = master
        self.frame1 = Frame(master)
        self.frame1.grid(row = 0, column = 0, columnspan = 3)
        # Create dynamic font for text
        self.dfont = tkFont.Font(size=-24)
        self.fullscreen = True

        self.currentStep = 0
        master.attributes('-fullscreen', self.fullscreen)
        master.title("A simple GUI for DNA bit")

        self.animation1 = gifAnimation(self.frame1,"/home/pi/bit/v6board/bit/bitGUImultiProcess/medias/items_loading.gif")
        self.bn_frame1 = Frame(master, bg = "#000064", width = 320, height = 78)
        self.bn_frame1.grid_propagate(False)
        self.bn_frame1.pack_propagate(False)
        self.bn_frame1.grid(row = 1, column = 0, columnspan = 3)
        self.bn_frame1.rowconfigure(0, weight=10)

        self.next_button = Button(self.bn_frame1, bg = "#000064",fg="white", text="Next", font=("Calibri", 26), command=self.itemsloadingNx)
        self.next_button.pack(fill = "both", expand = 1)

        self.frame2 = Frame(master, width = 16, height = 24)
        self.frame2.grid_propagate(False)
        self.frame2.pack_propagate(False)
        self.frame2.grid(row = 2, column = 0)
        self.demo_button = Button(self.frame2, bg = "#000064",fg="blue", text="", font=("Calibri", 12), command=self.demoTaggle)
        self.demo_button.pack(fill = "both", expand = 1)

        self.frame3 = Frame(master, width = 288, height = 24)
        self.frame3.grid_propagate(False)
        self.frame3.pack_propagate(False)
        self.frame3.grid(row = 2, column = 1)

        self.status = Label(self.frame3, bg = "#000064", fg="white", text="Current status: IDLE", anchor="w", justify="left",font=("Calibri", 12))
        self.status.pack(fill = "both", expand = 1)

        self.frame4 = Frame(master, width = 16, height = 24)
        self.frame4.grid_propagate(False)
        self.frame4.pack_propagate(False)
        self.frame4.grid(row = 2, column = 2)
        self.close_button = Button(self.frame4, bg = "#000064",fg="red", text="X", font=("Calibri", 12), command=self.shutdown)
        self.close_button.pack(fill = "both", expand = 1)

        # Create process but not run it yet
        self.timerStatus = timerStatus
        self.rts = results
        self.DEMO = False

        self.lysis_process = Process(target = lysisRunSA, args=(self.timerStatus,))
        self.detec_process = Process(target = react_detectRunSA, args=(self.timerStatus,self.rts))

    def demoTaggle(self):
        self.status['text'] = "Current status: Demo mode"
        self.master.update_idletasks()
        self.DEMO = True
        self.lysis_process = Process(target = lysisDemo, args=(self.timerStatus,))
        self.detec_process = Process(target = react_detectDemo, args=(self.timerStatus,self.rts))

    def shutdown(self):
        subprocess.call(["sudo", "shutdown", "-h", "now"])
    def itemsloadingNx(self):

        self.animation1.updateGIF("/home/pi/bit/v6board/bit/bitGUImultiProcess/medias/swab_and_dip.gif")
        self.next_button['command'] = self.swabdipNx
        self.master.update_idletasks()


    def swabdipNx(self):

        self.animation1.updateGIF("/home/pi/bit/v6board/bit/bitGUImultiProcess/medias/mix_load.gif")
        self.next_button['command'] = self.mixloadNx
        self.next_button['text'] = "Lysis sample"
        self.master.update_idletasks()

    def mixloadNx(self):

        self.lysis_process.start()
        self.lysisWait()

    def nxBeforeDetection(self):
        self.animation1.displayText("")
        self.animation1.updateGIF("/home/pi/bit/v6board/bit/bitGUImultiProcess/medias/CEACfilling.gif")
        self.next_button['command'] = self.CEACfillingNx
        self.next_button['text'] = "Start detection"
        self.master.update_idletasks()


    def CEACfillingNx(self):
        self.detec_process.start()
        self.detectWait()

    def doneNx(self):
        self.animation1.updateGIF("/home/pi/bit/v6board/bit/bitGUImultiProcess/medias/items_loading.gif")
        self.next_button['command'] = self.itemsloadingNx
        self.next_button['text'] = "Next"
        self.DEMO = False

        self.lysis_process = Process(target = lysisRunSA, args=(self.timerStatus,))
        self.detec_process = Process(target = react_detectRunSA, args=(self.timerStatus,self.rts))
        self.master.update_idletasks()

    def checkForLysisTimerStop(self):
        if self.lysis_process.is_alive():
            self.master.after(DELAY1, self.checkForLysisTimerStop)
            return
        else:
            self.status['text'] = "Current status: IDLE"
            self.next_button['command'] = self.nxBeforeDetection
            self.next_button['text'] = "Next"
            self.next_button['state'] = 'normal'
            self.master.update_idletasks()
            while not self.timerStatus.empty():
                self.timerStatus.get()

    def checkForLysisTimerStart(self):
        if self.timerStatus.empty():
            self.master.after(DELAY1, self.checkForLysisTimerStart)
            return
        elif self.timerStatus.get() == 2:
            if self.DEMO:
                self.demoTimer()
            else:
                self.lysisTimer()
            self.checkForLysisTimerStop()

    def checkForDetectTimerStop(self):
        if self.detec_process.is_alive():
            self.master.after(DELAY1, self.checkForDetectTimerStop)
            return
        else:
            self.detec_process.join()
            self.animation1.displayText("")

            if self.DEMO:
                self.is_salmon()
            else:
                if self.rts[0] == 0 and self.rts[4] == 1:
                    if self.rts[1] or self.rts[2] or self.rts[3]:
                        self.is_salmon()
                    else:
                        self.is_not_salmon()
                else:
                    self.reaction_error()

            self.next_button['state'] = 'normal'
            self.status['text'] = "Current status: IDLE"
            self.next_button['command'] = self.doneNx
            self.next_button['text'] = "Detection done"
            self.master.update_idletasks()

    def checkForDetectTimerStart(self):
        if self.timerStatus.empty():
            self.master.after(DELAY1, self.checkForDetectTimerStart)
            return
        elif self.timerStatus.get() == 2:
            if self.DEMO:
                self.demoTimer()
            else:
                self.detectTimer()
            self.checkForDetectTimerStop()

    def lysisWait(self):
        self.status['text'] = "Current status: lysis is running..."
        self.animation1.displayText("Initiating")
        self.next_button['state'] = 'disable'
        self.animation1.updateGIF("/home/pi/bit/v6board/bit/bitGUImultiProcess/medias/loading.gif")
        self.master.update_idletasks()
        self.checkForLysisTimerStart()

    def detectWait(self):
        self.status['text'] = "Current status: reaction is running..."
        self.animation1.displayText("Initiating")
        self.next_button['state'] = 'disable'
        self.animation1.updateGIF("/home/pi/bit/v6board/bit/bitGUImultiProcess/medias/loading.gif")
        self.master.update_idletasks()
        self.checkForDetectTimerStart()

    def lysisTimer(self):
        self.timer(5, 0)

    def detectTimer(self):
        self.timer(40, 0)

    def demoTimer(self):
        self.timer(1, 0)

    def timer(self, min, sec):

        if sec < 10:
            self.animation1.displayText(str(min) + ":0" + str(sec))

        else:
            self.animation1.displayText(str(min) + ":" + str(sec))

        if min != 0:
            if sec == 0:
                min = min - 1
                sec = 59
            else:
                sec = sec - 1
        else:
            if sec == 0:
                self.animation1.displayText("Finishing")
                return
            else:
                sec = sec - 1
        self.master.after(983, lambda: self.timer(min, sec))


    def is_salmon(self):
        self.animation1.updateGIF("/home/pi/bit/v6board/bit/bitGUImultiProcess/medias/Pos_rts.png")

    def is_not_salmon(self):
        self.animation1.updateGIF("/home/pi/bit/v6board/bit/bitGUImultiProcess/medias/Neg_rts.png")

    def reaction_error(self):
        self.animation1.updateGIF("/home/pi/bit/v6board/bit/bitGUImultiProcess/medias/errors.png")


    def barUpdate(self, newVal):

        self.progress['value']=newVal
        self.master.update_idletasks()

    # Toggle fullscreen
    def toggle_fullscreen(self, event=None):

        # Toggle between fullscreen and windowed modes
        self.fullscreen = not self.fullscreen
        self.master.attributes('-fullscreen', self.fullscreen)
        self.resize()

    # Return to windowed mode
    def end_fullscreen(self, event=None):

        # Turn off fullscreen mode
        self.fullscreen = False
        self.master.attributes('-fullscreen', False)
        self.resize()

    # Automatically resize font size based on window size
    def resize(self, event=None):

        # Resize font based on frame height (minimum size of 12)
        # Use negative number for "pixels" instead of "points"
        new_size = -max(12, int((self.frame1.winfo_height() / 10)))
        self.dfont.configure(size=new_size)

    def lysis_worker(self, cnt, timerStatus):
        #lysisRunSA()
        #self.worker(cnt, timerStatus)
        counter = cnt
        while counter != 0:
            if counter >= 40:
                msg = 1
                timerStatus.put(msg)
            else:
                msg = 2
                timerStatus.put(msg)
            time.sleep(1)
            print(str(counter) + ":" + str(msg))
            counter = counter - 1

if __name__ == "__main__":

    timerStatus = Queue()
    results = Array('i', [0, 0, 0, 0, 1])

    root = Tk()
    root.geometry("320x480")
    bit_gui = bitGUIapp(root, timerStatus, results)

    # Bind F11 to toggle fullscreen and ESC to end fullscreen
    root.bind('<F11>', bit_gui.toggle_fullscreen)
    root.bind('<Escape>', bit_gui.end_fullscreen)

    # Have the resize() function be called every time the window is resized
    root.bind('<Configure>', bit_gui.resize)

    root.mainloop()
