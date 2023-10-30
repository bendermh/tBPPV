# -*- coding: utf-8 -*-
"""
Spyder Editor

tBPPV, code by Jorge Rey-Martinez 2023 .

"""

import pathlib
import tkinter as tk
import tkinter.ttk as ttk
from tkinter import messagebox
import pygubu
import time
from mbientlab.metawear import MetaWear,libmetawear
from mbientlab.metawear.cbindings import *
from mbientlab.warble import * 
import six
import tBPPVMain
import configparser

PROJECT_PATH = pathlib.Path(__file__).parent
PROJECT_UI = PROJECT_PATH /"GUI"/ "deviceSelect.ui"
PROJECT_CONFIG = PROJECT_PATH /"config.ini"

class deviceSelect:
    def __init__(self, master=None):
        # 1: Create a builder and setup resources path (if you have images)
        self.builder = builder = pygubu.Builder()
        builder.add_resource_path(PROJECT_PATH)
        self.imuAdr = None
        self.devices = None
        self.reloadMain = False
        self.canSave = False 
        
        # 2: Load an ui file
        builder.add_from_file(PROJECT_UI)

        # 3: Create the mainwindow
        self.mainwindow = builder.get_object('mainwindow', master)
        self.mainwindow.focus()
        # 4: Connect callbacks
        builder.connect_callbacks(self)
        self.textConsole = builder.get_object("consoTx")
        self.enterDev = builder.get_object("devNum")
        self.textConsole.insert("0.0", "Turn on bluetooth and IMU \nPress SCAN when ready!\n\n")
    

    def blink(self):
        try:
            self.devices
        except AttributeError:
            messagebox.showinfo("Warning", "Pleease, press SCAN button")
            return
        i = 0
        mac = None
        for imu in six.iteritems(self.devices):
            if str(i) == self.enterDev.get():
                mac = imu
            i += 1
        if mac is not None:
            if mac[1] != "MetaWear":
                messagebox.showinfo("Warning", "Selected device is not a valid IMU")
                return
            else:
                self.imuAdr = mac[0]
        else:
            messagebox.showinfo("Warning", "No BT device was selected !")
            return
        self.textConsole.configure(state='normal')
        self.textConsole.insert("0.0", "Trying to connect IMU: " + self.imuAdr + "\nIf there is not response check device status \n(battery) and close this window...\n\n")
        self.textConsole.update()
        device = MetaWear(self.imuAdr)
        device.connect()
        self.textConsole.insert("0.0","Connected to IMU: "+ device.address +"\nIMU device will blink: \n-If is not the desired device select other \ndevice and test it again. \n-If it is correct close this window\n\n")
        self.textConsole.configure(state='disabled')
        self.textConsole.update()
        # create led pattern
        
        pattern= LedPattern(delay_time_ms= 400, repeat_count= 10)
        libmetawear.mbl_mw_led_load_preset_pattern(byref(pattern), LedPreset.BLINK)
        libmetawear.mbl_mw_led_write_pattern(device.board, byref(pattern), LedColor.BLUE)

        # play the pattern
        libmetawear.mbl_mw_led_play(device.board)

        # wait 5s
        time.sleep(4.0)

        # remove the led pattern and stop playing
        libmetawear.mbl_mw_led_stop_and_clear(device.board)
        time.sleep(2.0)

        # disconnect
        device.disconnect()
        time.sleep(1.0)
        print("Closed")
        self.canSave = True
    
    def saveExit(self):
        if not self.canSave:
            messagebox.showinfo("Warning", "There is not valid IMU selected, please scan and test again")
            return
        else:
            config = configparser.ConfigParser()
            config.read(PROJECT_CONFIG)
            config.set("IMU", "mac", self.imuAdr)
            config.write(open(PROJECT_CONFIG, 'w'))
            self.mainwindow.destroy()
            self.mainwindow.update()
            if self.reloadMain:
                newMain = tBPPVMain.tBPPVMain()
                newMain.run()
            
        

    def scan(self):
        #messagebox.showinfo("Warning", "Bluetooth not working !")
        self.textConsole.configure(state='normal')
        self.textConsole.delete("0.0", tk.END)
        self.textConsole.insert("0.0", "Scanning devices... (please wait)\n")
        self.textConsole.update()
        listDev = {}
        def handler(result):
            listDev[result.mac] = result.name
        BleScanner.set_handler(handler)
        BleScanner.start()
        time.sleep(10.0)
        BleScanner.stop()
        i = 0
        for address, name in six.iteritems(listDev):
            device = str("[%d] %s (%s) \n" % (i, address, name))
            self.textConsole.insert("0.0", device)
            if name == "MetaWear":
                #self.imuAdr = (address, name)
                self.enterDev.delete(0)
                self.enterDev.insert(0, i)
                self.enterDev.update()
            i+= 1
        self.devices = listDev
        self.textConsole.configure(state='disabled')


    def run(self):
        self.mainwindow.mainloop()

if __name__ == '__main__':
    app = deviceSelect()
    app.run()