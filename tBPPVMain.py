# -*- coding: utf-8 -*-
"""
Spyder Editor

tBPPV, code by Jorge Rey-Martinez 2023.

"""
import pathlib
import configparser
import os
import deviceSelect
import tkinter as tk
import tkinter.ttk as ttk
from tkinter import messagebox
import pygubu
import time
from PIL import Image, ImageTk
from mbientlab.metawear import MetaWear,libmetawear, parse_value
from mbientlab.metawear.cbindings import *
from mbientlab.warble import * 
import six
import collections
import numpy as np

# Implement the default Matplotlib key bindings for plots.
from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg, NavigationToolbar2Tk)
from matplotlib.backend_bases import key_press_handler
from matplotlib.figure import Figure

PROJECT_PATH = pathlib.Path(__file__).parent
PROJECT_UI = PROJECT_PATH /"GUI"/ "mainTBPPV.ui"
PROJECT_CONFIG = PROJECT_PATH /"config.ini"
PROJECT_IMU_PIC = PROJECT_PATH /"GUI"/ "IMU.png"

class tBPPVMain:
    def __init__(self, master=None):
        # 1: Create a builder and setup resources path (if you have images)
        self.builder = builder = pygubu.Builder()
        builder.add_resource_path(PROJECT_PATH)

        # 2: Load an ui file
        builder.add_from_file(PROJECT_UI)

        # 3: Create the mainwindow
        self.mainwindow = builder.get_object('mainWindow', master)

        # 4: Connect tk objects and callbacks
        builder.connect_callbacks(self)
        self.imuCanvas = builder.get_object("imuCanvas")
        self.scanButton = builder.get_object("scanButton")
        self.connectButton = builder.get_object("connectButton")
        
        # IMU Callback
        self.readIMU = FnVoid_VoidP_DataP(self.streamIMU)

        
        # Other variables
        self.imuMac = ""
        self.canConnect = False
        self.isIMUConected = False
        self.delayEvents = 150 # milliseconds to GUI events
        self.imuDevice = None
        self.signal = None
        self.rawSample = None
        self.sample = None # sample data format = (time stamp in seconds,(quaternion w, quaternion x, quaternion y, quaternion z)) quaternion in normalized units
        self.timeZero = time.time()
        self.timeRecord = None
        self.timeConnect = None
        self.timeNow = None
        self.timeLast = None
        self.samplingInterval = 0.01 # minimum time in secs to read IMU data 0.01 are 65Hz aprox
        self.timeIMUSetup = 2.1 # delay in seconds to stream data to avoid IMU setup empty samples
        
        #load images
        aux = Image.open(PROJECT_IMU_PIC)
        self.imuImg = ImageTk.PhotoImage(aux)
        self.imuCanvas.create_image(4, 4, image=self.imuImg, anchor="nw")
        
        #Liveplot setup
        livePlot = builder.get_object('livePlot1')
        self.realDataTime= collections.deque(np.zeros(10))
        self.realDataX = collections.deque(np.zeros(10))
        
        # Setup matplotlib canvas
        ancho = 6
        alto = 3
        self.figureLivePlot = fig = Figure(figsize=(ancho, alto), dpi=100)
        self.canvasLivePlot = canvas = FigureCanvasTkAgg(fig, master=livePlot)
        canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)
        
        # Setup matplotlib toolbar (optional)
        self.toolbar = NavigationToolbar2Tk(canvas, livePlot)
        self.toolbar.update()
        canvas._tkcanvas.pack(side=tk.TOP, fill=tk.BOTH, expand=0)
        
        #debug plot
        #a = self.figureLivePlot.add_subplot(111)
        #a.plot(self.realDataTime,self.realDataX)
        #self.canvasLivePlot.draw()
        
    def loadConfig(self):
        config = configparser.ConfigParser()
        if not os.path.exists(PROJECT_CONFIG):
            print("Config file not found")
            config['IMU'] = {'mac': '', 'test2': 'probando123'}
            config.write(open(PROJECT_CONFIG, 'w'))
            #config['testing'] = {'test': '45', 'test2': 'yes'}
        else:
            print("Config file exists")
            config.read(PROJECT_CONFIG)
        
        self.imuMac = config.get("IMU","mac")
        if self.imuMac == "":
            print("No IMU mac, load search wizard")
            self.mainwindow.destroy()
            self.mainwindow.update()
            devSel = deviceSelect.deviceSelect()
            devSel.reloadMain = True
            devSel.run()
        else:
            print(self.imuMac)
            self.canConnect = True
            mac_lavel = self.builder.get_variable('mac_value')
            mac_lavel.set("IMU mac address to connect: " + self.imuMac)
            
    def scanIMU(self):
            if not self.isIMUConected:
                self.mainwindow.destroy()
                #self.mainwindow.update() #not sure what this line means !
                devSel = deviceSelect.deviceSelect()
                devSel.reloadMain = True #this will make te app to open when finish is over
                devSel.run()
            
    def connectIMU(self):
        if self.canConnect:
            self.imuDevice = MetaWear(self.imuMac)
            try:
                self.imuDevice.connect()
            except:
                print("IMU connection error")
            time.sleep(2.5)
            self.canConnect = False
            if self.imuDevice.is_connected:
                print("IMU connected")
                self.scanButton.state(["disabled"])
                self.connectButton.state(["disabled"])
                self.isIMUConected = True
                self.configureIMU()
            else:
                print("IMU is NOT connected")
                self.isIMUConected = False
                self.canConnect = True
                messagebox.showinfo("Warning", "Unable to connec to with IMU, try wake up or reset current IMU or selecting other IMU")
                
        else:
            print("IMU is already connected")
    
    def configureIMU(self):
        if self.imuDevice.is_connected:
            self.timeConnect = time.time() 
            self.timeLast = time.time()
            print("Setting up IMU...")
            # confiure connection
            libmetawear.mbl_mw_settings_set_connection_parameters(self.imuDevice.board, 7.5, 7.5, 0, 6000)
            time.sleep(1.5)
            # setup quaternion
            libmetawear.mbl_mw_sensor_fusion_set_mode(self.imuDevice.board, SensorFusionMode.NDOF);
            libmetawear.mbl_mw_sensor_fusion_set_acc_range(self.imuDevice.board, SensorFusionAccRange._8G)
            libmetawear.mbl_mw_sensor_fusion_set_gyro_range(self.imuDevice.board, SensorFusionGyroRange._2000DPS)
            libmetawear.mbl_mw_sensor_fusion_write_config(self.imuDevice.board)
            time.sleep(0.5)
            # get quat signal and subscribe
            self.signal = libmetawear.mbl_mw_sensor_fusion_get_data_signal(self.imuDevice.board, SensorFusionData.QUATERNION);
            libmetawear.mbl_mw_datasignal_subscribe(self.signal, None, self.readIMU)
            # start acc, gyro, mag
            libmetawear.mbl_mw_sensor_fusion_enable_data(self.imuDevice.board, SensorFusionData.QUATERNION);
            libmetawear.mbl_mw_sensor_fusion_start(self.imuDevice.board);
            print("IMU setup done.")
            time.sleep(0.25)

    
    def resetIMU(self):
        if self.isIMUConected:
            answer = messagebox.askokcancel("Be aware","Reset procedure will close the program and data will be lost. Do you want to continue ?")
            if answer:
                print("Erase logger, state, and macros")
                self.isIMUConected = False
                # stop
                libmetawear.mbl_mw_sensor_fusion_stop(self.imuDevice.board)
                libmetawear.mbl_mw_datasignal_unsubscribe(self.signal)
                time.sleep(0.5)
                #reset procedure
                libmetawear.mbl_mw_logging_stop(self.imuDevice.board)
                # Clear the logger of saved entries
                libmetawear.mbl_mw_logging_clear_entries(self.imuDevice.board)
                # Remove all macros on the flash memory
                libmetawear.mbl_mw_macro_erase_all(self.imuDevice.board)
                # Restarts the board after performing garbage collection
                libmetawear.mbl_mw_debug_reset_after_gc(self.imuDevice.board)
                libmetawear.mbl_mw_debug_disconnect(self.imuDevice.board)
                print("IMU reseted, app will close")
                time.sleep(10)
                self.imuDevice.disconnect()
                self.mainwindow.destroy()
        else:
            print("No IMU, no reset, man")
                    

    
    def streamIMU(self, ctx, data):
        self.timeNow = time.time()
        if (self.timeNow-self.timeLast) > self.samplingInterval: 
            if self.isIMUConected:
                if (self.timeLast-self.timeConnect) > self.timeIMUSetup: # add a little delay to stream data to avoid setup empty samples
                    self.rawSample = parse_value(data)
                    self.sample = ((self.timeLast-self.timeConnect)-self.timeIMUSetup,(self.rawSample.w,self.rawSample.x,self.rawSample.y,self.rawSample.z))
                    print(self.sample)
            self.timeLast = time.time()
            

    def plotSample(self):
        if self.isIMUConected:
            #print(self.sample)
            pass
        
    def quat_to_euler(self, w, x, y, z):
        ysqr = y * y

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + ysqr)
        X = np.degrees(np.arctan2(t0, t1))

        t2 = +2.0 * (w * y - z * x)

        t2 = np.clip(t2, a_min=-1.0, a_max=1.0)
        Y = np.degrees(np.arcsin(t2))

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (ysqr + z * z)
        Z = np.degrees(np.arctan2(t3, t4))

        return X, Y, Z  
    
        # # ALTERNATIVE
        # import pandas as pd
        # from scipy.spatial.transform import Rotation

        # rot = Rotation.from_quat(quat_df)
        # rot_euler = rot.as_euler('xyz', degrees=True)
        # euler_df = pd.DataFrame(data=rot_euler, columns=['x', 'y', 'z'])
        
    
    def loopEvents(self):
        #print("Event " + str(time.localtime().tm_sec))
        if self.isIMUConected:
            self.imuCanvas.configure(bg='#58D68D') #hexadecimal green
        else:
            self.imuCanvas.configure(bg='#DB0606') #hexadecimal red
        self.plotSample()
        self.mainwindow.after(self.delayEvents, self.loopEvents)
    
    def safeIMUDisconnect(self):            
        if not self.imuDevice is None:
                print("Closing IMU connection...")
                #stop
                libmetawear.mbl_mw_sensor_fusion_stop(self.imuDevice.board)
                libmetawear.mbl_mw_datasignal_unsubscribe(self.signal)
                time.sleep(0.5)
                # disconnect
                self.imuDevice.disconnect()
                time.sleep(3)
                self.isIMUConected = False
                print("IMU connection closed")
                
        else:
                print("No IMU connection to close")
                
    def on_exit(self):
        if self.isIMUConected:
            self.safeIMUDisconnect()
        self.mainwindow.destroy()
        
    def run(self):
        self.loadConfig()
        self.mainwindow.after(2000, self.loopEvents)
        self.mainwindow.protocol("WM_DELETE_WINDOW", self.on_exit)
        self.mainwindow.mainloop()

if __name__ == '__main__':
    app = tBPPVMain()
    app.run()