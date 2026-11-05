"""MT6701/MPU6050串口曲线工具，依赖pyserial，界面使用Python自带Tk。"""
import csv
import math
import queue
import threading
import time
from collections import deque
from pathlib import Path
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import sys
sys.path.insert(0, str(Path(__file__).resolve().parent / "vendor"))
import serial
from serial.tools import list_ports

import re
FIELDS = ['ms','unused_target','left_speed','right_speed','left_voltage','right_voltage','unused1','unused2','unused3','unused4','unused5','unused6','running','unused7','unused8']
START=time.monotonic()
def parse_line(line):
    match=re.fullmatch(r'state=RUN wl=([-+0-9.eE]+) wr=([-+0-9.eE]+) ul=([-+0-9.eE]+) ur=([-+0-9.eE]+)',line.strip())
    if not match: return None
    try: wl,wr,ul,ur=map(float,match.groups())
    except ValueError: return None
    return [(time.monotonic()-START)*1000,float('nan'),wl,wr,ul,ur,float('nan'),float('nan'),float('nan'),0,0,0,1,0,0]

class Monitor:
    def __init__(self, root):
        self.root = root
        self.port = None
        self.incoming = queue.Queue(maxsize=5000)
        self.points = deque(maxlen=600)
        self.thread = None
        self.stopping = threading.Event()
        self.log_file = None
        root.title('原作者双电机测试曲线 | 转速与驱动电压')
        root.geometry('1100x850')
        toolbar = ttk.Frame(root)
        toolbar.pack(fill='x')
        self.choice = ttk.Combobox(toolbar, width=14)
        self.choice.pack(side='left', padx=5)
        ttk.Button(toolbar, text='刷新串口', command=self.refresh).pack(side='left')
        ttk.Button(toolbar, text='连接 / 断开', command=self.connect).pack(side='left')
        for title, char in [('停止电机 X','X')]:
            ttk.Button(toolbar, text=title, command=lambda c=char:self.send(c)).pack(side='left')
        self.state = tk.StringVar(value='先关闭VS Code串口监视器，再连接。打开串口可能复位ESP32并触发自动测试。')
        ttk.Label(root,textvariable=self.state).pack(fill='x')
        self.canvas = tk.Canvas(root,bg='#111827',highlightthickness=0)
        self.canvas.pack(fill='both',expand=True)
        self.logs = tk.Text(root,height=7)
        self.logs.pack(fill='x')
        self.refresh()
        root.protocol('WM_DELETE_WINDOW', self.close)
        self.update()

    def refresh(self):
        ports = [p.device for p in list_ports.comports()]
        self.choice['values'] = ports
        if ports and not self.choice.get():
            self.choice.set(ports[0])

    def connect(self):
        if self.port:
            self.disconnect()
            return
        if not self.choice.get():
            messagebox.showerror('串口','请选择ESP32串口')
            return
        try:
            self.port = serial.Serial(self.choice.get(),115200,timeout=0.1,write_timeout=0.2)
            folder = Path(__file__).resolve().parent/'logs'
            folder.mkdir(exist_ok=True)
            path = folder/time.strftime('%Y%m%d-%H%M%S.csv')
            self.log_file = path.open('w',newline='',encoding='utf-8')
            self.writer = csv.writer(self.log_file)
            self.writer.writerow(FIELDS)
            while not self.incoming.empty():
                self.incoming.get_nowait()
            self.points.clear()
            self.stopping.clear()
            self.thread = threading.Thread(target=self.read,args=(self.port,),daemon=True)
            self.thread.start()
            self.state.set(f'已连接 {self.choice.get()}，保存至 {path}')
        except (serial.SerialException,OSError) as exc:
            self.disconnect()
            messagebox.showerror('连接失败',str(exc))

    def read(self, port):
        buffer = b''
        try:
            while not self.stopping.is_set():
                buffer += port.read(port.in_waiting or 1)
                while b'\n' in buffer:
                    line, buffer = buffer.split(b'\n',1)
                    text = line.decode('utf-8',errors='replace')
                    try:
                        self.incoming.put_nowait(text)
                    except queue.Full:
                        pass
                if len(buffer)>8192:
                    buffer=b''
        except (serial.SerialException,OSError) as exc:
            self.incoming.put('串口中断 '+str(exc))

    def send(self,char):
        if self.port:
            try:
                self.port.write(char.encode())
            except (serial.SerialException,OSError) as exc:
                self.state.set(str(exc))

    def disconnect(self):
        self.send('X')
        self.stopping.set()
        if self.thread:
            self.thread.join(timeout=0.5)
        if self.port:
            self.port.close()
            self.port=None
        if self.log_file:
            self.log_file.close()
            self.log_file=None
        self.state.set('已断开')

    def update(self):
        for _ in range(500):
            try:
                line=self.incoming.get_nowait()
            except queue.Empty:
                break
            row=parse_line(line)
            if row is not None:
                self.points.append(row)
                if self.log_file:
                    self.writer.writerow(row)
                self.state.set(f'收到RUN数据 | 左轮 {row[2]:.2f} rad/s | 右轮 {row[3]:.2f} rad/s | 电压指令 {row[4]:.2f}/{row[5]:.2f} V')
            else:
                if line.startswith(('STOP', 'FAULT')):
                    self.state.set(line)
                self.logs.insert('end',line+'\n')
                if int(self.logs.index('end-1c').split('.')[0])>200:
                    self.logs.delete('1.0','100.0')
                self.logs.see('end')
        if self.log_file:
            self.log_file.flush()
        self.draw()
        self.root.after(100,self.update)

    def draw(self):
        self.canvas.delete('all')
        w,h=self.canvas.winfo_width(),self.canvas.winfo_height()
        charts=[('转速 rad/s  蓝色左轮 / 红色右轮',[(2,'#38bdf8'),(3,'#fb7185')]),
                ('驱动电压指令 V  蓝色左轮 / 红色右轮',[(4,'#38bdf8'),(5,'#fb7185')])]
        data=list(self.points)
        for panel,(title,series) in enumerate(charts):
            top=panel*h/2+30
            bottom=(panel+1)*h/2-20
            self.canvas.create_text(55,top-15,text=title,fill='white',anchor='w')
            values=[r[col] for r in data for col,_ in series if math.isfinite(r[col])]
            lo,hi=(min(values),max(values)) if values else (-1,1)
            pad=max((hi-lo)*0.1,0.2)
            lo-=pad;hi+=pad
            self.canvas.create_rectangle(55,top,w-15,bottom,outline='#475569')
            self.canvas.create_text(48,top,text=f'{hi:.1f}',fill='white',anchor='e')
            self.canvas.create_text(48,bottom,text=f'{lo:.1f}',fill='white',anchor='e')
            if len(data)<2:
                continue
            t0=data[0][0];dt=max(data[-1][0]-t0,1)
            for col,color in series:
                segment=[]
                for row in data:
                    value=row[col]
                    if math.isfinite(value):
                        segment.extend([55+(w-70)*(row[0]-t0)/dt,bottom-(bottom-top)*(value-lo)/(hi-lo)])
                    else:
                        if len(segment)>=4:self.canvas.create_line(*segment,fill=color,width=2)
                        segment=[]
                if len(segment)>=4:self.canvas.create_line(*segment,fill=color,width=2)

    def close(self):
        self.disconnect()
        self.root.destroy()

if __name__=='__main__':
    app=tk.Tk()
    Monitor(app)
    app.mainloop()
