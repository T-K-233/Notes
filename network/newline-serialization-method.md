# NewLine Serialization Method



## Arduino Library

###

### Initialization

```
NLSMSerial.init(115200);
```



### Receive

```
uint8_t buffer[128];
uint16_t rx_size = NLSMSerial.receive(buffer, 128);
```



### Transmit

```
uint8_t buffer[128];
uint16_t tx_size = 1;
NLSMSerial.transmit(buffer, tx_size);
```



## Python

```
import serial

class NLSMSerial:
    END = b"\x0A"
    ESC = b"\x1B"
    ESC_END = b"\x1C"
    ESC_ESC = b"\x1D"
    def __init__(self, COM=None, baudrate=115200, timeout=0):
        self._COM = COM
        if not self._COM:
            self._COM = "COM6"
            
        self._ser = serial.Serial(port=self._COM, baudrate=baudrate, timeout=timeout)

    def transmit(self, buffer):
        index = 0
        while index < len(buffer):
            c = struct.pack("B", buffer[index])
            if c == NLSMSerial.END:
                self._ser.write(NLSMSerial.ESC)
                self._ser.write(NLSMSerial.ESC_END)
            elif c == NLSMSerial.ESC:
                self._ser.write(NLSMSerial.ESC)
                self._ser.write(NLSMSerial.ESC_ESC)
            else:
                self._ser.write(c)
            index += 1
        self._ser.write(NLSMSerial.END)

    def receive(self):
        c = b""
        buffer = b""
        while c != NLSMSerial.END:
            if c == NLSMSerial.ESC:
                c = self._ser.read(1)
                if c == NLSMSerial.ESC_END:
                    buffer += NLSMSerial.END
                elif c == NLSMSerial.ESC_ESC:
                    buffer += NLSMSerial.ESC
                else:
                    buffer += c
            else:
                buffer += c
            c = self._ser.read(1)
            if c == b"":
                return buffer
        return buffer
```





## Example Usage: Echo

### Arduino

```
#include "NLSMSerial.h"

uint8_t buffer[128];

void setup() {
  NLSMSerial.init(115200);
}

void loop() {
  uint16_t recv_size = NLSMSerial.receive(buffer, 128);
  NLSMSerial.transmit(buffer, recv_size);
}
```



### Python

```
import struct
import logging
import json
import time

import random
import threading

import serial
import serial.tools.list_ports

class NLSMSerial:
    END = b"\x0A"
    ESC = b"\x1B"
    ESC_END = b"\x1C"
    ESC_ESC = b"\x1D"
    def __init__(self, COM=None, baudrate=115200, timeout=0):
        self._COM = COM
        if not self._COM:
            self._COM = "COM6"
            
        self._ser = serial.Serial(port=self._COM, baudrate=baudrate, timeout=timeout)

    def transmit(self, buffer):
        index = 0
        while index < len(buffer):
            c = struct.pack("B", buffer[index])
            if c == NLSMSerial.END:
                self._ser.write(NLSMSerial.ESC)
                self._ser.write(NLSMSerial.ESC_END)
            elif c == NLSMSerial.ESC:
                self._ser.write(NLSMSerial.ESC)
                self._ser.write(NLSMSerial.ESC_ESC)
            else:
                self._ser.write(c)
            index += 1
        self._ser.write(NLSMSerial.END)

    def receive(self):
        c = b""
        buffer = b""
        while c != NLSMSerial.END:
            if c == NLSMSerial.ESC:
                c = self._ser.read(1)
                if c == NLSMSerial.ESC_END:
                    buffer += NLSMSerial.END
                elif c == NLSMSerial.ESC_ESC:
                    buffer += NLSMSerial.ESC
                else:
                    buffer += c
            else:
                buffer += c
            c = self._ser.read(1)
            if c == b"":
                return buffer
        return buffer

rx_counter = 0
tx_counter = 0
recv_data = b""


def tx_handler():
    global tx_counter
    counter = 0
    while True:
        counter += 1
        
        buffer = b"hello world"+struct.pack("B", counter % 256)
        ser.transmit(buffer)
        tx_counter += 1


def rx_handler():
    global rx_counter
    global recv_data
    while True:
        recv = ser.receive()
        if not recv:
            continue
        recv_data = recv
        rx_counter += 1

ser = NLSMSerial(timeout=0.1)
time.sleep(2)
print("connected")

tx = threading.Thread(target=tx_handler)
rx = threading.Thread(target=rx_handler)
tx.start()
rx.start()

counter = 0
while True:
    print(tx_counter, rx_counter, tx_counter-rx_counter, recv_data)

    

```
