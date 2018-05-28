# Autonomous Robot-Car Platoons System 
Version -> Alpha 0.1

more information in [our project report](https://github.com/BankyKmitl/SeniorProject/blob/master/57070045_57070148_report.pdf) 
## Source Code

#### Raspberry Pi / Robot-Car (Python 2.7)
ใช้ [Raspbian OS](https://www.raspberrypi.org/downloads/raspbian/) ตัวเต็ม แล้ว Clone Repository นี้ลงใน Raspberry Pi

การเชื่อมต่อกับ Rpi จะใช้การ SSH ผ่านโปรแกรม เช่น [Putty](https://www.putty.org/) จากนั้นอาจใช้การดึงหน้าจอโดยใช้ [realvnc](https://www.realvnc.com/en/connect/download/viewer/) เพื่อใช้ในการ coding

โฟลเดอร์ Rpi คือ Code ที่อยู่ในตัว Raspberry Pi ทั้งหมด เขียนด้วย Python 2.7 จำเป็นต้องลง Library ต่างๆ เพื่อให้สามารถใช้งานได้ เช่น
- [Bluez](http://www.bluez.org/download/)
- [pygattlib](https://bitbucket.org/OscarAcena/pygattlib)
- [RPi.GPIO](https://pypi.org/project/RPi.GPIO/)
- [netpie microgear](https://github.com/netpieio/microgear-python)

run file โดยใช้ terminal รัน **sudo python main.py** (จำเป็นต้องเชื่อมต่อ Internet เพื่อเชื่อมต่อกับ NETPIE)

#### NETPIE และ Freeboard

เนื่องจากใช้ NETPIE เป็น Broker จึงต้องสมัครสมาชิกเพื่อใช้งาน
จากนั้นสร้าง Application ขึ้นมา แล้ว generate Key ตามอุปกรณ์และหน้าเว็บที่ต้องใช้ ดังนี้
- Device Key สำหรับ Robot-car แต่ละคัน
- Session Key สำหรับ Freeboard (1 หน้า Freeboard ต่อรถ 1 คัน)
- Session Key สำหรับ Tracking Map (1 หน้า Tracking Map ต่อรถ 1 คัน)

ตัวอย่างเช่น
![ef5388ee38de16b254c9ab3b29c8729d.png](https://www.img.in.th/images/ef5388ee38de16b254c9ab3b29c8729d.png)

จากนั้นสร้าง Freeboard ขึ้นมา แล้ว Import ไฟล์ freeboard.json ในโฟลเดอร์ NETPIE เพื่อใช้เป็น Template สำหรับหน้าควบคุมรถแต่ละคัน
(code ที่เขียนในแต่ละ widget ของ Freeboard ใช้ Javascript)

#### Tracking Map (Javascript)
ใช้ library ที่ชื่อว่า [tiny-turtle](https://github.com/toolness/tiny-turtle) มาดัดแปลงเพิ่มเติม (html หนึ่งไฟล์ ต่อรถ 1 คัน)

Source code อยู่ในโฟลเดอร์ tinyturtle


## Hardware (Robot-Car & Sensors)
- Raspberry Pi 3 Model B จ่ายไฟด้วย Powerbank 2A
- L298N Motor Driver Module [Tutorial](https://www.bluetin.io/python/gpio-pwm-raspberry-pi-h-bridge-dc-motor-control/)
- HC-020K Speed Sensor [How to detect speed (event detect concept)](http://raspi.tv/2014/rpi-gpio-update-and-detecting-both-rising-and-falling-edges)
- HC-SR04 Ultrasonic Sensor [Tutorial](https://www.modmypi.com/blog/hc-sr04-ultrasonic-range-sensor-on-the-raspberry-pi)
- GY-521 Accelerometer/Gyro Module [Tutorial](http://www.electronicwings.com/raspberry-pi/mpu6050-accelerometergyroscope-interfacing-with-raspberry-pi)

** เนื่องจาก Rpi3 ใช้ไฟ 3.3 v แต่ HC-020K และ HC-SR04 ใช้ไฟ 5v จึงต้องใช้ [Logic Level Converter Module](https://www.arduinoall.com/product/259/logic-level-converter-module) แปลงไฟก่อนค่อยต่อเข้ากับ Rpi

## Bug / Unfinished Task or Feature / Advice
- ถ้า Robot-Car ถอยหลัง ควรจะลบระยะทางลง ไม่ใช่เพิ่มขึ้น
- ย้าย class SpeedSensor และ Iot ในไฟล์ main.py ไปเป็น module
- code ส่วนควบคุม Robot-car ที่ต้อง Tuning คือ เนื่องจากประกอบรถขึ้นเอง จึงมีความไม่เป็นมาตรฐานที่เหมือนกันของรถทุกคัน จึงต้องมีการปรับแก้ code ให้คุมรถแต่ละคันให้ได้เหมาะสม คำแนะนำคือ อาจต้องเปลี่ยนไปใช้ Robot-car ที่สำเร็จรูป มีมาตรฐาน
- Ultrasonic Sensor ยังจับระยะได้ไม่ดีพอ อาจต้องเปลี่ยนไปใช้ sensor รูปแบบอื่น
- ควรเขียนให้รันไฟล์ main.py แบบเป็น script คือให้ autorun ตั้งแต่เปิด Rpi
- การเลือกใช้ NETPIE เป็น broker มีข้อจำกัดเรื่อง จำนวนข้อมูลที่ส่งได้ (20 message / sec) อาจทำให้ได้ข้อมูลที่ไม่ realtime มากพอ

***
#### If you have any question or any problem. Please Contact me
- Tel. 0894583085
- thanakrit.p39@gmail.com
