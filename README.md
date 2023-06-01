# STM32_WITH_HMC5883L
Interfacing HMC5883L sensor with STM32 using I2C Interfacing

I was stuck on this for too long, had to connect around 4K resistor to SCL line externally, even if internally it is pulled up.
#	Wiring


  PB8 - SCL (I2C1_SCL)
  
  PB9 - SDA (I2C1_SDA)
  
  5V - for HMC5883L
  
  GND - GND

  ALSO EXTERNALL RESISTOR OF 4k OHM TO SCL AND VCC


# For logic Analyzer Wiring is :

 * GND TO CH8
 * SDA TO CH1
 * SCL TO CH2

# Result From Logic Analyzer

![chapter_closed](https://github.com/abhisheeekkk/STM32_WITH_HMC5883L/assets/91733340/1cceeb0c-b5b2-4753-a853-91ec5dbf5aaa)
