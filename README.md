# Yardstick
This project is a fully functional cloud connected yard watering status reporting station with Losant cloud integration and easy provosioning
## Rationale ##

[Losant](https://www.losant.com/) is an IoT cloud service provider.
This project is designed to deliver the following data to the cloud environment

1. Air temperature.      	This value is measured by a [DHT22] (https://www.sparkfun.com/datasheets/Sensors/Temperature/DHT22.pdf) sensor. Value is presented in units of degrees Celcius
2. Air humidity.         	This value is measured by a [DHT22] (https://www.sparkfun.com/datasheets/Sensors/Temperature/DHT22.pdf) sensor. Value is presented in units of Percentage Relative Humidity
3. Soil humidity level. 	This value is measured by a [ground resistance sensor] (https://www.banggood.com/Soil-Hygrometer-Humidity-Detection-Module-Moisture-Sensor-For-Arduino-p-79227.html?rmmds=search) and presented in percentage

The main paradigm is push the data into the cloud provider, then:

* Display results in graphs
* Send data to another device (Hidration computer)

## Features ##

Most important features of this project are:

- Sense agricultural parameters
- Push data to cloud
- Integrate with other devices on the cloud level

Side issues:

- Advanced debug modes for the code via a serial connection
- Learn how to document code with Doxygen
- Improve coding style
