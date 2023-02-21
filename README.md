# Windows EVE2, EVE3 AND EVE4 TEST DEMO SOFTWARE

A quick and easy sanity check to ensure that your Matrix Orbital EVE2, EVE3 or EVE4 SPI TFT Display and touch hardware works properly. This software supports the EVE3x and EVE4x modules as well.

Support Tool

------------------------------------------------------------------

**Hardware Requirements:**

![alt text](https://www.matrixorbital.com/image/cache/catalog/products/EVE/EVE3-43G-300x300.jpg)

**EVE4x SPI TFT** - Uses BT817/BT818 Graphics controller

https://www.matrixorbital.com/ftdi-eve/eve-bt817-bt818

**OR**

**EVE3 & EVE3x SPI TFT** - Uses BT815/BT816 Graphics controller

https://www.matrixorbital.com/ftdi-eve/eve-bt815-bt816

**OR**

**EVE2 SPI TFT** - Uses FT812/FT813 Graphics Controller

https://www.matrixorbital.com/ftdi-eve/eve-ft812

**AND**

An EVE2-USB2SPI Bridge

https://www.matrixorbital.com/ftdi-eve/EVE2-USB2SPI-KIT-A

![alt text](https://www.matrixorbital.com/image/cache/catalog/products/EVE2%20USB%20to%20SPI%20Module-250x250.jpg)


**SOFTWARE**

**Builing**

The sample code is provided as a cmake based project, on windows all dependencies are included, on linux the `libftdi1-dev` package is required. 

for both windows and linux: 
```
git clone https://github.com/MatrixOrbital/Eve-Sample-Code.git
cd Eve-Sample-Code
mkdir build
cd build
cmake ..
cmake --build .
```
**Running**

Select the binary generated for your specific EVE2, EVE3 or EVE4 variant to test both the TFT and touch functionality of your display

1. Select [**EVE2**](https://www.matrixorbital.com/ftdi-eve/eve-ft812) or [**EVE3**](https://www.matrixorbital.com/ftdi-eve/eve-bt815-bt816) or [**EVE4**](https://www.matrixorbital.com/ftdi-eve/eve-bt817-bt818)

2. Select your display size:

* [**29** - 2.9" 320 x 102 TFT](https://www.matrixorbital.com/eve2-29a)
* [**35** - 3.5" 320 x 240 TFT](https://www.matrixorbital.com/index.php?route=product/search&search=eve3-35)
* [**38** - 3.8" 480 x 116 TFT](https://www.matrixorbital.com/index.php?route=product/search&search=eve2-38)
* [**39** - 3.9" 480 x 128 TFT](https://www.matrixorbital.com/index.php?route=product/search&search=eve3x-39)
* [**40** - 4.0" 720 x 720 TFT](https://www.matrixorbital.com/index.php?route=product/search&search=eve4x-40)
* [**43** - 4.3" 480 x 272 TFT](https://www.matrixorbital.com/index.php?route=product/search&search=eve3-43)
* [**50** - 5.0" 800 x 480 TFT](https://www.matrixorbital.com/index.php?route=product/search&search=eve3-50)
* [**52** - 5.2" 800 x 128 TFT](https://www.matrixorbital.com/index.php?route=product/search&search=eve3-52)
* [**70** - 7.0" 800 x 480 TFT](https://www.matrixorbital.com/index.php?route=product/search&search=eve3-70)
* [**70** - 7.0" 1024 x 600 TFT](https://www.matrixorbital.com/index.php?route=product/search&search=eve4x-70)
* [**101** - 10.1" 1280 x 800 TFT](https://www.matrixorbital.com/index.php?route=product/search&search=eve4x-101)


3. Select your touch screen

* **TPN** - No touch panel
* **TPR** - Resistive touch panel. Uses FT812/BT816/BT818 Graphics controller
* **TPC** - Capacitive touch panel. Uses FT813/BT815/BT817 Graphics controller

Note: for linux depending on your operating system normal users may not have access to USB devices. On those systems, run the examples with `sudo` 

Support Forums: http://www.lcdforums.com/forums/viewforum.php?f=45
