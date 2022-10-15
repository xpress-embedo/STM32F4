
## Graphics Application
A microcontroller based interactive application which may involve any of the below components to display on the display device.
* Colours
  * Color of the texts
  * Background Color/Layer Color
* Shapes
  * Box
  * Circle
  * Arrows
  * Lines
* Images
* Texts
* Graphical Effects (Scrolling, Sliding, Swipe, Press, Release)
* Widgets (Simple Button, Radio Button, Check Box, Videos )
* 3D Rendering
* Gaming

## Embedded Graphics System

![Embedded Graphics System](Documentation/EmbeddedGraphicsSystem.PNG)

### Important Part of the Embedded Graphics System
* A Microcontroller (Host)
  * Processor (Executes our code, updates the buffer)
  * RAM (FRAME Buffer)
  * Flash (Static Images, Fonts, Texts etc )
* Display Module
  * Glass (Where we see the graphics)
  * Driver Chip (Interpret the signals sent by the host, generates required electric signals and voltages to lit the pixel of the display panel)

**Frame Buffer: A memory area which holds the pixel values of the frame to be updated on the display**

### Some Other Important Part of the Embedded Graphics System
* Display Controller
  * Present on the Host Side
  * Generates display timing related signals
  * Transmit Pixel Data  
  In STM32 there is a peripheral called LTDC which stands for LCD TFT Display Controller, this is a bus master, this takes directly the pixel data from the frame buffer and it can send it to the display module over LTDC module.
* External Memories
  * External Flash (our code plus graphic components such as images text, this may not fit inside the internal flash)
  * External RAM
* Graphics Library (LVGL, TouchGFX, emWin, Embedded Wizard etc. )
* UI designer tool
  * To create an interactive UI application
* Touch Sensing
  * Touch Panel
  * Touch Screen Controller (Which senses the touch panel and inform the host)
* DMA
  * Helps to transfer frame buffer to display without the intervention of the CPU
  * Helps to transfer the graphics details from the flash to frame buffer without the intervention of the CPU.


The following diagram shows how TFT is connected to the STM32F429 development kit.  

![Embedded Graphics System](Documentation/EmbeddedGraphicsSTM32F4.PNG)  

There are around 22 pins that are used for display signals. The microcontroller sends the color information over these display pins.  
There is a SPI communication also, it is used for LCD driver chip programming, so first we have to configure the LCD driver chip using these lines and then only we can send the display information on above display lines.  
There are some control signals also for this LCD driver chip, which are controlled by the host, these are basically used to interpret the command and  data.  

## RGB Display and Pixel
A display which taskes red-green-blue signal components of a pixel to generate the colors to be displayed on the screen.  
RGB is a model in which Red-Green-Blue primary colors are added together in various ways to reproduce a broad array of colors.  
### Some Important Terminologies
* Pixel
  * Pixel of the image is the smallest information (color information) of the image stored in computer memory.  
  ![Roaring Tiger](Documentation/RoaringTiger_480X270.png)
  * Image information is as below.  
  ```
  Dimension is 480x270
  width is 480 pixels
  height is 270 pixels
  Bit Depth is 32 (this means each pixel is of 32 bits )
  ```
  * Similarly the Pixel of Display is also the smallest glowing (color producing) electronic element of the display module.  
  * **How much memory does one pixel consume in the computer memory?**  
    * This is also called as **Pixel Depth**, **Color Depth** or **Bit Depth**  
    * This depends on the pixel format and is measured in bpp (bits per pixel)  
* Pixel Density (PPI-Pixel Per Inch)
* Pixel Color Depth (Bit Depth)
* Pixel Format (Color Formats)
  * The following tables shows some pixel formats (color formats).  

    | Pixel Format  | 	Description                                     |
    | ------------- | ------------------------------------------------- |
    | ARGB8888	    | 32 bits, 8 bits per component                     |
    | L8_ARGB8888	  | 8 bits indexed format, ARGB8888 palette           |
    | RGB888	      | 24 bits, 8 bits per component                     |
    | L8_RGB888	    | 8 bits indexed format, RGB888 palette             |
    | RGB666	      | 24 bits, 6 bits per component                     |
    | RGB565	      | 16 bits, 5 bits red, 6 bits green, 5 bits blue    |
    | L8_RGB565     |	8 bits indexed format, RGB565 palette             |
    | ARGB2222      |	8 bits, 2 bits per component                      |
    | ABGR2222      |	8 bits, 2 bits per component                      |
    | RGBA2222      |	8 bits, 2 bits per component                      |
    | BGRA2222      |	8 bits, 2 bits per component                      |
    | GRAY4	        | 4 bits grayscale                                  |
    | GRAY2	        | 2 bits grayscale                                  |
    | BW	          | 1 bit grayscale                                   |
    | BW_RLE	      | 1 bit grayscale run-length encoded                |
  
  * **ARGB8888** if an image is represented in ARGB8888 pixel format, that means each of that image consumes 32 bits (32bpp) in memory, and each pixel has 4 components.
    * Alpha (Opacity) component of 8 bits
    * Red Color component of 8 bits
    * Green Color component of 8 bits
    * Blue Color component of 8 bits
  * L8 Formats
    * This represents the color of the image by color lookup table (CLUT).
    * The L8 format, 8 bit index value is used to look for the color value in the predefined lookup table.
    * This helps in saving memory, a comparison is given below.

    | Frame Buffer  | 	Pixel Format  | Total Memory Consumed           |
    | ------------- | --------------- | ------------------------------- |
    | 470x270       | RGB888          | 470 * 270 * 3 = 3805kb          |
    | 470x270       | L8_RGB888       | 470 * 270 * 1 + 256*3 = 127.3kb |
* Resolution



