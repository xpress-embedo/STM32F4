# Adding LVGL to STM32 Project
First Generate CubeMX project, but make sure SPI5, LTDC and GPIO codes are not included and after that follow the following steps.  

* Add LVGL source files and `lvgl.h` file
* Add `lvgl_examples` source files
* Add the `tft` and `touch screen controller` drivers.
* Add `lvgl_confg.h` file
* Add include path settings in the IDE
* Turn off DMA2D and SDRAM configuration code (We will not use these peripehrals in this sample project)
* Call `lvgl_init()` function before you call any `lvgl` API's
* Register display driver and input deice driver (touch screen) with lvgl.
* Call `lv_tick_inc(x)` every x milliseconds in an interrupt to report the elapsed time to LVGL.
* Call `lv_timer_handler()` every few milliseconds to handle LVGL related tasks.