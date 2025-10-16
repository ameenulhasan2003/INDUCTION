STM32F103C8T6_InductionHeater_FullIOC - Full CubeMX-style project (informational .ioc)
-------------------------------------------------------------
This package adds fuller MX_* initialization files so STM32CubeIDE can import and generate code.

Files:
 - STM32F103C8T6_InductionHeater_FullIOC.ioc   (informational; open in STM32CubeIDE then Generate Code to re-create project)
 - Core/Inc/main.h
 - Core/Src/main.c
 - Core/Src/gpio.c
 - Core/Src/adc.c
 - Core/Src/tim.c
 - Core/Src/system_stm32f1xx.c

Notes:
 - After opening this folder in STM32CubeIDE, open the .ioc file and click "Project > Generate Code" to let CubeIDE recreate any missing startup/linker files and ensure peripheral settings match.
 - Test without coil attached; use gate driver, proper deadtime, and isolate mains.
 - If you want, I can also create a full CubeMX .ioc that includes clock wizard settings and NVIC priorities—tell me and I'll produce a more "official" .ioc that CubeMX can edit directly.
