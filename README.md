/*******************************************************************************
* File Name          : readme.md
* Author             : timmy00274672 (timmy00274672@gmail.com)
* Date               : 06/05/2014
* Description        : This file describe the environment-related setting.
*******************************************************************************/

In the repo, I just copy the examples from *STM32 Note*.

## Environment

* IDE : IAR
* FWLib : V2.0.3 (STMicroelectronics)
* Board : CEPARK STM32F103RCT6

## Establish

1. Create a IAR project. (PWD in below is the IAR project directory.)
2. Include C/C++ libriary directories:

	```
	$WS_DIR$\..\simple_cmsis
	$WS_DIR$\..\CMSIS_V1P10\Core\CM3
	$WS_DIR$\..\FWLib\library\inc
	$WS_DIR$\inc
	```
3. Add Group `CMSIS_Include` (optional)

	```
	$WS_DIR$\..\CMSIS_V1P10\Core\CM3\core_cm3.h
	$WS_DIR$\..\CMSIS_V1P10\Core\CM3\stm32.h
	$WS_DIR$\..\CMSIS_V1P10\Core\CM3\system_stm32.h
	```

4. Add Group `CMSIS_Source`

	```
	$WS_DIR$\..\CMSIS_V1P10\Core\CM3\core_cm3.c
	$WS_DIR$\..\CMSIS_V1P10\Core\CM3\system_stm32.c
	```

5. Add Group `CMSIS_Startup` (Depend on IDE)

	```
	$WS_DIR$\..\CMSIS_V1P10\Core\CM3\startup\iar\startup_stm32f10x.s
	```

6. Add Group `FWLib_src` (Add the used files)

	```
	$WS_DIR$\..\FWLib\library\src\*.c
	```
7. Add Group `Include` (optional)

	```
	$WS_DIR$\inc\*.h
	```
8. Add Group `User` or `Source`

	All the files of the project, espacially the file containing `main` function.

9. More information can be found in `$WS_DIR$\..\FWLib\project\readme`.

## Additional Project Properties

### Show debug information

Select Full library, and just use printf and scanf via Terminal I/O (select in `View`)

### Configurate the linker

Use the linker configuration files: `$WS_DIR$\..\FWLib\project\EWARMv5stm32f10x_flash.icf`.

	This file is the IAR specific linking and loading file used to load in Flash and execute code and variables to target memories and regions. You can customize this file to your need.

### Configuration Debbuger

#### D-Linker

1. In `Downlolad` 

	- verify download
	- use flash loader (make sure that processor setting is correct or manually select proper one)

		For example, `$TOOLKIT_DIR$\config\flashloader\ST\FlashSTM32F10xxB.board`