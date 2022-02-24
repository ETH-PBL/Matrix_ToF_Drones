<!--
*** Template source: https://github.com/othneildrew/Best-README-Template/blob/master/README.md
-->

<!-- PROJECT SHIELDS -->
<!--
*** I'm using markdown "reference style" links for readability.
*** Reference links are enclosed in brackets [ ] instead of parentheses ( ).
*** See the bottom of this document for the declaration of the reference variables
*** for contributors-url, forks-url, etc. This is an optional, concise syntax you may use.
*** https://www.markdownguide.org/basic-syntax/#reference-style-links
-->
[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![License][license-shield]][license-url]



<!-- PROJECT LOGO -->
<br />
<p align="center">
  <a href="https://github.com/ETH-PBL/H-Watch">
    <img src="pics/drone.png" alt="Logo" width="620" height="325">
  </a>

  <h3 align="center">Matrix_ToF_Drones</h3>

  <p align="center">
    Indoor Navigation System based on Multi-Pixel Time-of-Flight Imaging for Nano-Drone Applications
    <br />
    <a href="https://github.com/ETH-PBL/Matrix_ToF_Drones"><strong>Explore the docs »</strong></a>
    <br />
    <br />
    <a href="https://www.youtube.com/watch?v=-WDpvO4t7PM">View Demo</a>
    ·
    <a href="https://github.com/ETH-PBL/Matrix_ToF_Drones/issues">Report Bug</a>
    ·
    <a href="https://github.com/ETH-PBL/Matrix_ToF_Drones/issues">Request Feature</a>
  </p>
</p>


<!-- TABLE OF CONTENTS -->
## Table of Contents

* [About the Project](#about-the-project)
* [Hardware](#hardware)
  * [Components](#hardware)
  * [PCB](#pcb) 
* [Getting Started](#getting-started)
  * [Software](#software)
  * [Installation](#installation)
  


<!-- ABOUT THE PROJECT -->
## About The Project

Unmanned aerial vehicles (UAVs) have recently attracted the industry's attention due to their numerous civilian and potential commercial applications.
A promising UAV sub-class includes nano and micro UAVs, characterized by centimeter size, few grams of payload and extremely limited on-board computational resources. 
Those features pose major challenges to enable autonomous navigation or even more basic relevant sub-tasks, such as reliable obstacle avoidance. This project exploits a multi-zone Time of Flight (ToF) sensor to enable autonomous navigation and obstacle avoidance with a low lower computational load than most common visual-based solutions. 

## Hardware

The matrix ToF sensor, and thus this project aims to characterize and in-field evaluate the sensors <a href="www.st.com/en/imaging-and-photonics-solutions/vl53l5cx.html">VL53L5CX</a> from STMicroelectronics using a nano-drone platform. This project exploits a commercial nano-UAV platform, the Crazyflie 2.1 from bitcraze, together with a custom PCB designed to support two 8x8 ToF sensors. The following sections describe in detail the overall framework and the hardware setup.

### Components
The <a href="www.st.com/en/imaging-and-photonics-solutions/vl53l5cx.html">VL53L5CX</a> is a ToF multizone ranging sensor produced by STMicroelectronics. 
It integrates a single-photon avalanche diode (SPAD) array, physical infrared filters, and diffractive optical elements (DOE) to achieve a millimeter accuracy in various ambient lighting conditions with a wide range of cover glass materials. The working range spans between 2 cm and 4 meters, but above 2 m the overall ranging precision degrades to 11% of the absolute distance. The most important feature of the VL53L5CX is the multizone capability, which can be configured as an 8x8 or 4x4 matrix. 
Listed below the list of key components for our integrated deck targetted for the Crazyflie 2.1 platform: 

* [VL53L5CX][VL53L5CX_url],       ToF multizone ranging sensor
* [TPS62233][tps62233_url],       3-MHz Ultra Small Step-Down Converter
* [TCA6408A][tca6408a_url],       Low-Voltage 8-Bit I2C and SMBus I/O Expander

### PCB 
The H-Watchs printed circuit board (PCB) is built of 4 Layers with a total board thichness of only 0.83mm. Further informations about the PCB can be found here:

File                                  | Content
--------------------------------------|--------
[Deck_schematics.pdf]                 | Schematics of the deck that supports 2 VL53L5CX.  
[Deck_PCB_3D.pdf]                     | Layout 3D view with details to components and nets.
[Deck_BOM.xlsx]			      | Bill of material for the Deck PCB.
[Sensor_Board_schematics.pdf]         | Schematic of the lateral PCB that hold one VL53L5CX.
[Sensor_Board_PCB_3D.pdf]             | Layout 3D view with details to components and nets.
[Sensor_Board_BOM.xlsx]               | Bill of material for the sensor board.
[Final_Assembly_3D.pdf]               | Assembly of one Deck PCB together with two sensor board PCBs, front-facing and back-facing directions. 


The preview of the assembly, with details of logical connections can be found here:

<a href="https://github.com/ETH-PBL/H-Watch">
    <img src="pics/cad.png" alt="Logo" width="420" height="325">
</a>

<!-- GETTING STARTED -->

## Getting Started

#### Material

* One fully mounted [Deck_schematics.pdf] and two [Sensor_Board_schematics.pdf] assembled as shown here: [Final_Assembly_3D.pdf]
* The [Crazyflie 2.1][crazyflie_url]
* The [Flow Deck v2][flowdeck_url]. The VL53L1x ToF sensor measures the distance to the ground with high precision and the PMW3901 optical flow sensor measures movements of the ground.

#### Software

* [STM32CubeIDE][stmcubeIDE_url]
* Firmware Package FW_WB V1.8.0
* [Altium][altium_url]

### Installation



## Acknowledges

If you use **Matrix ToF Drone** in an academic or industrial context, please cite the following publications:

~~~~
@INPROCEEDINGS{20.500.11850/476189,
	copyright = {Creative Commons Attribution 4.0 International},
	year = {2021-05-22},
	author = {Polonelli, Tommaso and Schulthess, Lukas and Mayer, Philipp and Magno, Michele and Benini, Luca},
	size = {6 p. submitted version},
	keywords = {wearable device; Covid-19; smart sensing; low power design; Tiny Machine Learning; wireless sensor networks},
	language = {en},
	DOI = {10.3929/ethz-b-000476189},
	title = {H-Watch: An Open, Connected Platform for AI-Enhanced COVID19 Infection Symptoms Monitoring and Contact Tracing.},
	Note = {IEEE International Symposium on Circuits and Systems (ISCAS 2021); Conference Location: online; Conference Date: May 22–28}
}
~~~~


<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->

<!--Subsection Hardware-->
[VL53L5CX_url]:     www.st.com/en/imaging-and-photonics-solutions/vl53l5cx.html
[tps62233_url]:     https://www.ti.com/lit/ds/symlink/tps62230.pdf?ts=1642068933103&ref_url=https%253A%252F%252Fwww.ti.com%252Fproduct%252FTPS62230
[tca6408a_url]:     https://www.ti.com/lit/ds/symlink/tca6408a.pdf?ts=1642063709908&ref_url=https%253A%252F%252Fwww.ti.com%252Fproduct%252FTCA6408A
[crazyflie_url]:    https://www.bitcraze.io/products/crazyflie-2-1/
[flowdeck_url]:     https://store.bitcraze.io/collections/decks/products/flow-deck-v2

<!--Subsection PCB-->

[Deck_schematics.pdf]:                    /Hardware/TofDeck/TofDeck.pdf
[Deck_PCB_3D.pdf]:                        /Hardware/TofDeck/Deck_3D.pdf
[Deck_BOM.xlsx]:                          /Hardware/TofDeck/BOM_TofDeck.xlsx
[Sensor_Board_schematics.pdf]:            /Hardware/SensorBoard/SensorBoard.pdf
[Sensor_Board_PCB_3D.pdf]:                /Hardware/SensorBoard/SensorPCB_3D.pdf
[Sensor_Board_BOM.xlsx]:                  /Hardware/SensorBoard/BOM_SensorBoard.xlsx
[Final_Assembly_3D.pdf]:                  /Hardware/MultiBoard_Project/Outputs/Assembly1.pdf




[stmcubeIDE_url]:	https://www.st.com/en/development-tools/stm32cubeide.html
[altium_url]:	        https://www.altium.com/


[contributors-shield]: https://img.shields.io/github/contributors/ETH-PBL/Matrix_ToF_Drones.svg?style=flat-square
[contributors-url]: https://github.com/ETH-PBL/Matrix_ToF_Drones/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/ETH-PBL/Matrix_ToF_Drones.svg?style=flat-square
[forks-url]: https://github.com/ETH-PBL/Matrix_ToF_Drones/network/members
[stars-shield]: https://img.shields.io/github/stars/ETH-PBL/Matrix_ToF_Drones.svg?style=flat-square
[stars-url]: https://github.com/ETH-PBL/Matrix_ToF_Drones/stargazers
[issues-shield]: https://img.shields.io/github/issues/ETH-PBL/Matrix_ToF_Drones.svg?style=flat-square
[issues-url]: https://github.com/ETH-PBL/Matrix_ToF_Drones/issues
[license-shield]: https://img.shields.io/github/license/ETH-PBL/Matrix_ToF_Drones.svg?style=flat-square
[license-url]: https://github.com/ETH-PBL/Matrix_ToF_Drones/blob/master/LICENSE
[product-screenshot]: pics/drone.png
