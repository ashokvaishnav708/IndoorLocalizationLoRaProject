# IndoorLocalizationLoRaProject

## Folder Description

### SX1280

This folder contains the drivers for Semtech's SX1280. Simply copy the content inside this folder to the **zephyr-framework** directory.

### Testbed Visulalization

This folder constains the python script to visualize the live Indoor Localization System. Open the python script and change teh **COM** port to the appropriate port connected to the Mobile device.


### Indoor_Localization_Anchor_v2.0

This directory contains zephyr program for Anchor devices using LoRa. It needs to be uploaded to all the anchor nodes available in the system.

### Indoor_Localization_Master_v2.0

This directory contains the zephyr program for Master (Central) device for Indoor Localization System. This needs to be updated whenever we introduce a new Acnhor device in the system.
Node: Master Node contains all the information (Device IDs and Corrdinates) about Anchors and responsible for sharing the Anchor's information to Mobile device.

### Indoor_Localization_Mobile_v3.0

This directory contains teh zephyr code for Mobile device. This program contains all teh logic and algorithm for Indoor Localization System's location estimation. It can be uploaded to any device having LoRa module attached to it and quickly used as mobile node.

### Hwid_Collection_nrf52840dk

This directory conains the zephyr program for nRF52840DK board to collect the board's hardware ID which is later used as device ID in Indoor Localization system. 
Note: This is only necessary if you introduce new Anchor to the System.

## File Description

### Master_Thesis_Indoor_Localization_LoRa.pdf
This file contains the complete documentaion report about this masters thesis work. It can be used as a reference book for complete Indoor Localization System. 