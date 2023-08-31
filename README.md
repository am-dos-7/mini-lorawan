# Mini-LoRaWAN
A minimalistic LoRaWAN implementation for evaluating a Slotted-Aloha synchronization algorithm on LoRaWAN.

The current project is intended for implementing LoRaWAN at both end-device and Network Server side with the bare minimum of the functionalities required for runnning the algorithm in question.
For our tests, it was used on top of the Open-Source Gnu Radio implementation of a LoRa transceiver, provided at https://github.com/tapparelj/gr-lora_sdr, with some additional components to integrate some
 frame exchange rules between the Gateway using that implementation and our Network Server. Those additional components will be also be added very soon to this repo, after some clean-up. 
The file tree is as follows:
- The 'end-node' folder contains files related to the implementation at the end-node side. Currently, it only contains a main.cpp file that holds the entire implementation, coded on PlatformIO with the Arduino framework for an Adafuit Feather M0 target.
- The 'network-server' folder contains the file lorawan_ns.c which is the server-side implementation for a Linux machine, along with a Makefile for compiling a linking to the appropriate libraries.
- The 'output-plot' folder is for files related to visualizing the results of the experimentation. The main goal of all this being to evaluate a synchronization algorithm, the network server produces at runtime, a log file that contains information about frame arrival time and their position in the ongoing time-slot, in a csv-format. A typical example of such file is the 'slot_violation_new.txt' file, obtained after a run of more that 2h (8000 s). A header 'time,pos' has been added at the beginning to make it easily usable with libraries like 'pandas'. The 'plot_slot_violation.py' file is just a script that reads the specified file a output a 2D plot.
- This _readme.md_ file

## Dependencies
- The _end-node_ implementation uses the Radio Head library to communicate with the RF95 LoRa module on the Feather M0 board. For encryption related operation, it uses an AES implementation from [Arduino Crypto Library](https://rweather.github.io/arduinolibs/crypto.html). For derivating LoRaWAN's MIC, it also used the AES_CMAC library from https://github.com/IndustrialShields/arduino-AES_CMAC to calculate the CMAC.
- The _network-server_ for its part uses the cmac library from openssl, so openssl should be installed on the machine.

## Implementation details
For the sake of simplicity, no Over-The-Air-Activation process (OTAA) is handle in this implementation. We assumed an Activation By Personalization (ABP) and the shared-key is hardcoded at both the end-node and the server. Regarding, the __FOpts__ field, we assumed there is no MAC command used on the uplink, so this field is always empty, thus the __FOptsLen__ field is always equal to 0 on the Uplink. On the downlink however, __FOpts__ is used to piggyback a synchronization-related value on the ACK, hence __FOptsLen__ is equal to the size in byte of this value, 2 for instance.

Security mechanisms like payload encryption and Message Integrity Code are implemented, but there is no filter based on __FCnt__ (Frame count) field at the server side. This is because we used only one gateway in our experiment and more because we do not worry about threats like replay attacks, as it out of the scope of the current work.

The rest of the code is related to synchronization stuffs.
