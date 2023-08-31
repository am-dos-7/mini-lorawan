# Mini-LoRaWAN
A minimalistic LoRaWAN implementation for evaluating a Slotted-Aloha synchronization algorithm on LoRaWAN.

The current project is intended for implementing LoRaWAN at both end-device and Network Server side with the bare minimum of the functionalities required for runnning the algorithm in question.
For our tests, it was used on top of the Open-Source Gnu Radio implementation of a LoRa transceiver, provided at https://github.com/tapparelj/gr-lora_sdr, with some additional components to integrate some
 frame exchange rules between the Gateway using that implementation and our Network Server. Those additional components will be also be added very soon to this repo, after some clean-up. 
The file tree is as follows:
- The 'end-node' folder contains files related to the implementation at the end-node side. Currently, it only contains a main.cpp file that holds the entire implementation
- The 'network-server' folder contains the file lorawan_ns.c which is the server-side implementation, along with a Makefile for compiling a linking to the appropriate libraries.
- The 'output-plot' folder is for files related to visualizing the results of the experimentation. The main goal of all this being to evaluate a synchronization algorithm, the network server produces at runtime, a log file that contains information about frame arrival time and their position in the ongoing time-slot, in a csv-format. A typical example of such file is the 'slot_violation_new.txt' file, obtained after a run of more that 2h (8000 s). A header 'time,pos' has been added at the beginning to make it easily usable with libraries like 'pandas'. The 'plot_slot_violation.py' file is just a script that reads the specified file a output a 2D plot.

   
 

