# Nissan-Leaf-Bruteforce-Upgrade
Software for correcting instrumentation on a battery "bruteforce" upgraded LEAF. Example code integrated 30kWh cells using 24kWh original BMS.

## A word of caution
Battery upgrading a Nissan Leaf by opening it up and replacing the original LMNO2 cells with something else is always a bad idea. It requires great electrical skill, and takes a long time to perform. It also compromises the waterproof seal present, so you have to rely on your glue-caulking skills to get it waterproof again. If you use something else than Nissan cells you also risk having it catch on fire in the event of a crash, due to not using the OEM fasteners properly. The last and biggest problem is that it results in an overall unpleasant upgrade, due to the instrumentation issues. The code in this repository will alleviate some of the instrumentation issues, but it's not a ideal solution. This is the reason I made this way to upgrade obsolete, and only perform clean swaps. This is of no comfort to you if you already did this bruteforce upgrade, so here is the code that might make your life easier.

## I accept the risks, how to proceed?
The attached code should run on an Muxsan CAN MITM Bridge (3-port, rev. 2.5), available for purchase here. https://www.tindie.com/products/muxsan/can-mitm-bridge-3-port-rev-25/

Take the .c file from this repository, and apply it to the Muxsan example repository. Compile and upload to bridge.
Muxsan repository: https://bitbucket.org/emile_nijssen/open-source-can-bridge/src/master/

Attach the bridge on the EV-CAN running from the battery to the car. On a 2011-2013 ZE0 Leaf, an ideal spot is under the cupholders in the center console. On the 2013-2017 AZE0, the easiest spot to splice it in is behind the footwell LH side trim. (Add pictures?)

## How do I modify the code for my usecase?
There is a parameter called 'EXTRAGIDS'. This define can be experimented with to suit your upgrade. For a 30kWh pack, 65 extragids worked quite OK.

## What are the issues?
This code ONLY fixes the guess-o-meter estimate. That is the km/miles remaining value. The battery bars (12) are not properly translated, and will jump around a lot on the ZE0, but be a bit more stable on the AZE0. It also doesn't fix the %-SOC display on the AZE0.
Another issue is that when the voltage drops low, the code switches to capacity left estimation using voltage. So the GOM value will jump a bit depending on how hard you press on the gas pedal. This only happens on low SOC, but can be maybe be solved by filtering the value some more. Try experimenting with that :)

Good luck bruteforcers! 
