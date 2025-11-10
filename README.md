# agv_plug_in
This plugin integrates other manufacturers' AMRs or AGVs into the MSI FMS for unified traffic management. Other manufacturers' AGVs must provide their current spatial coordinates based on the MSI AMR's coordinate system. This plugin can determine the relative spatial coordinates between the MSI AMR and other brands' AGVs.

![overview](document/overview.jpg)

## agv-plug-in.py
This program is used to bridge with FMS and other AMR/AGV systems.

## sim_agv.py
A simple example program is used to simulate how their AMR/AGV sends out its spatial information and obtains whether it has right-of-way.