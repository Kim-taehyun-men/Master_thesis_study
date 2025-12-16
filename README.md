# Master_thesis_study

Ubuntu   Version: 22.04
Fast-DDS Version: 2.6.9

Ehternet port 1 of an OVS should be connected its a host device.
ARP table of all hosts should be provided in advance (there is no need to be precise).
If device id of the OVS is "of:000...00x", IP of host connected the OVS should be set "10.0.0.x".

## Setup Desktop(Publisher)
```bash
# OS buffer size check and change
sysctl net.core.rmem_default net.core.rmem_default # check rmem_buffer_size
sudo sysctl -w net.core.rmem_default=268435456     # if not set buffer size, change buffer size
sudo sysctl -w net.core.rmem_max=268435456         # if not set buffer size, change buffer size

sudo ./install
IP No.: 1
wlan iface0 : Interface Name
vlan No.: 1
```

## Setup Laptop_1(Subscriber_1)
```bash
# OS buffer size check and change
sysctl net.core.rmem_default net.core.rmem_default # check rmem_buffer_size
sudo sysctl -w net.core.rmem_default=268435456     # if not set buffer size, change buffer size
sudo sysctl -w net.core.rmem_max=268435456         # if not set buffer size, change buffer size

sudo ./install
IP No.: X(Number)
wlan iface0 : Interface Name_1 (wlan)
wlan iface1 : Interface Name_2 (vlan_1)
wlan iface2 : Interface Name_3 (vlan_2)
ethernet iface0: veth0
ethernet iface1: veth1
vlan No. (1): 1
vlan No. (2): 2
```

## Setup Laptop_2(Subscriber_2)
```bash
# OS buffer size check and change
sysctl net.core.rmem_default net.core.rmem_default # check rmem_buffer_size
sudo sysctl -w net.core.rmem_default=268435456     # if not set buffer size, change buffer size
sudo sysctl -w net.core.rmem_max=268435456         # if not set buffer size, change buffer size

sudo ./install
IP No.: X(Number)
wlan iface0 : Interface Name_1 (wlan)
wlan iface1 : Interface Name_2 (vlan_1)
wlan iface2 : Interface Name_3 (vlan_2)
ethernet iface0: veth0
ethernet iface1: veth1
vlan No. (1): 1
vlan No. (2): 2
```

## Setup Laptop_3(Subscriber_3)
```bash
# OS buffer size check and change
sysctl net.core.rmem_default net.core.rmem_default # check rmem_buffer_size
sudo sysctl -w net.core.rmem_default=268435456     # if not set buffer size, change buffer size
sudo sysctl -w net.core.rmem_max=268435456         # if not set buffer size, change buffer size

sudo ./install
IP No.: X(Number)
wlan iface0 : Interface Name_1 (wlan)
wlan iface1 : Interface Name_2 (vlan_1)
wlan iface2 : Interface Name_3 (vlan_2)
ethernet iface0: veth0
ethernet iface1: veth1
vlan No. (1): 1
vlan No. (2): 2
```
