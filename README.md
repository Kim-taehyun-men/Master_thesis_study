# Master_thesis_study

Ubuntu   Version: 22.04
<br />
Fast-DDS Version: 2.6.9
<br />
Network (AP) environment: 2.4 GHz
<br />
When the number of nodes is 10, 12, and 14, respectively, the Publisher execution period is set to 1 s, 2 s, and 4 s.
<br />

## Setup and Execution Desktop(Publisher)
```bash
# OS buffer size setup
sysctl net.core.rmem_default net.core.rmem_default # check rmem_buffer_size
sudo sysctl -w net.core.rmem_default=268435456     # if not set buffer size, change buffer size
sudo sysctl -w net.core.rmem_max=268435456         # if not set buffer size, change buffer size

sudo ./install
IP No.: 1
wlan iface0 : Interface Name
vlan No.: 1
```

## Setup and Execution Laptop_1(Subscriber_1)
```bash
# OS buffer size setup
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

## Setup and Execution Laptop_2(Subscriber_2)
```bash
# OS buffer size setup
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

## Setup and Execution Laptop_3(Subscriber_3)
```bash
# OS buffer size setup
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
