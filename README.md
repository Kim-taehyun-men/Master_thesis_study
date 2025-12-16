# Wireless_SDN Network

ONOS Version: 2.3.0
OVS  Version: 2.9.0

Ehternet port 1 of an OVS should be connected its a host device.
ARP table of all hosts should be provided in advance (there is no need to be precise).
If device id of the OVS is "of:000...00x", IP of host connected the OVS should be set "10.0.0.x".

## Setup ONOS
```bash
cd onos_setting
sudo chmod +x install.sh
sudo chmod +x remove.sh

sudo ./install
IP No.: 1
wlan iface0 : Interface Name
vlan No.: 1
```

## Setup OVS
```bash
cd ovs_setting
sudo chmod +x install.sh
sudo chmod +x remove.sh

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

# SDN_forwarding Rule

```bash
cd cobaal-app
mvn clean install
onos-app localhost install target/cobaal-app-1.0-SNAPSHOT
```
