# Master_thesis_study

Ubuntu   Version: 22.04
<br />
Fast-DDS Version: 2.6.9
<br />
Network (AP) environment: 2.4 GHz
<br />
When the number of nodes is 10, 12, and 14, respectively, the Publisher execution period is set to 1 s, 2 s, and 8 s.
<br />

## Setup and Execution Desktop(Publisher)
```bash
# OS buffer size setup
sysctl net.core.rmem_default net.core.rmem_default # check rmem_buffer_size
sudo sysctl -w net.core.rmem_default=268435456     # if not set buffer size, change buffer size
sudo sysctl -w net.core.rmem_max=268435456         # if not set buffer size, change buffer size

# Execution
orchestrate_rounds.sh # 전체 실행 파일, 총 진행할 라운드, 라운드간 지속시간(max 시간), 다음 라운드 간의 시간 설정 // Discovery가 끝났다고 인지하면 자동으로 ros2 노드들 종료시킴.
run_all.sh # 노드 몇개 실행할 꺼인지 설정하는 파일
watch_2x28.sh # 노드 시작 및 종료 시간 기록 파일
mac_aqm_to_csv.sh # Software mac Queue 측정 파일(단위 0.1s)
convery_ts.py # watch_2x28.sh 에서 시간 변환을 위해 필요한 파일

#출력물
fastdds_discvoery_hb_N #hb 잘 끝났나 확인가능판 파일
fastdds_udp_tx_hb_data_N # DDS 단에서 실제로 전송한 Discovery 데이터 기록 파일
mac_aqm_tid0_ac2_wlx94a67e6e4790@70:5d:cc:38:d7:42_2025_N # Software mac Queue의 전송, 드랍, 처리한 패킷값들을 0.1초기 기록한 파일
Time_N : Discovery 시작과 끝나는 시간을 기록한 파일

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

ros2 run my_topic_example subscriber

ros2 run my_topic_example_14 subscriber_14

## Result_Data
