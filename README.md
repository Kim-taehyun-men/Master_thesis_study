# Master_thesis_study
모든 프로그램의 유동적으로 변경 가능한 변수가 있으니, 잘 확인하고 진행하면 좋을껏.

## Master_thesis_study_1
의의: Publisher에서 각 지표들이 얼만큼 변경되는지 확인하는 실험(Normal Vs SPREAD Algorithms)
1. Subscriber_1, _2, _3들을 각 노드(10, 12, 14)만큼 실행 시킨다.
2. Publisher들을 명령어를 실행시켜 데이터를 확인한다.
3. colab에 들어가 데이터를 넣고, 결과값을 확인한다.
   
## Master_thesis_study_2
의의: Discovery에서 1대1일 때 전송률에 따른 재전송 매커니즘 성공하는 라운드 확인하기 위한 실험(유선에서 진행되고 있음)
1. Publisher와 Subscriber_4를 유선으로 연결하고 eth(이더포트)의 ip와 손실률을 설정한다.(ping 192.168.0.1 or 192.168.0.2로 연결 상태 확인하기)
2. Subscriber_4를 먼저 실행시키고, Publisher에서 자동화 프로그램을 설정하도 실행시킨다. 확인한다.
   
## Master_thesis_study_3
의의: Publisher에서 각 지표들이 얼만큼 변경되는지 확인하는 실험(Normal Vs SPREAD Algorithms)
1. Subscriber_1, _2, _3들을 각 노드(10, 12, 14)만큼 실행 시킨다.
2. Publisher들을 명령어를 실행시켜 데이터를 확인한다.
3. colab에 들어가 데이터를 넣고, 결과값을 확인한다.
   
# Master_thesis_study_1

Ubuntu   Version: 22.04
<br />
Fast-DDS Version: 2.6.9
<br />
Network (AP) environment: 2.4 GHz
<br />
When the number of nodes is 10, 12, and 14, respectively, the Publisher execution period is set to 1 s, 4 s, and 8 s.
<br />



## Setup and Execution Desktop(Publisher)
```bash

# Execution
cd /tmp
./orchestrate_round.sh                                   # Execution study (실행전 sudo 권한을 받아둬야함, sudo ifconfig 명령어 후 진행하면 됨)
python3 summarize_drops_and_time.py                      # Data processing

# Execution file explanation
orchestrate_rounds.sh                                    # 전체 실행 파일, 총 진행할 라운드, 라운드간 지속시간(max 시간), 다음 라운드 간의 시간 설정 // Discovery가 끝났다고 인지하면 자동으로 ros2 노드들 종료시킴.
run_all.sh                                               # 몇개의 노드를 실행할 꺼인지 설정하는 파일
watch_2x28.sh                                            # 노드 시작 및 종료 시간 기록 파일
mac_aqm_to_csv.sh                                        # Software mac Queue 측정 파일(단위 0.1s)
convery_ts.py                                            # watch_2x28.sh 에서 시간 변환을 위해 필요한 파일

# Result file explanation
fastdds_discvoery_hb_N                                    # hb 잘 끝났나 확인가능판 파일
fastdds_udp_tx_hb_data_N                                  # DDS 단에서 실제로 전송한 Discovery 데이터 기록 파일
mac_aqm_tid0_ac2_wlx94a67e6e4790@70:5d:cc:38:d7:42_2025_N # Software mac Queue의 전송, 드랍, 처리한 패킷값들을 0.1초기 기록한 파일
Time_N                                                    # Discovery 시작과 끝나는 시간을 기록한 파일
```

## Setup and Execution Laptop_1(Subscriber_1)
```bash

# Execution (Input command anther terminal, 10/12/14)
ros2 run my_topic_example subscriber
ros2 run my_topic_example_1 subscriber_1
ros2 run my_topic_example_2 subscriber_2
~
ros2 run my_topic_example_9 subscriber_9
~
ros2 run my_topic_example_11 subscriber_11
~
ros2 run my_topic_example_13 subscriber_13

# If you want to build for DDS, use this command
cd ros2_humble/src/eProsima/
colcon build
source insatll/setup.bash
```

## Setup and Execution Laptop_2(Subscriber_2)
```bash

# Execution (Input command anther terminal, 10/12/14)
ros2 run my_topic_example subscriber
ros2 run my_topic_example_1 subscriber_1
ros2 run my_topic_example_2 subscriber_2
~
ros2 run my_topic_example_9 subscriber_9
~
ros2 run my_topic_example_11 subscriber_11
~
ros2 run my_topic_example_13 subscriber_13

# If you want to build for DDS, use this command
cd ros2_humble/src/eProsima/
colcon build
source insatll/setup.bash
```

## Setup and Execution Laptop_3(Subscriber_3)
```bash

# Execution (Input command anther terminal, 10/12/14)
ros2 run my_topic_example subscriber
ros2 run my_topic_example_1 subscriber_1
ros2 run my_topic_example_2 subscriber_2
~
ros2 run my_topic_example_9 subscriber_9
~
ros2 run my_topic_example_11 subscriber_11
~
ros2 run my_topic_example_13 subscriber_13

# If you want to build for DDS, use this command
cd ros2_ws/src/Fast-DDS/
colcon build
source insatll/setup.bash
```

## Result
<img width="989" height="440" alt="Image" src="https://github.com/user-attachments/assets/67cacb2d-7e94-4551-96fd-365ebec12c80" />
<img width="989" height="439" alt="Image" src="https://github.com/user-attachments/assets/053f78aa-6518-49a0-8106-0662b4268965" />
<img width="989" height="440" alt="Image" src="https://github.com/user-attachments/assets/828fdc6d-04ee-41be-b2d2-1268b503ff39" />

# Master_thesis_study_2

## Setup and Execution Desktop(Publisher)
```bash
# eth ip and loss rate setup
sudo ip addr add 192.168.10.1/24 dev eth1(interface name)

sudo tc qdisc add dev eth1 root netem loss 5     (5, 10, 20, 30)
sudo tc qdisc change dev eth1 root netem loss 5  (5, 10, 20, 30)

# Execution
./ros2_hb_cycle.sh

# Data processing
python3 -m venv .venv
source .venv/bin/activate
pip install "numpy<2" pandas matplotlib

python3 end_round_cdf.py --prefix "./fastdds_discovery_hb_" --start 1 --end 1000 --ext ".csv" --max-threshold 15 --out-dir "./"     # 1, 1000 의 변수는 범위로 ros2_hb_cycls.sh에서 설정한 반복 횟수만큼 변동하면됨
```

## Setup and Execution Laptop_4(Subscriber_4)
```bash
# eth ip and loss rate setup
sudo ip addr add 192.168.10.2/24 dev eth1(interface name)

sudo tc qdisc add dev eth1 root netem loss 5     (5, 10, 20, 30)
sudo tc qdisc change dev eth1 root netem loss 5  (5, 10, 20, 30)

# Execution
ros2 run my_topic_example subscriber

```
## Result
<img width="989" height="589" alt="Image" src="https://github.com/user-attachments/assets/15463128-d6c9-42d7-b910-e9d504e0ff78" />


# Master_thesis_study_3

## Setup and Execution Desktop(Publisher)
```bash
# Execution
# Data collection
cd /tmp
# 파일 실행 시 이름 설정 잘해야함.(다음 실행을 위해)
./SH_all.sh                                   # Execution study (실행전 sudo 권한을 받아둬야함, sudo ifconfig 명령어 후 진행하면 됨)

# Input file name 설정 잘하기, out put file name 설정 잘하기. (각 회차들 데이터들의 평균값 가지고 오는 부분)
python3 show_elbow_illine.py   --glob "mac_aqm_tid0_ac2_wlx94a67e6e4790@70:5d:cc:38:d7:42_16_*_H4_P0(NP).csv"   --neighbors 1   --out-csv "service_and_backlog_window_avg_16.csv"

# 각 부분들의 데이터 중앙값 출력
python3 summarize.py

```
## Subscriber들의 노드들이 총 13개, 14개, 15개, 16개일 때의 전송량을 측정한거임.
EX) Subscriber_1, Subscriber_2, Subscriber_3들에거 각각 4,4,5 개의 노드 실행. (총 13개)
## Setup and Execution Laptop_1(Subscriber_1)
```bash

# Execution (Input command anther terminal, 10/12/14)
ros2 run my_topic_example subscriber
ros2 run my_topic_example_1 subscriber_1
ros2 run my_topic_example_2 subscriber_2
~
ros2 run my_topic_example_9 subscriber_9
~
ros2 run my_topic_example_11 subscriber_11
~
ros2 run my_topic_example_13 subscriber_13

# If you want to build for DDS, use this command
cd ros2_humble/src/eProsima/
colcon build
source insatll/setup.bash
```

## Setup and Execution Laptop_2(Subscriber_2)
```bash

# Execution (Input command anther terminal, 10/12/14)
ros2 run my_topic_example subscriber
ros2 run my_topic_example_1 subscriber_1
ros2 run my_topic_example_2 subscriber_2
~
ros2 run my_topic_example_9 subscriber_9
~
ros2 run my_topic_example_11 subscriber_11
~
ros2 run my_topic_example_13 subscriber_13

# If you want to build for DDS, use this command
cd ros2_humble/src/eProsima/
colcon build
source insatll/setup.bash
```

## Setup and Execution Laptop_3(Subscriber_3)
```bash

# Execution (Input command anther terminal, 10/12/14)
ros2 run my_topic_example subscriber
ros2 run my_topic_example_1 subscriber_1
ros2 run my_topic_example_2 subscriber_2
~
ros2 run my_topic_example_9 subscriber_9
~
ros2 run my_topic_example_11 subscriber_11
~
ros2 run my_topic_example_13 subscriber_13

# If you want to build for DDS, use this command
cd ros2_ws/src/Fast-DDS/
colcon build
source insatll/setup.bash
```
## Result
<img width="1389" height="409" alt="Image" src="https://github.com/user-attachments/assets/571fa617-5fbc-4b57-a0ad-26021c073bf7" />
