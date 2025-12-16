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
```

## Setup and Execution Laptop_1(Subscriber_1)
```bash
# OS buffer size setup
sysctl net.core.rmem_default net.core.rmem_default # check rmem_buffer_size
sudo sysctl -w net.core.rmem_default=268435456     # if not set buffer size, change buffer size
sudo sysctl -w net.core.rmem_max=268435456         # if not set buffer size, change buffer size

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

# IF you want to build using this command
cd ros2_humble/src/eProsima/
colcon build
source insatll/setup.bash
```

## Setup and Execution Laptop_2(Subscriber_2)
```bash
# OS buffer size setup
sysctl net.core.rmem_default net.core.rmem_default # check rmem_buffer_size
sudo sysctl -w net.core.rmem_default=268435456     # if not set buffer size, change buffer size
sudo sysctl -w net.core.rmem_max=268435456         # if not set buffer size, change buffer size

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

# IF you want to build using this command
cd ros2_humble/src/eProsima/
colcon build
source insatll/setup.bash
```

## Setup and Execution Laptop_3(Subscriber_3)
```bash
# OS buffer size setup
sysctl net.core.rmem_default net.core.rmem_default # check rmem_buffer_size
sudo sysctl -w net.core.rmem_default=268435456     # if not set buffer size, change buffer size
sudo sysctl -w net.core.rmem_max=268435456         # if not set buffer size, change buffer size

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

# IF you want to build using this command
cd ros2_ws/src/Fast-DDS/
colcon build
source insatll/setup.bash
```

## Result

<img width="989" height="440" alt="Image" src="https://github.com/user-attachments/assets/67cacb2d-7e94-4551-96fd-365ebec12c80" />
<img width="989" height="439" alt="Image" src="https://github.com/user-attachments/assets/053f78aa-6518-49a0-8106-0662b4268965" />
<img width="989" height="440" alt="Image" src="https://github.com/user-attachments/assets/828fdc6d-04ee-41be-b2d2-1268b503ff39" />
