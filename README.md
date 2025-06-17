# usv_udp_bridge

ROS 2 패키지 `usv_udp_bridge`는 **USV(Unmanned Surface Vehicle)** 혹은 무인 선박의 **위치 및 자세 데이터를 UDP를 통해 송수신**하는 기능을 제공합니다. 이 패키지는 ROS2 메시지로부터 데이터를 수신하고, 이를 외부 시스템으로 전송하거나, 반대로 외부 시스템으로부터 수신하여 내부 로직에서 사용할 수 있도록 설계되어 있습니다.

## 📦 구성 요소

- `UdpSendor`: ROS2 메시지를 수신하여 UDP로 전송하는 송신 노드
- `UdpReceiver`: 외부에서 UDP로 수신한 데이터를 `NavigationMsg` 구조체에 저장하는 수신 클래스

---

## 🧭 주요 기능

### 1. UDP 송신 (`UdpSendor`)

- 특정 ROS2 토픽(`/navigation_data`, `agent{id}` 형태 가능)을 구독하여 데이터를 수신
- 수신된 데이터를 `NavigationMsg` 구조체에 맞춰 직렬화
- 지정된 IP 주소와 포트로 UDP 패킷 전송

**송신 대상 설정 예시:**
```bash
ros2 run usv_udp_bridge udp_sender --ros-args -p receiverIP:=192.168.0.4
```
자동 USV ID 추출:
- /agent1/navigation_data 형태의 토픽에서 숫자를 추출하여 USV ID로 사용
- 일치하지 않거나 정보가 없다면 ID는 기본값 0으로 설정됨

```cpp
std::regex agent_regex("agent(\\d+)");
std::smatch matches;

if (std::regex_search(navigation_topic, matches, agent_regex)) {
  usv_id_ = std::stoul(matches[1].str());
}
```
UDP 전송 포맷 (NavigationMsg):
```cpp
struct NavigationMsg {
  uint32_t usv_id;
  float x, y, psi;
  float u, v, r, w;
};
```

### 2. UDP 수신 (UdpReceiver)
- 지정된 포트(RECV_PORT)에서 UDP 데이터를 수신
- NavigationMsg 구조체로 디코딩
- recvfrom() 호출 시 송신자의 IP 정보를 확인 가능

내부 동작:

- receive() 호출 시 내부적으로 recvfrom() 사용
- 데이터 수신 시 nav_msg 구조체가 업데이트됨
- 반환 값은 true일 경우 수신 성공, false는 수신 실패 또는 크기 불일치

다중 송신자 대응
여러 USV 송신기(예: 192.168.0.1, 0.2, 0.3 등)가 하나의 수신기(예: 192.168.0.4)에 데이터를 보내는 경우:

- 모든 송신자로부터 수신 가능
- 각 송신자의 IP는 recvfrom()를 통해 확인 가능
- 수신 순서는 네트워크/OS 큐에 따라 비결정적이며, 순서 보장은 없음
- 동시에 보내는 경우, OS의 수신 큐에서 처리되며 멀티스레드로 처리하거나 수신 큐를 이용해 분기 가능

## 빌드 및 실행
### 의존성
- ROS 2 (Foxy / Humble / Iron 등)
- 사용자 정의 메시지: NavigationType (예: geometry_msgs 또는 custom msg)
- data_struct 헤더 또는 메시지 정의 필요

```bash
colcon build --packages-select usv_udp_bridge
source install/setup.bash
```

### 예시 노드 실행
```bash
ros2 run usv_udp_bridge udp_sender \
  --ros-args -p receiverIP:=192.168.0.4 \
  -p navigation_topic:=/agent1/navigation_data
```

### 수신 코드 사용 예시
```cpp
#include "usv_udp_bridge/receiver.hpp"

UdpReceiver receiver;
NavigationMsg msg;

if (receiver.receive(msg)) {
  std::cout << "Received from USV ID: " << msg.usv_id << " at position (" 
            << msg.x << ", " << msg.y << ")" << std::endl;
}
```

본 코드는 자율운항선박(USV) 시스템의 ROS 2 기반 통신 브릿지를 위해 개발되었습니다.

문의 및 기여는 Pull Request 또는 Issues를 통해 환영합니다.


