# 기여 가이드 (Contributing Guide)

Arduino-ROS Labs 프로젝트에 기여해 주셔서 감사합니다! 🎉

## 기여 방법

### 버그 리포트
1. GitHub Issues에서 기존 이슈 검색
2. 새로운 이슈 생성 시 다음 정보 포함:
   - 사용 환경 (Arduino 모델, ROS 버전, OS)
   - 문제 재현 방법
   - 예상 동작 vs 실제 동작
   - 오류 메시지 (있는 경우)

### 기능 제안
1. GitHub Issues에 "Feature Request" 라벨로 이슈 생성
2. 제안하는 기능의 목적과 사용 사례 설명
3. 가능하면 구현 아이디어 제시

### 코드 기여

#### 1. Fork & Clone
```bash
# Fork 후
git clone https://github.com/YOUR_USERNAME/arduino-ros-labs.git
cd arduino-ros-labs
```

#### 2. Branch 생성
```bash
git checkout -b feature/your-feature-name
# 또는
git checkout -b fix/bug-description
```

#### 3. 변경 사항 작성
- 기존 코드 스타일 유지
- 주석을 한글과 영어로 병기 (가능한 경우)
- 각 Lab의 README.md 업데이트

#### 4. 테스트
- Arduino 코드: 실제 하드웨어에서 테스트
- ROS 코드: roscore와 함께 동작 확인
- 문서: 오타 및 링크 확인

#### 5. Commit
```bash
git add .
git commit -m "Add: 새로운 기능 설명"
# 또는
git commit -m "Fix: 버그 수정 설명"
```

Commit 메시지 규칙:
- `Add:` 새로운 기능
- `Fix:` 버그 수정
- `Update:` 기존 기능 개선
- `Docs:` 문서 수정
- `Refactor:` 코드 리팩토링

#### 6. Pull Request
```bash
git push origin feature/your-feature-name
```

GitHub에서 Pull Request 생성 후:
- 변경 사항 명확히 설명
- 관련 이슈 번호 연결 (#이슈번호)
- 테스트 결과 첨부

## 코딩 스타일

### Arduino 코드
```cpp
// 상수는 대문자와 언더스코어
const int TRIG_PIN = 9;
const int ECHO_PIN = 10;

// 함수는 카멜 케이스
void readSensor() {
  // ...
}

// 변수는 스네이크 케이스 또는 카멜 케이스
int sensor_value;
float currentSpeed;

// 주석은 한글로 설명
// 트리거 펄스 생성
digitalWrite(TRIG_PIN, HIGH);
```

### 문서
- 한글 문서 우선 (영어 번역 환영)
- 마크다운 포맷 준수
- 코드 블록에 언어 지정 (```cpp, ```bash)
- 이미지는 docs/images/ 폴더에 저장

## 프로젝트 구조

```
arduino-ros-labs/
├── README.md                    # 메인 README
├── LICENSE                      # 라이선스
├── docs/                        # 문서 폴더
│   ├── setup/                   # 설치 가이드
│   ├── circuits/                # 회로 다이어그램
│   ├── PARTS_LIST.md            # 부품 목록
│   ├── TROUBLESHOOTING.md       # 문제 해결
│   └── QUICK_REFERENCE.md       # 빠른 참조
├── lab1_basic_sensors/          # Lab 1
│   ├── README.md
│   └── *.ino
├── lab2_motor_control/          # Lab 2
├── ...
├── lab7_autonomous_navigation/  # Lab 7
└── ros_launch/                  # ROS launch 파일
```

## 새로운 Lab 추가

1. `labX_topic_name/` 디렉토리 생성
2. README.md 작성 (다른 Lab 참고)
3. Arduino 코드 (.ino) 작성
4. 회로 연결 정보 추가
5. 메인 README.md에 Lab 목록 추가

## 문서 번역

영어 번역을 도와주실 수 있습니다:
1. 한글 문서를 기반으로 영어 번역
2. 파일명에 `.en.md` 추가 (예: `README.en.md`)
3. Pull Request 제출

## 이슈 라벨

- `bug`: 버그 리포트
- `enhancement`: 기능 개선
- `documentation`: 문서 관련
- `question`: 질문
- `help-wanted`: 도움 필요
- `good-first-issue`: 초보자 환영

## 행동 강령

- 존중과 배려로 소통
- 건설적인 피드백
- 다양성 존중
- 교육 목적에 부합하는 기여

## 질문이 있으신가요?

GitHub Issues에 질문을 남겨주세요!

---

다시 한번 기여해 주셔서 감사합니다! 🙏
