# LineWatch

realvirtual Starter 기반 로봇 피킹·분류 라인 관제 시뮬레이터 포트폴리오입니다.

## 현재 진행 상태

Unity 환경 구성과 Starter Getting Started 데모 실행 검증, LW-01 RobotLab 작업 씬·SCARA 작업 프리팹 구성을 완료했습니다. 씬 재열기·참조 검사와 기본 Play 검증을 마쳤습니다. 자체 4축 SCARA FK·IK, 분류 제어, 관제 UI, 고장 대응과 외부 서버 연동은 기획·개발 예정 범위입니다.

자세한 범위와 검증 기준은 [기획안](Assets/Docs/Starter_Portfolio_Plan.md)에 있습니다.
실제 작업 순서와 완료 기준은 [작업 체크리스트](Assets/Docs/LineWatch_Work_Plan.md)를 따릅니다.

## 개발 환경

- Unity 6000.3.21f1 / URP
- realvirtual Starter 6.3.6 (별도 설치)
- Windows PC 우선

## 프로젝트 열기

1. 이 저장소를 클론하고 Unity Hub에 프로젝트 폴더를 추가합니다.
2. [realvirtual Digital Twin Starter 6.3](https://assetstore.unity.com/packages/tools/utilities/realvirtual-digital-twin-starter-6-3-366612)을 본인 Unity 계정의 My Assets에 추가합니다.
3. Unity Package Manager의 My Assets에서 Starter를 다운로드·가져오기 합니다. 검증한 버전은 6.3.6입니다.
4. Active Input Handling을 Both로 설정하고 Unity를 재시작합니다. URP와 TextMesh Pro 필수 리소스를 확인합니다.
5. Package Manager에서 Starter의 Getting Started 샘플을 가져오고 DemoRealvirtual 씬을 열어 Play로 확인합니다.

세부 설치 절차는 [제작사 가이드](https://realvirtual.io/doc/unity/basics/installation/)를 참고합니다. Starter 원본과 샘플은 공개 저장소에 포함하지 않아 각 개발 환경에서 직접 설치해야 합니다.

## 에셋 활용과 본인 구현 범위

구동부·센서·이송 등의 기반 기능과 제공 모델은 realvirtual Starter를 활용합니다. 본인 구현 예정 범위는 FK·IK 계산, 관절 제한과 해 선택, 로봇 피킹 제어, 관제 UI, 알람·복구, 외부 서버 연동과 성능 검증입니다. 제공 데모의 기능을 자체 구현 성과로 표시하지 않습니다.

New Gen Urban은 이 포트폴리오에 사용하지 않습니다.

## 저장소에서 제외하는 항목

- Unity 캐시·로그·빌드 결과와 자동 생성 IDE 솔루션
- 유료 에셋 보관 폴더 Assets/10.Assets 및 해당 폴더 메타데이터
- realvirtual Starter 원본 패키지와 가져온 Starter 샘플

로컬에 설치한 에셋은 삭제하지 않습니다. 제3자 에셋의 이용 조건은 각 제작사 라이선스를 따릅니다.
