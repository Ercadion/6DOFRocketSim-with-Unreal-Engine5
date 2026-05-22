#pragma once
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Camera/CameraComponent.h"
#include "GameFramework/SpringArmComponent.h"
#include "Components/SceneComponent.h"
#include "RocketActor.generated.h"

// ─────────────────────────────────────────────────────────────────────────────
//  궤적 색상 모드 (UENUM은 반드시 전역 스코프에 선언)
// ─────────────────────────────────────────────────────────────────────────────
UENUM(BlueprintType)
enum class ETrajectoryColorMode : uint8
{
    ByPhase     UMETA(DisplayName = "By Phase"),     // 연소→상승→하강 단계별 색
    BySpeed     UMETA(DisplayName = "By Speed"),     // 속도 크기에 따라 색 변화
    SingleColor UMETA(DisplayName = "Single Color")  // 단일 색상
};

// ─────────────────────────────────────────────────────────────────────────────
//  전방 선언 (PIMPL 패턴)
//  시뮬레이터 헤더는 RocketActor.cpp 에서만 포함.
//  UHT(언리얼 헤더 툴)가 STL / 수학 헤더를 파싱하는 문제를 방지.
// ─────────────────────────────────────────────────────────────────────────────
struct FRocketSimPImpl;   // .cpp 에 정의된 PIMPL 래퍼

// ─────────────────────────────────────────────────────────────────────────────
//  ARocketActor
//  - RocketSim6DOF 물리 엔진을 구동하고
//  - 결과(위치·자세)를 Datasmith/CAD 로켓 액터에 매 Tick 적용
// ─────────────────────────────────────────────────────────────────────────────
UCLASS()
class ROCKETSIM_API ARocketActor : public AActor
{
    GENERATED_BODY()

// ── 생성자 / 언리얼 이벤트 ────────────────────────────────────────────────
public:
    ARocketActor();
    virtual void BeginPlay() override;
    virtual void Tick(float DeltaTime) override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

// ── 씬 컴포넌트 ───────────────────────────────────────────────────────────
    UPROPERTY(VisibleAnywhere, Category = "Rocket")
    USceneComponent* Root;

// ── 카메라 ────────────────────────────────────────────────────────────────
    UPROPERTY(VisibleAnywhere, Category = "Camera")
    USpringArmComponent* SpringArm;

    UPROPERTY(VisibleAnywhere, Category = "Camera")
    UCameraComponent* FollowCamera;

    UPROPERTY(VisibleAnywhere, Category = "Camera")
    UCameraComponent* FPSCamera;

    UPROPERTY(EditAnywhere, Category = "Camera")
    AActor* WorldCamera;

// ── 로켓 CAD 모델 ─────────────────────────────────────────────────────────
    // 에디터에서 Datasmith import 한 로켓 CAD 액터를 연결
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rocket|Setup")
    AActor* RocketActor;

// ── 발사 제어 ─────────────────────────────────────────────────────────────
    // Enter 키로 발사 ON/OFF (RocketPlayerController 에서 호출)
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Rocket|Launch")
    bool bIsLaunched;

    // true: BeginPlay 즉시 발사 (Enter 키 불필요)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rocket|Launch")
    bool bAutoLaunch;

    // 발사 지연 [s] (bAutoLaunch=true 일 때)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rocket|Launch",
              meta = (ClampMin = "0.0"))
    float LaunchDelaySeconds;

    void ToggleLaunch();   // PlayerController 에서 호출

// ── 물리 설정 파일 ────────────────────────────────────────────────────────
    // JSON 또는 CSV 경로 (비워두면 아래 직접 입력값 사용)
    // 예: "C:/MyProject/Content/Rockets/my_motor.json"
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rocket|Physics",
              meta = (FilePathFilter = "json,csv"))
    FString ConfigFilePath;

// ── 물리 직접 입력 (ConfigFilePath 비워두거나 덮어쓰기용) ──────────────────
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rocket|Physics")
    float DryMassKg;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rocket|Physics")
    float PropMassKg;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rocket|Physics")
    float BurnTimeSeconds;

    // 최대 추력 [N] (ThrustCurve 없을 때 구형 단순 추력으로 사용)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rocket|Physics")
    float MaxThrustN;

// ── 발사 레일 ────────────────────────────────────────────────────────────
    // 로켓이 이 고도(m)를 넘을 때까지 자세를 수직으로 고정.
    // 0 = 레일 없음(바람 즉시 영향), 5 = 5m 수직 상승 후 자유 비행(기본값)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rocket|Physics",
              meta = (ClampMin = "0.0"))
    float LaunchRailLengthM;

// ── 풍향 / 풍속 ───────────────────────────────────────────────────────────
    // World 좌표 방향 벡터 (자동 정규화 됨)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rocket|Wind")
    FVector WindDirection;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rocket|Wind",
              meta = (ClampMin = "0.0"))
    float WindSpeedMPS;

// ── 추력 방향 오차 (TVC 불완전성) ────────────────────────────────────────
    // 정상 추력 확률: 0.95 → 95% 정렬, 5% 랜덤 편향
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rocket|Misalignment",
              meta = (ClampMin = "0.0", ClampMax = "1.0"))
    float ThrustAlignedProb;

    // 편향 시 최대 각도 [deg], 권장 범위 1~45
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rocket|Misalignment",
              meta = (ClampMin = "0.0", ClampMax = "90.0"))
    float ThrustMaxMisalignDeg;

// ── 재현성 시드 (0: 비재현 / 랜덤) ──────────────────────────────────────
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rocket|Physics")
    int32 RandomSeed;

// ── 궤적 시각화 (언리얼 뷰포트 실시간 드로잉) ─────────────────────────────
    // true: 로켓이 날아가면서 궤적 선을 뷰포트에 실시간으로 그림
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rocket|Trajectory")
    bool bDrawTrajectory;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rocket|Trajectory")
    ETrajectoryColorMode TrajectoryColorMode;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rocket|Trajectory")
    FLinearColor TrajectoryColor;   // SingleColor 모드일 때 사용

    // 궤적 선 두께 [cm]
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rocket|Trajectory",
              meta = (ClampMin = "0.5"))
    float TrajectoryLineThickness;

    // 최대 궤적 포인트 수 (메모리 제한)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rocket|Trajectory",
              meta = (ClampMin = "100", ClampMax = "50000"))
    int32 MaxTrajectoryPoints;

    // 몇 초마다 포인트를 기록할지 (0.0 = 매 프레임)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rocket|Trajectory",
              meta = (ClampMin = "0.0"))
    float TrajectoryRecordInterval;

    // Blueprint에서 궤적 지우기
    UFUNCTION(BlueprintCallable, Category = "Rocket|Trajectory")
    void ClearTrajectory();

    // 궤적을 CSV 파일로 저장 (FilePath 경로에 저장)
    UFUNCTION(BlueprintCallable, Category = "Rocket|Trajectory")
    void SaveTrajectoryToCSV(const FString& FilePath);

// ── 읽기 전용 디버그 출력 ────────────────────────────────────────────────
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Rocket|Debug")
    FVector SimPosition;       // 현재 위치 [m]

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Rocket|Debug")
    FVector SimVelocity;       // 현재 속도 [m/s]

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Rocket|Debug")
    FVector SimAcceleration;   // 현재 가속도 [m/s²]

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Rocket|Debug")
    float SimMachNumber;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Rocket|Debug")
    float SimThrustN;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Rocket|Debug")
    float SimAltitudeM;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Rocket|Debug")
    double SimTimeSeconds;

// ── Blueprint 이벤트 ──────────────────────────────────────────────────────
    UFUNCTION(BlueprintImplementableEvent, Category = "Rocket|Events")
    void OnLiftoff();      // 발사대 이탈 시 호출

    UFUNCTION(BlueprintImplementableEvent, Category = "Rocket|Events")
    void OnBurnout();      // 연소 완료 시 호출

    UFUNCTION(BlueprintImplementableEvent, Category = "Rocket|Events")
    void OnLanded();       // 착지 시 호출

    // 스냅샷 전달 (Blueprint/다른 C++ 에서 호출 가능)
    UFUNCTION(BlueprintCallable, Category = "Rocket")
    void SetRocketState(FVector InPosition, FRotator InRotation);

private:
    // ── PIMPL: 시뮬레이터 인스턴스 ───────────────────────────────────────
    // FRocketSimPImpl 은 RocketActor.cpp 에 정의 (STL/물리 헤더 격리)
    FRocketSimPImpl* SimPImpl = nullptr;

    // ── 내부 상태 ─────────────────────────────────────────────────────────
    float   LaunchDelayTimer = 0.f;
    bool    bLiftedOff  = false;
    bool    bBurnedOut  = false;
    bool    bLanded     = false;
    double  PrevSimTime = 0.0;

    // ── 궤적 데이터 저장소 ────────────────────────────────────────────────
    struct FTrajectoryPoint {
        FVector  LocationCM;   // 위치 [cm]  Unreal 좌표
        FVector  VelocityMS;   // 속도 [m/s]
        float    SpeedMS;      // 속력 크기
        float    ThrustN;
        float    AltitudeM;
        float    MachNum;
        double   TimeS;
        bool     bBurning;     // 연소 중 여부
    };
    TArray<FTrajectoryPoint> TrajectoryData;
    float  TrajectoryRecordTimer = 0.f;
    float  MaxSpeedSeen = 1.f;  // 색상 정규화용
    bool   bApogeeMarked = false;  // Apogee 마커 1회만 그리기 가드
    float  MaxAltitudeSeen = 0.f;  // 정점 추적용

    void InitSimulator();
    void ApplySnapshotToActor();
    void RecordAndDrawTrajectory(const FVector& NewLocCM, float SpeedMS,
                                  float ThrustN, float AltM, float Mach,
                                  double TimeS, bool bBurning);
    FColor GetTrajectoryColor(const FTrajectoryPoint& Pt) const;
};
