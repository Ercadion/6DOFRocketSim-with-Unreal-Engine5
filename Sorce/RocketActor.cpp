// RocketActor.cpp
// RocketSim6DOF 물리 엔진 → Unreal Engine 5 연동
//
// ┌─────────────────────────────────────────────────────────────────────┐
// │  좌표계 규약                                                         │
// │  시뮬레이터: 미터(m), +Z 위쪽, 오른손 좌표계                          │
// │  언리얼:    센티미터(cm), +Z 위쪽, 왼손 좌표계                        │
// │  변환: 위치 = sim_pos × 100   (m → cm)                              │
// │        자세 = FQuat(snap.QuatX, snap.QuatY, snap.QuatZ, snap.QuatW) │
// └─────────────────────────────────────────────────────────────────────┘
#include "RocketActor.h"
#include "Camera/CameraComponent.h"
#include "GameFramework/SpringArmComponent.h"
#include "DrawDebugHelpers.h"    // DrawDebugLine (디버그 시각화)
#include "Kismet/KismetSystemLibrary.h"

// ── 물리 시뮬레이터 (PIMPL 격리: UHT는 이 섹션을 파싱하지 않음) ──────────
THIRD_PARTY_INCLUDES_START
#define _USE_MATH_DEFINES        // Windows에서 M_PI 활성화
#include <cmath>
#include "RocketSim6DOF.h"
#include "RocketInputLoader.h"
THIRD_PARTY_INCLUDES_END

// ─────────────────────────────────────────────────────────────────────────────
//  PIMPL 구조체 — 시뮬레이터 + 설정을 UE 바깥 메모리에 격리
// ─────────────────────────────────────────────────────────────────────────────
struct FRocketSimPImpl {
    FRocketConfig    Cfg;
    FRocketSim6DOF*  Sim = nullptr;   // 유효 설정 후 new

    ~FRocketSimPImpl() {
        delete Sim;
        Sim = nullptr;
    }

    // Cfg 완성 후 호출
    void CreateSim() {
        delete Sim;
        Sim = new FRocketSim6DOF(Cfg);
    }
};

// ─────────────────────────────────────────────────────────────────────────────
//  헬퍼: 시뮬레이터 수직 발사 초기 자세
//   Body +X → World +Z  (추력이 수직 위를 향하도록)
//   Y축 -90° 회전: q = (cos(-π/4), 0, sin(-π/4), 0)
// ─────────────────────────────────────────────────────────────────────────────
static FQuaternion MakeVerticalQuat()
{
    const double sq2inv = 0.70710678118654752;
    return { sq2inv, 0.0, -sq2inv, 0.0 };
}

// ─────────────────────────────────────────────────────────────────────────────
//  ARocketActor 생성자 — 기본값
// ─────────────────────────────────────────────────────────────────────────────
ARocketActor::ARocketActor()
{
    PrimaryActorTick.bCanEverTick = true;

    Root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
    SetRootComponent(Root);

    SpringArm = CreateDefaultSubobject<USpringArmComponent>(TEXT("SpringArm"));
    SpringArm->SetupAttachment(Root);
    SpringArm->TargetArmLength          = 1000.f;
    SpringArm->SetRelativeRotation(FRotator(-20.f, 0.f, 0.f));
    SpringArm->bUsePawnControlRotation  = false;

    FollowCamera = CreateDefaultSubobject<UCameraComponent>(TEXT("FollowCamera"));
    FollowCamera->SetupAttachment(SpringArm);

    FPSCamera = CreateDefaultSubobject<UCameraComponent>(TEXT("FPSCamera"));
    FPSCamera->SetupAttachment(Root);
    FPSCamera->SetRelativeLocation(FVector(0.f, 0.f, 200.f));
    FPSCamera->SetRelativeRotation(FRotator(90.f, 0.f, 0.f));
    FPSCamera->bAutoActivate = false;

    // 물리 기본값 (에디터에서 변경 가능)
    RocketActor        = nullptr;
    bIsLaunched        = false;
    bAutoLaunch        = false;
    LaunchDelaySeconds = 0.f;

    DryMassKg          = 0.420f;
    PropMassKg         = 0.095f;
    BurnTimeSeconds    = 1.8f;
    MaxThrustN         = 120.f;

    WindDirection      = FVector(1.f, 0.f, 0.f);
    WindSpeedMPS       = 0.f;

    ThrustAlignedProb    = 1.0f;
    ThrustMaxMisalignDeg = 45.f;
    RandomSeed           = 0;

    LaunchRailLengthM    = 5.f;   // 기본 5m 레일 → 수직 이탈 후 자유 비행

    // 궤적 시각화 기본값
    bDrawTrajectory         = true;
    TrajectoryColorMode     = ETrajectoryColorMode::ByPhase;
    TrajectoryColor         = FLinearColor::White;
    TrajectoryLineThickness = 3.f;
    MaxTrajectoryPoints     = 10000;
    TrajectoryRecordInterval = 0.05f;   // 0.05초마다 포인트 기록

    SimMachNumber  = 0.f;
    SimThrustN     = 0.f;
    SimAltitudeM   = 0.f;
    SimTimeSeconds = 0.0;
}

// ─────────────────────────────────────────────────────────────────────────────
//  BeginPlay
// ─────────────────────────────────────────────────────────────────────────────
void ARocketActor::BeginPlay()
{
    Super::BeginPlay();

    // 초기 ARocketActor 위치를 CAD 로켓 위치에 맞춤
    if (RocketActor)
        SetActorLocation(RocketActor->GetActorLocation());

    // 궤적 초기화
    ClearTrajectory();

    // 시뮬레이터 초기화
    InitSimulator();

    if (bAutoLaunch && LaunchDelaySeconds <= 0.f)
        ToggleLaunch();

    if (GEngine)
        GEngine->AddOnScreenDebugMessage(-1, 3.f, FColor::Green,
            TEXT("[RocketSim6DOF] 초기화 완료. Enter: 발사/정지"));
}

// ─────────────────────────────────────────────────────────────────────────────
//  InitSimulator — 설정 로드 + 시뮬레이터 생성 + 초기 상태 설정
// ─────────────────────────────────────────────────────────────────────────────
void ARocketActor::InitSimulator()
{
    delete SimPImpl;
    SimPImpl = new FRocketSimPImpl();
    FRocketConfig& Cfg = SimPImpl->Cfg;

    // ── 1. JSON / CSV 파일 로드 ───────────────────────────────────────────
    bool loadedFromFile = false;
    if (!ConfigFilePath.IsEmpty())
    {
        std::string path(TCHAR_TO_UTF8(*ConfigFilePath));
        std::string err;
        if (FRocketInputLoader::Load(path, Cfg, err))
        {
            loadedFromFile = true;
            UE_LOG(LogTemp, Log, TEXT("[RocketSim] 파일 로드 성공: %s"), *ConfigFilePath);
        }
        else
        {
            UE_LOG(LogTemp, Warning, TEXT("[RocketSim] 파일 로드 실패: %s → %s"),
                   *ConfigFilePath, UTF8_TO_TCHAR(err.c_str()));
        }
    }

    // ── 2. 에디터 직접 입력값으로 덮어쓰기 (파일 미설정 또는 보완) ─────────
    if (!loadedFromFile)
    {
        // 기본 기체 물리
        Cfg.RefLength   = 0.064;
        Cfg.RefArea     = 3.14159265358979323846 * 0.032 * 0.032;
        Cfg.DryMass     = DryMassKg;
        Cfg.PropMass    = PropMassKg;
        Cfg.BurnTime    = BurnTimeSeconds;
        Cfg.MaxThrust   = MaxThrustN;
        Cfg.Ixx         = 0.00012;
        Cfg.Iyy         = 0.042;
        Cfg.Izz         = 0.042;
        Cfg.XCG_initial = 0.295;
        Cfg.XCG_final   = 0.310;

        // 기본 추력 곡선 (단순 박스형)
        Cfg.ThrustCurve = {
            { 0.000,        0.0 },
            { 0.050, (double)MaxThrustN * 0.833 },
            { 0.200, (double)MaxThrustN },
            { (double)BurnTimeSeconds - 0.1f, (double)MaxThrustN * 0.333 },
            { (double)BurnTimeSeconds, 0.0 }
        };

        // 공력 형상 (Barrowman)
        Cfg.Geometry.RefLength    = Cfg.RefLength;
        Cfg.Geometry.BodyLength   = 0.58;
        Cfg.Geometry.NoseLength   = 0.12;
        Cfg.Geometry.FinCount     = 4;
        Cfg.Geometry.FinSpan      = 0.040;
        Cfg.Geometry.FinRootChord = 0.070;
        Cfg.Geometry.FinTipChord  = 0.030;
        Cfg.Geometry.FinSweep     = 0.020;
        Cfg.Geometry.XCG_initial  = Cfg.XCG_initial;
        Cfg.Geometry.XCP          = 0.370;
    }
    else
    {
        // ── 파일에서 로드한 경우 ──
        // 에디터 값이 기본값(0)이 아니면 덮어쓰기 (사용자 명시 입력 우선)
        if (DryMassKg   > 0.f) Cfg.DryMass  = DryMassKg;
        if (PropMassKg  > 0.f) Cfg.PropMass = PropMassKg;
        if (MaxThrustN  > 0.f) Cfg.MaxThrust = MaxThrustN;

        // RefLength 단위 검증 (0 또는 비정상이면 경고)
        if (Cfg.RefLength <= 0.0 || Cfg.RefLength > 5.0)
        {
            UE_LOG(LogTemp, Warning,
                TEXT("[RocketSim] ⚠ RefLength=%g m 비정상. mm/m 단위 확인!"),
                Cfg.RefLength);
        }
        // Geometry.RefLength 자동 동기 (혹시 0 으로 남아 있으면)
        if (Cfg.Geometry.RefLength <= 0.0)
            Cfg.Geometry.RefLength = Cfg.RefLength;
    }

    // ── 3. 풍향 / 풍속 ───────────────────────────────────────────────────
    FVector WDir = WindDirection.GetSafeNormal();
    Cfg.WindVelocity = {
        WDir.X * WindSpeedMPS,
        WDir.Y * WindSpeedMPS,
        WDir.Z * WindSpeedMPS
    };

    // ── 4. 추력 방향 오차 ────────────────────────────────────────────────
    Cfg.ThrustAlignedProb    = ThrustAlignedProb;
    Cfg.ThrustMaxMisalignDeg = ThrustMaxMisalignDeg;

    // ── 4-b. 발사 레일 ───────────────────────────────────────────────────
    // 0으로 설정하면 레일 없음 (즉시 자유 비행)
    Cfg.LaunchRailLength = static_cast<double>(LaunchRailLengthM);

    // ── 5. 시뮬레이터 인스턴스 생성 ──────────────────────────────────────
    SimPImpl->CreateSim();

    // 시드 설정
    if (RandomSeed != 0)
        SimPImpl->Sim->SetSeed(static_cast<uint32_t>(RandomSeed));

    // ── 6. 초기 상태 설정 ────────────────────────────────────────────────
    FRocketState Init;

    // 발사대 위치: CAD 로켓 액터의 현재 위치를 m 단위로 변환
    if (RocketActor)
    {
        FVector Loc = RocketActor->GetActorLocation(); // cm
        Init.Position = { Loc.X / 100.0, Loc.Y / 100.0, Loc.Z / 100.0 };
    }
    else
    {
        Init.Position = { 0, 0, 0 };
    }

    Init.Velocity        = { 0, 0, 0 };
    Init.Attitude        = MakeVerticalQuat(); // Body+X → World+Z (수직 발사)
    Init.AngularVelocity = { 0, 0, 0 };
    Init.Mass            = Cfg.DryMass + Cfg.PropMass;
    Init.Time            = 0.0;

    SimPImpl->Sim->Reset(Init);

    UE_LOG(LogTemp, Log,
        TEXT("[RocketSim] 시뮬레이터 준비 완료 | 발사질량=%.3f kg | 연소시간=%.2f s | 최대추력=%.1f N | Isp=%.1fs"),
        Init.Mass, Cfg.BurnTime, Cfg.MaxThrust, Cfg.IspSeconds);
    UE_LOG(LogTemp, Log,
        TEXT("[RocketSim] 공력 형상 | RefArea=%.6f m² | RefLength=%.4f m | XCP=%.3f m | XCG=%.3f m"),
        Cfg.RefArea, Cfg.RefLength, Cfg.Geometry.XCP, Cfg.XCG_initial);

    // ── 항력 비활성 경고: ref_area 가 너무 작으면 사실상 진공 비행 → 비현실적 고도 ──
    if (Cfg.RefArea < 1.0e-6)
    {
        if (GEngine)
        {
            GEngine->AddOnScreenDebugMessage(-1, 10.f, FColor::Red,
                TEXT("⚠ RefArea ≈ 0 → 항력 비활성! 비현실적 고도 예상."));
            GEngine->AddOnScreenDebugMessage(-1, 10.f, FColor::Red,
                TEXT("    발사 입력 빌더(Step 02) 를 먼저 실행해 외형 값을 채우세요."));
        }
        UE_LOG(LogTemp, Error,
            TEXT("[RocketSim] ⚠ RefArea≈0 → 항력 0 → 예상보다 매우 높은 고도!"));
    }

    // 정적 안정도 점검 (SM = (XCP - XCG)/RefLength)
    if (Cfg.RefLength > 1e-6)
    {
        double SM = (Cfg.Geometry.XCP - Cfg.XCG_initial) / Cfg.RefLength;
        if (SM < 1.0 && GEngine)
        {
            GEngine->AddOnScreenDebugMessage(-1, 5.f,
                SM < 0.0 ? FColor::Red : FColor::Yellow,
                FString::Printf(TEXT("⚠ Static Margin = %.2f cal — %s"),
                    SM, SM < 0.0 ? TEXT("불안정!") : TEXT("마진 부족 (1cal 미만)")));
        }
        if (Cfg.InertiaTable.size() >= 2 && GEngine)
        {
            GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Green,
                FString::Printf(TEXT("✓ Inertia table: %d points (time-varying I)"),
                    (int)Cfg.InertiaTable.size()));
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Tick — 물리 적분 → CAD 로켓 Transform 반영
// ─────────────────────────────────────────────────────────────────────────────
void ARocketActor::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);

    // ── 자동 발사 지연 타이머 ────────────────────────────────────────────
    if (bAutoLaunch && !bIsLaunched && LaunchDelaySeconds > 0.f)
    {
        LaunchDelayTimer += DeltaTime;
        if (LaunchDelayTimer >= LaunchDelaySeconds)
            ToggleLaunch();
    }

    // ── 발사 중이 아니면 스킵 ────────────────────────────────────────────
    if (!bIsLaunched || !SimPImpl || !SimPImpl->Sim)
        return;

    // ── RK45 적분 (DeltaTime 동안) ────────────────────────────────────────
    SimPImpl->Sim->Step(static_cast<double>(DeltaTime));

    // ── 스냅샷 → 언리얼 Actor Transform 반영 ─────────────────────────────
    ApplySnapshotToActor();
}

// ─────────────────────────────────────────────────────────────────────────────
//  ApplySnapshotToActor — FSimSnapshot 데이터를 Actor에 적용
// ─────────────────────────────────────────────────────────────────────────────
void ARocketActor::ApplySnapshotToActor()
{
    if (!SimPImpl || !SimPImpl->Sim) return;

    FSimSnapshot snap = SimPImpl->Sim->GetSnapshot();

    // ── 좌표 변환 ────────────────────────────────────────────────────────
    // 시뮬: 미터(m), Unreal: 센티미터(cm)  → ×100
    FVector NewLocationCM(
        snap.PosX * 100.f,
        snap.PosY * 100.f,
        snap.PosZ * 100.f
    );

    // 쿼터니언: 시뮬 (w, x, y, z) → Unreal FQuat(x, y, z, w)
    FQuat NewQuat(snap.QuatX, snap.QuatY, snap.QuatZ, snap.QuatW);
    NewQuat.Normalize();

    // ── CAD 로켓 액터 이동 ────────────────────────────────────────────────
    if (RocketActor)
    {
        RocketActor->SetActorLocationAndRotation(NewLocationCM, NewQuat,
            /*bSweep=*/false, nullptr, ETeleportType::TeleportPhysics);
    }

    // ARocketActor 자신도 동기화 (카메라/SpringArm 추적)
    SetActorLocationAndRotation(NewLocationCM, NewQuat,
        false, nullptr, ETeleportType::TeleportPhysics);

    // ── Blueprint / 디버그용 읽기 전용 필드 갱신 ─────────────────────────
    SimPosition     = FVector(snap.PosX,  snap.PosY,  snap.PosZ);
    SimVelocity     = FVector(snap.VelX,  snap.VelY,  snap.VelZ);
    SimAcceleration = FVector(snap.AccX,  snap.AccY,  snap.AccZ);
    SimMachNumber   = snap.Mach;
    SimThrustN      = snap.ThrustN;
    SimAltitudeM    = snap.AltAGL;
    SimTimeSeconds  = snap.SimTime;

    // ── 궤적 시각화 ───────────────────────────────────────────────────────
    float SpeedMS = SimVelocity.Size();
    bool  bBurn   = (snap.ThrustN > 0.1f);
    RecordAndDrawTrajectory(NewLocationCM, SpeedMS,
                             snap.ThrustN, snap.AltAGL,
                             snap.Mach, snap.SimTime, bBurn);

    // ── 이벤트 발생 ───────────────────────────────────────────────────────
    const FRocketConfig& Cfg = SimPImpl->Cfg;
    const FRocketState&  S   = SimPImpl->Sim->GetState();

    // 이륙 감지: 발사 후 z > 0.1 m 이상 상승
    if (!bLiftedOff && S.Position.z > 0.1)
    {
        bLiftedOff = true;
        OnLiftoff();
        UE_LOG(LogTemp, Log, TEXT("[RocketSim] 🚀 이륙!"));
    }

    // 연소 완료 감지
    if (!bBurnedOut && S.Time > Cfg.BurnTime + 0.05)
    {
        bBurnedOut = true;
        OnBurnout();
        UE_LOG(LogTemp, Log,
            TEXT("[RocketSim] 🔥 연소 완료 | 고도=%.1f m | 속도=%.1f m/s"),
            S.Position.z,
            FMath::Sqrt(S.Velocity.x*S.Velocity.x +
                        S.Velocity.y*S.Velocity.y +
                        S.Velocity.z*S.Velocity.z));
    }

    // 착지 감지
    if (!bLanded && bLiftedOff && S.Position.z <= 0.01 &&
        S.Velocity.z >= -0.5 && S.Time > 1.0)
    {
        bLanded = true;
        bIsLaunched = false;
        OnLanded();
        UE_LOG(LogTemp, Log,
            TEXT("[RocketSim] 🛬 착지 완료 | 비행시간=%.1f s"), S.Time);
    }

    // ── 화면 디버그 출력 (60fps 마다 1회) ────────────────────────────────
    if (GEngine && FMath::Fmod(static_cast<float>(S.Time), 0.016f) < 0.001f)
    {
        GEngine->AddOnScreenDebugMessage(1, 0.02f, FColor::Cyan,
            FString::Printf(TEXT("T=%.2fs | Alt=%.1fm | V=%.1fm/s | Mach=%.3f"),
                S.Time, S.Position.z,
                FMath::Sqrt(S.Velocity.x*S.Velocity.x +
                            S.Velocity.y*S.Velocity.y +
                            S.Velocity.z*S.Velocity.z),
                snap.Mach));
        GEngine->AddOnScreenDebugMessage(2, 0.02f, FColor::Yellow,
            FString::Printf(TEXT("Thrust=%.1fN | Drag=%.2fN | Mass=%.3fkg"),
                snap.ThrustN, snap.DragN, S.Mass));
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  ToggleLaunch — Enter 키 콜백
// ─────────────────────────────────────────────────────────────────────────────
void ARocketActor::ToggleLaunch()
{
    bIsLaunched = !bIsLaunched;

    if (bIsLaunched)
    {
        // 재발사: 시뮬레이터 재초기화
        InitSimulator();
        bLiftedOff = false;
        bBurnedOut = false;
        bLanded    = false;
        bApogeeMarked   = false;        // Apogee 마커 다시 표시 가능
        MaxAltitudeSeen = 0.f;
        LaunchDelayTimer = 0.f;          // 자동 발사 지연 타이머 리셋
        ClearTrajectory();               // 기존 궤적 지우기
    }

    if (GEngine)
    {
        FString Msg = bIsLaunched
            ? FString::Printf(TEXT("🚀 발사! (질량=%.3fkg, 추력=%.1fN)"),
                SimPImpl ? SimPImpl->Cfg.DryMass + SimPImpl->Cfg.PropMass : 0.f,
                SimPImpl ? SimPImpl->Cfg.MaxThrust : 0.f)
            : TEXT("⏸ 시뮬레이션 정지");
        GEngine->AddOnScreenDebugMessage(-1, 3.f,
            bIsLaunched ? FColor::Orange : FColor::White, Msg);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  SetRocketState — Blueprint/외부 C++ 에서 강제 위치/자세 설정
// ─────────────────────────────────────────────────────────────────────────────
void ARocketActor::SetRocketState(FVector InPosition, FRotator InRotation)
{
    if (!SimPImpl || !SimPImpl->Sim) return;

    FRocketState S = SimPImpl->Sim->GetState();
    S.Position = { InPosition.X, InPosition.Y, InPosition.Z };
    // 자세는 FRotator → Quaternion 변환
    FQuat Q = InRotation.Quaternion();
    S.Attitude = { Q.W, Q.X, Q.Y, Q.Z };
    SimPImpl->Sim->Reset(S);
}

// ─────────────────────────────────────────────────────────────────────────────
//  RecordAndDrawTrajectory — 궤적 포인트 기록 + DrawDebugLine 호출
// ─────────────────────────────────────────────────────────────────────────────
void ARocketActor::RecordAndDrawTrajectory(const FVector& NewLocCM,
    float SpeedMS, float ThrustN, float AltM, float Mach,
    double TimeS, bool bBurning)
{
    if (!bDrawTrajectory || !GetWorld()) return;

    // 기록 인터벌 체크
    TrajectoryRecordTimer += GetWorld()->GetDeltaSeconds();
    if (TrajectoryRecordTimer < TrajectoryRecordInterval) return;
    TrajectoryRecordTimer = 0.f;

    // 최대 포인트 초과 시 가장 오래된 포인트 제거
    if (TrajectoryData.Num() >= MaxTrajectoryPoints)
        TrajectoryData.RemoveAt(0, 1, EAllowShrinking::No);

    // 속력 최대값 업데이트 (BySpeed 색상 정규화용)
    MaxSpeedSeen = FMath::Max(MaxSpeedSeen, SpeedMS);

    // 포인트 저장
    FTrajectoryPoint Pt;
    Pt.LocationCM = NewLocCM;
    Pt.VelocityMS = SimVelocity;
    Pt.SpeedMS    = SpeedMS;
    Pt.ThrustN    = ThrustN;
    Pt.AltitudeM  = AltM;
    Pt.MachNum    = Mach;
    Pt.TimeS      = TimeS;
    Pt.bBurning   = bBurning;
    TrajectoryData.Add(Pt);

    // 2포인트 이상이면 직선 드로잉
    if (TrajectoryData.Num() < 2) return;

    const FTrajectoryPoint& Prev = TrajectoryData[TrajectoryData.Num() - 2];
    const FTrajectoryPoint& Curr = TrajectoryData[TrajectoryData.Num() - 1];

    FColor LineColor = GetTrajectoryColor(Curr);

    DrawDebugLine(
        GetWorld(),
        Prev.LocationCM,
        Curr.LocationCM,
        LineColor,
        true,           // 영구 유지 (지울 때까지)
        -1.f,
        0,
        TrajectoryLineThickness
    );

    // 정점(apogee) 표시: 이전보다 고도 낮아지기 시작한 첫 프레임에 1회만 마커
    // bApogeeMarked 가드 + MaxAltitudeSeen 추적으로 매 프레임 중복 그림 방지
    MaxAltitudeSeen = FMath::Max(MaxAltitudeSeen, Curr.AltitudeM);
    if (!bApogeeMarked &&
        Curr.AltitudeM < Prev.AltitudeM &&
        Prev.AltitudeM > 100.f &&
        !Curr.bBurning)
    {
        bApogeeMarked = true;
        DrawDebugSphere(GetWorld(), Prev.LocationCM, 50.f, 8,
                        FColor::Yellow, true, -1.f);
        DrawDebugString(GetWorld(), Prev.LocationCM + FVector(0,0,100.f),
                        FString::Printf(TEXT("▲ 정점 %.0fm"), Prev.AltitudeM),
                        nullptr, FColor::Yellow, -1.f, true, 1.5f);
        UE_LOG(LogTemp, Log, TEXT("[RocketSim] ▲ Apogee 도달: %.1f m"), Prev.AltitudeM);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  GetTrajectoryColor — 색상 모드에 따라 선 색상 결정
// ─────────────────────────────────────────────────────────────────────────────
FColor ARocketActor::GetTrajectoryColor(const FTrajectoryPoint& Pt) const
{
    switch (TrajectoryColorMode)
    {
    case ETrajectoryColorMode::ByPhase:
        // 연소 중: 주황, 연소 후 상승: 초록, 하강: 파랑
        if (Pt.bBurning)
        {
            return FColor(255, 140, 0);    // 주황
        }
        else if (Pt.VelocityMS.Z > 0.f)
        {
            return FColor(0, 220, 80);     // 초록
        }
        else
        {
            return FColor(60, 140, 255);   // 파랑
        }

    case ETrajectoryColorMode::BySpeed:
        // 느림: 파랑 → 중간: 초록 → 빠름: 빨강 (열지도)
        {
            float t = FMath::Clamp(Pt.SpeedMS / FMath::Max(MaxSpeedSeen, 1.f), 0.f, 1.f);
            if (t < 0.5f) {
                float s = t * 2.f;
                return FColor(0, (uint8)(s * 255), (uint8)((1.f-s) * 255));
            } else {
                float s = (t - 0.5f) * 2.f;
                return FColor((uint8)(s * 255), (uint8)((1.f-s) * 255), 0);
            }
        }

    case ETrajectoryColorMode::SingleColor:
    default:
        return TrajectoryColor.ToFColor(true);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  ClearTrajectory — 궤적 데이터 초기화 + 뷰포트 Debug 드로잉 삭제
// ─────────────────────────────────────────────────────────────────────────────
void ARocketActor::ClearTrajectory()
{
    TrajectoryData.Empty();
    TrajectoryRecordTimer = 0.f;
    MaxSpeedSeen = 1.f;

    if (GetWorld())
        FlushPersistentDebugLines(GetWorld());
}

// ─────────────────────────────────────────────────────────────────────────────
//  SaveTrajectoryToCSV — 궤적 데이터를 CSV 파일로 저장
//  (Python/Excel 3D 그래프용)
// ─────────────────────────────────────────────────────────────────────────────
void ARocketActor::SaveTrajectoryToCSV(const FString& FilePath)
{
    if (TrajectoryData.Num() == 0)
    {
        UE_LOG(LogTemp, Warning, TEXT("[RocketSim] 저장할 궤적 데이터가 없습니다"));
        return;
    }

    FString Content;
    // 헤더
    Content += TEXT("time_s,pos_x_m,pos_y_m,pos_z_m,vel_x,vel_y,vel_z,speed_ms,thrust_N,altitude_m,mach\n");

    for (const FTrajectoryPoint& Pt : TrajectoryData)
    {
        // cm → m 변환
        Content += FString::Printf(
            TEXT("%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n"),
            Pt.TimeS,
            Pt.LocationCM.X / 100.f,
            Pt.LocationCM.Y / 100.f,
            Pt.LocationCM.Z / 100.f,
            Pt.VelocityMS.X, Pt.VelocityMS.Y, Pt.VelocityMS.Z,
            Pt.SpeedMS,
            Pt.ThrustN,
            Pt.AltitudeM,
            Pt.MachNum
        );
    }

    if (FFileHelper::SaveStringToFile(Content, *FilePath))
    {
        UE_LOG(LogTemp, Log, TEXT("[RocketSim] 궤적 CSV 저장: %s (%d 포인트)"),
               *FilePath, TrajectoryData.Num());
    }
    else
    {
        UE_LOG(LogTemp, Error, TEXT("[RocketSim] CSV 저장 실패: %s"), *FilePath);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  EndPlay — 메모리 정리
// ─────────────────────────────────────────────────────────────────────────────
void ARocketActor::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    Super::EndPlay(EndPlayReason);

    delete SimPImpl;
    SimPImpl = nullptr;
}
