#include "RocketPlayerController.h"
#include "RocketActor.h"
#include "Camera/CameraComponent.h"
#include "Kismet/GameplayStatics.h"

void ARocketPlayerController::BeginPlay()
{
    Super::BeginPlay();

    // ← 이거 추가! 키보드 입력을 게임으로 받게 설정
    FInputModeGameOnly InputMode;
    SetInputMode(InputMode);
    bShowMouseCursor = false;

    // 자동으로 Actor 찾기
    if (!RocketActor)
    {
        TArray<AActor*> Found;
        UGameplayStatics::GetAllActorsOfClass(
            GetWorld(), ARocketActor::StaticClass(), Found);
        if (Found.Num() > 0)
            RocketActor = Cast<ARocketActor>(Found[0]);
    }

    if (!WorldCamera)
    {
        TArray<AActor*> Found;
        UGameplayStatics::GetAllActorsWithTag(
            GetWorld(), FName("WorldCam"), Found);
        if (Found.Num() > 0)
            WorldCamera = Found[0];
    }

    // 화면에 연결 상태 표시
    if (GEngine)
    {
        GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Green,
            RocketActor ? TEXT("✅ RocketActor 연결됨") : TEXT("❌ RocketActor 없음"));
        GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Green,
            WorldCamera ? TEXT("✅ WorldCamera 연결됨") : TEXT("❌ WorldCamera 없음"));
    }

    SwitchToWorldView();
}

void ARocketPlayerController::SetupInputComponent()
{
    Super::SetupInputComponent();

    // 카메라 전환
    InputComponent->BindKey(EKeys::One, IE_Pressed, this,
        &ARocketPlayerController::SwitchToWorldView);
    InputComponent->BindKey(EKeys::Two, IE_Pressed, this,
        &ARocketPlayerController::SwitchToFollowCamera);
    InputComponent->BindKey(EKeys::Three, IE_Pressed, this,
        &ARocketPlayerController::SwitchToFPSCamera);

    // Enter키 → 발사/정지 토글
    InputComponent->BindKey(EKeys::Enter, IE_Pressed, this,
        &ARocketPlayerController::ToggleLaunch);
}

void ARocketPlayerController::ToggleLaunch()
{
    if (RocketActor)
        RocketActor->ToggleLaunch();
}

void ARocketPlayerController::SwitchToWorldView()
{
    // 로켓 카메라 둘 다 비활성화 (null 체크 추가)
    if (RocketActor)
    {
        if (RocketActor->FollowCamera) RocketActor->FollowCamera->SetActive(false);
        if (RocketActor->FPSCamera)    RocketActor->FPSCamera->SetActive(false);
    }

    if (WorldCamera)
    {
        SetViewTargetWithBlend(WorldCamera, 0.3f);
        if (GEngine)
            GEngine->AddOnScreenDebugMessage(-1, 2.f, FColor::White,
                TEXT("1번: 월드 카메라"));
    }
    else
    {
        if (GEngine)
            GEngine->AddOnScreenDebugMessage(-1, 2.f, FColor::Red,
                TEXT("❌ WorldCamera 없음!"));
    }
}

void ARocketPlayerController::SwitchToFollowCamera()
{
    if (RocketActor && RocketActor->FollowCamera)
    {
        // 카메라 전환 전 위치 강제 동기화
        if (RocketActor->RocketActor)
        {
            FVector RocketPos = RocketActor->RocketActor->GetActorLocation();
            RocketActor->SetActorLocation(RocketPos);
        }

        RocketActor->FollowCamera->SetActive(true);
        if (RocketActor->FPSCamera) RocketActor->FPSCamera->SetActive(false);
        SetViewTargetWithBlend(RocketActor, 0.5f);
        if (GEngine)
            GEngine->AddOnScreenDebugMessage(-1, 2.f, FColor::Cyan,
                TEXT("2번: 추적 카메라"));
    }
}

void ARocketPlayerController::SwitchToFPSCamera()
{
    if (RocketActor && RocketActor->FPSCamera)
    {
        // 카메라 전환 전 위치 강제 동기화
        if (RocketActor->RocketActor)
        {
            FVector RocketPos = RocketActor->RocketActor->GetActorLocation();
            RocketActor->SetActorLocation(RocketPos);
        }

        RocketActor->FPSCamera->SetActive(true);
        if (RocketActor->FollowCamera) RocketActor->FollowCamera->SetActive(false);
        SetViewTargetWithBlend(RocketActor, 0.3f);
        if (GEngine)
            GEngine->AddOnScreenDebugMessage(-1, 2.f, FColor::Yellow,
                TEXT("3번: 로켓 시점"));
    }
}