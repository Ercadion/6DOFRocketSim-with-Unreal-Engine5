#pragma once
#include "CoreMinimal.h"
#include "GameFramework/PlayerController.h"
#include "RocketPlayerController.generated.h"

UCLASS()
class ROCKETSIM_API ARocketPlayerController : public APlayerController
{
    GENERATED_BODY()
protected:
    virtual void SetupInputComponent() override;
    virtual void BeginPlay() override;

public:
    UPROPERTY(EditAnywhere, Category = "Camera")
    AActor* WorldCamera;

    UPROPERTY(EditAnywhere, Category = "Camera")
    class ARocketActor* RocketActor;

private:
    void SwitchToWorldView();
    void SwitchToFollowCamera();
    void SwitchToFPSCamera();
    void ToggleLaunch();      // Enter키
};

/*
-- -

### 동작 요약
```
▶ 플레이 시작
↓
Enter키 누름  →  🚀 로켓 발사(위로 10m / s)
Enter키 다시  →  ⏸ 정지
↓
1번키  →  월드 시점
2번키  →  로켓 추적 시점
3번키  →  로켓 1인칭 시점
*/