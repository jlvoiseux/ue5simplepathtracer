#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Components/PointLightComponent.h"
#include "Engine/TextureRenderTarget2D.h"

#include "PathTracerMaterialComponent.h"

#include "PathTracerComponent.generated.h"

DECLARE_LOG_CATEGORY_EXTERN(LogPathTracer, Log, All)

UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class UE5SIMPLEPATHTRACER_API UPathTracerComponent : public UActorComponent
{
    GENERATED_BODY()

public:
    UPathTracerComponent();

    struct PathTracingCamera
    {
        FVector CameraLocation = FVector::Zero();
        FRotator CameraRotation = FRotator::ZeroRotator;
        float FOV = 0.0f;
        float AspectRatio = 0.0f;
        FVector Forward = FVector::Zero();
        FVector Right = FVector::Zero();
        FVector Up = FVector::Zero();
        float HalfWidth = 0.0f;
        float HalfHeight = 0.0f;
    };

    struct PathTracingRay
    {
        FVector Origin = FVector::Zero();
        FVector Direction = FVector::Zero();
    };

    struct PathTracingPID
    {
        float Kp = 0.01f;
        float Ki = 0.001f;
        float Kd = 0.005f;
        float IntegralError = 0.0f;
        float LastError = 0.0f;
        float MovingAverageTime = 1.f / 120.f;
        int64 FrameCount = 0;
        int64 WarmupFrames = 60;
        int64 MinBatchSize = 1000;
        int64 MaxBatchSize = 50000;
        int64 CurrBatchSize = 5000;
    };

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Path Tracer")
    UMaterialInterface* PathTracerMaterial;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Path Tracer")
    int32 MaxBounces = 5;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Path Tracer")
    int32 SamplesPerPixel = 10;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Path Tracer")
    float FrameTimeBudget = 1.0f / 120.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Path Tracer")
    int32 FramesBeforeStartingRender = 10;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Path Tracer")
    bool AdaptiveSampling = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Path Tracer")
    float RussianRouletteStartDepth = 3;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Path Tracer")
    float RussianRouletteProbability = 0.5f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Path Tracer Debug")
    bool ShowNormals = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Path Tracer Debug")
    bool ShowDepth = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Path Tracer Debug")
    bool ShowAlbedo = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Path Tracer Debug")
    bool ShowLightContribution = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Path Tracer Debug")
    bool ShowSampleCount = false;

    UFUNCTION(BlueprintCallable, Category = "Path Tracer")
    void RenderSceneProgressive();

    UFUNCTION(BlueprintCallable, Category = "Path Tracer")
    void ResetFrameCounter();

    virtual void BeginPlay() override;
    virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

private:
    void SetupCamera();
    void GatherPointLights();
    void UpdateBatchSize(float PathTracingTime);
    const UPathTracerMaterialComponent* GetMaterialProperties(const FHitResult& Hit);
    PathTracingRay GetCameraRay(float U, float V);
    FHitResult TraceRay(const PathTracingRay& Ray);
    void ResetMaps();
    void ResetRendering();

    FLinearColor TracePixel(const PathTracingRay& Ray, int32 Depth, float& OutHitDistance);
    FLinearColor CalculateDirectLighting(const FVector& Position, const FVector& Normal, const FVector& ViewDirection, const UPathTracerMaterialComponent* Material);
    FLinearColor CalculateIndirectLighting(const FHitResult& Hit, const FVector& Normal, const PathTracingRay& Ray, const UPathTracerMaterialComponent* Material, int32 Depth);

    FLinearColor CalculatePointLightContribution(const FVector& Position, const FVector& Normal, const FVector& ViewDirection, const UPathTracerMaterialComponent* Material, const UPointLightComponent* Light);
    bool IsInShadow(const FVector& Position, const UPointLightComponent* Light);
    float CalculateAttenuation(float Distance);
    FLinearColor CalculateDiffuseContribution(const UPathTracerMaterialComponent* Material, float NdotL);
    FLinearColor CalculateSpecularContribution(const FVector& Normal, const FVector& ViewDirection, const FVector& LightDirection, const UPathTracerMaterialComponent* Material);

    FLinearColor CalculateReflection(const FHitResult& Hit, const FVector& Normal, const PathTracingRay& Ray, int32 Depth);
    FLinearColor CalculateRefraction(const FHitResult& Hit, const FVector& Normal, const PathTracingRay& Ray, const UPathTracerMaterialComponent* Material, int32 Depth);
    FLinearColor CalculateDiffuseReflection(const FHitResult& Hit, const FVector& Normal, const UPathTracerMaterialComponent* Material, int32 Depth);

    FLinearColor CalculateEnvironmentLighting(const PathTracingRay& Ray);
    FVector SampleHemisphere(const FVector& Normal, float Roughness);
    float FresnelSchlick(float Cosine, float F0);
    FVector Refract(const FVector& Incident, const FVector& Normal, float EtaI, float EtaT);
    bool TotalInternalReflection(const FVector& Incident, const FVector& Normal, float EtaI, float EtaT);
    FVector CalculateInterpolatedNormal(UStaticMeshComponent* MeshComponent, const FHitResult& Hit);
    
    UTextureRenderTarget2D* _RenderTarget;
    UMaterialInstanceDynamic* _PathTracerMaterialInstance;

    TArray<FLinearColor> _AccumulatedLinearColor;
    TArray<FColor> _AccumulatedColor;
    TArray<int32> _SampleCounts;
    TArray<float> _PixelVariance;

    TArray<UPointLightComponent*> _PointLights;
    int32 _RayCount = 0;
    int64 _CurrentTotalSampleCount = 0;
    int64 _TargetTotalSampleCount = 0;
    PathTracingCamera _Camera;

    PathTracingPID _PID;
    int64 _FrameCount = 0;
};