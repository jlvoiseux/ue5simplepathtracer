#include "PathTracerComponent.h"
#include "Engine/World.h"
#include "Camera/CameraComponent.h"
#include "Kismet/GameplayStatics.h"


DEFINE_LOG_CATEGORY(LogPathTracer)

UPathTracerComponent::UPathTracerComponent()
{
    PrimaryComponentTick.bCanEverTick = true;
}

void UPathTracerComponent::BeginPlay()
{
    Super::BeginPlay();

    if (RenderTarget)
    {
        FRenderTarget* RT = RenderTarget->GameThread_GetRenderTargetResource();
        FIntPoint Size = RT->GetSizeXY();
        _AccumulatedLinearColor.SetNum(Size.X * Size.Y);
        _AccumulatedColor.SetNum(Size.X * Size.Y);
        _SampleCounts.SetNum(Size.X * Size.Y);
        _PID.MaxBatchSize = FMath::Min(RenderTarget->SizeX * RenderTarget->SizeY, _PID.MaxBatchSize);
        
        ResetRendering();

        UE_LOG(LogPathTracer, Log, TEXT("Initialized render target of size %dx%d"), Size.X, Size.Y);
    }
    else
    {
        UE_LOG(LogPathTracer, Error, TEXT("No render target"));
    }

    GatherPointLights();
}

void UPathTracerComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
    RenderSceneProgressive();
}

void UPathTracerComponent::GatherPointLights()
{
    TArray<AActor*> FoundLights;
    UGameplayStatics::GetAllActorsOfClass(GetWorld(), AActor::StaticClass(), FoundLights);

    for (AActor* Actor : FoundLights)
    {
        TArray<UPointLightComponent*> PointLightComponents;
        Actor->GetComponents<UPointLightComponent>(PointLightComponents);

        for (UPointLightComponent* PointLightComponent : PointLightComponents)
        {
            _PointLights.Add(PointLightComponent);
        }
    }

    UE_LOG(LogPathTracer, Log, TEXT("Collected %d lights"), _PointLights.Num());
}

void UPathTracerComponent::UpdateBatchSize(float PathTracingTime)
{
    _PID.MovingAverageTime = _PID.MovingAverageTime * 0.95f + PathTracingTime * 0.05f;

    float Error = FrameTimeBudget - _PID.MovingAverageTime;
    _PID.IntegralError += Error;
    float DerivativeError = Error - _PID.LastError;

    float Output = _PID.Kp * Error + _PID.Ki * _PID.IntegralError + _PID.Kd * DerivativeError;

    if (_PID.FrameCount >= _PID.WarmupFrames)
    {
        float AdjustmentFactor = FMath::Clamp(Output, -0.1f, 0.1f);
        _PID.CurrBatchSize = FMath::Clamp(
            FMath::RoundToInt64(_PID.CurrBatchSize * (1.0f + AdjustmentFactor)),
            _PID.MinBatchSize,
            _PID.MaxBatchSize
        );
    }

    _PID.LastError = Error;
    _PID.FrameCount++;
}

void UPathTracerComponent::RenderSceneProgressive()
{
    _RayCount = 0;

    if (!RenderTarget || _IsFrameComplete)
        return;

    FRenderTarget* RT = RenderTarget->GameThread_GetRenderTargetResource();
    FIntPoint Size = RT->GetSizeXY();

    TArray<FIntPoint> PixelsToProcess;
    PixelsToProcess.Reserve(_PID.CurrBatchSize);

    double StartTime = FPlatformTime::Seconds();
    {
        for (int32 i = 0; i < _PID.CurrBatchSize; ++i)
        {
            if (_CurrentY >= Size.Y)
            {
                if (_SampleCounts[Size.X * Size.Y - 1] >= SamplesPerPixel)
                {
                    _IsFrameComplete = true;
                    break;
                }
                _CurrentX = 0;
                _CurrentY = 0;
            }

            PixelsToProcess.Add(FIntPoint(_CurrentX, _CurrentY));

            _CurrentX++;
            if (_CurrentX >= Size.X)
            {
                _CurrentX = 0;
                _CurrentY++;
            }
        }

        ParallelFor(PixelsToProcess.Num(), [&](int32 Index)
            {
                FIntPoint Pixel = PixelsToProcess[Index];
                int32 PixelIndex = Pixel.Y * Size.X + Pixel.X;

                float U = (Pixel.X + FMath::FRand()) / (float)Size.X;
                float V = (Pixel.Y + FMath::FRand()) / (float)Size.Y;

                PathTracingRay CameraRay = GetCameraRay(U, V);
                FLinearColor SampleColor = TracePixel(CameraRay, 0);

                FCriticalSection CriticalSection;
                {
                    FScopeLock Lock(&CriticalSection);
                    _AccumulatedLinearColor[PixelIndex] += SampleColor;
                    _SampleCounts[PixelIndex]++;
                }

                FLinearColor AverageColor = _AccumulatedLinearColor[PixelIndex] / _SampleCounts[PixelIndex];
                _AccumulatedColor[PixelIndex] = AverageColor.ToFColor(false);

                FPlatformAtomics::InterlockedIncrement(&_RayCount);
            });
    }
    double EndTime = FPlatformTime::Seconds();

    double PathTracingTime = EndTime - StartTime;
    UpdateBatchSize(PathTracingTime);

    if (RT)
    {
        TArray<FColor>* ColorDataCopy = new TArray<FColor>(_AccumulatedColor);

        ENQUEUE_RENDER_COMMAND(UpdateTextureCommand)(
            [RT, ColorDataCopy, Size](FRHICommandListImmediate& RHICmdList)
            {
                FUpdateTextureRegion2D Region(0, 0, 0, 0, Size.X, Size.Y);
                RHIUpdateTexture2D(
                    RT->GetRenderTargetTexture(),
                    0,
                    Region,
                    Size.X * 4,
                    (uint8*)ColorDataCopy->GetData()
                );

                delete ColorDataCopy;
            });
    }

    double ProgressPercentage = (double)(_SampleCounts[Size.X * Size.Y - 1]) / SamplesPerPixel * 100.0f;
    UE_LOG(LogPathTracer, Log, TEXT("Progress %.2f%%, Pixel Batch Size %d, Path Tracing Time %.4f s, Moving Average %.4f s"), ProgressPercentage, _PID.CurrBatchSize, PathTracingTime, _PID.MovingAverageTime);
}

void UPathTracerComponent::ResetRendering()
{
    SetupCamera();
    if (RenderTarget)
    {
        FRenderTarget* RT = RenderTarget->GameThread_GetRenderTargetResource();
        FIntPoint Size = RT->GetSizeXY();
        for (int32 i = 0; i < _AccumulatedLinearColor.Num(); ++i)
        {
            _AccumulatedLinearColor[i] = FLinearColor::Black;
            _AccumulatedColor[i] = FColor::Black;
            _SampleCounts[i] = 0;
        }
    }
    _CurrentX = 0;
    _CurrentY = 0;
    _IsFrameComplete = false;

    UE_LOG(LogPathTracer, Log, TEXT("Rendering reset"));
}

void UPathTracerComponent::SetupCamera()
{
    APlayerCameraManager* CameraManager = UGameplayStatics::GetPlayerCameraManager(GetWorld(), 0);
    if (!CameraManager)
    {
        UE_LOG(LogPathTracer, Error, TEXT("Failed to get player camera manager"));
        return;
    }

    UCameraComponent* CameraComponent = CameraManager->GetViewTarget()->FindComponentByClass<UCameraComponent>();
    if (!CameraComponent)
    {
        UE_LOG(LogPathTracer, Error, TEXT("Failed to get player camera manager"));
        return;
    }

    _Camera.CameraLocation = CameraComponent->GetComponentLocation();
    _Camera.CameraRotation = CameraComponent->GetComponentRotation();

    _Camera.FOV = CameraComponent->FieldOfView;
    _Camera.AspectRatio = CameraComponent->AspectRatio;

    _Camera.Forward = _Camera.CameraRotation.Vector();
    _Camera.Right = FVector::CrossProduct(_Camera.Forward, FVector::UpVector);
    _Camera.Up = FVector::CrossProduct(_Camera.Right, _Camera.Forward);

    _Camera.HalfWidth = FMath::Tan(FMath::DegreesToRadians(_Camera.FOV * 0.5f));
    _Camera.HalfHeight = _Camera.HalfWidth / _Camera.AspectRatio;
}

UPathTracerComponent::PathTracingRay UPathTracerComponent::GetCameraRay(float U, float V)
{
    FVector RayDirection = _Camera.Forward + (1.0f - 2.0f * U) * _Camera.HalfWidth * _Camera.Right + (1.0f - 2.0f * V) * _Camera.HalfHeight * _Camera.Up;
    return { _Camera.CameraLocation, RayDirection.GetSafeNormal() };
}

FHitResult UPathTracerComponent::TraceRay(const PathTracingRay& Ray)
{
    FHitResult HitResult;
    FCollisionQueryParams QueryParams;
    QueryParams.bTraceComplex = true;

    GetWorld()->LineTraceSingleByChannel(HitResult, Ray.Origin, Ray.Origin + Ray.Direction * 100000.0f, ECC_Visibility, QueryParams);
    return HitResult;
}

FLinearColor UPathTracerComponent::TracePixel(const PathTracingRay& Ray, int32 Depth)
{
    TRACE_CPUPROFILER_EVENT_SCOPE(TEXT("PathTracer_TracePixel"));

    if (Depth >= MaxBounces)
        return FLinearColor::Black;

    FHitResult Hit = TraceRay(Ray);

    if (Hit.bBlockingHit)
    {
        if (ShowNormals)
            return FLinearColor(Hit.Normal * 0.5f + 0.5f);

        if (ShowDepth)
        {
            float NormalizedDepth = FMath::Clamp(Hit.Distance / 1000.0f, 0.0f, 1.0f);
            return FLinearColor(1.0f - NormalizedDepth, 1.0f - NormalizedDepth, 1.0f - NormalizedDepth);
        }

        FLinearColor Albedo = GetAlbedo(Hit);

        if (ShowAlbedo)
            return Albedo;

        if (ShowLightContribution)
            return CalculateLighting(Hit.Location, Hit.Normal, Albedo);

        FVector RandomDirection = FMath::VRandCone(Hit.Normal, 1.0f);
        PathTracingRay NewRay = { Hit.Location + Hit.Normal * 0.001f, RandomDirection };
        FLinearColor DirectLighting = CalculateLighting(Hit.Location, Hit.Normal, Albedo);
        FLinearColor IndirectLighting = TracePixel(NewRay, Depth + 1);

        return DirectLighting + Albedo * 0.5f * IndirectLighting;
    }

    float T = 0.5f * (Ray.Direction.Z + 1.0f);
    return FLinearColor(0.0f, 0.0f, 0.0f);
}

FLinearColor UPathTracerComponent::CalculateLighting(const FVector& Position, const FVector& Normal, const FLinearColor& Albedo)
{
    FLinearColor TotalLighting = FLinearColor::Black;

    for (const UPointLightComponent* Light : _PointLights)
    {
        FVector LightDirection = (Light->GetLightPosition() - Position).GetSafeNormal();
        float NdotL = FMath::Max(0.0f, FVector::DotProduct(Normal, LightDirection));

        float Distance = FVector::Distance(Light->GetLightPosition(), Position);
        float Attenuation = 1.0f / (1.0f + 0.1f * Distance + 0.01f * Distance * Distance);

        FLinearColor LightContribution = Light->GetLightColor() * Light->Intensity * NdotL * Attenuation;
        TotalLighting += LightContribution;
    }

    return Albedo * TotalLighting;
}

FLinearColor UPathTracerComponent::GetAlbedo(const FHitResult& Hit)
{
    FLinearColor Albedo = FLinearColor::White;
    if (Hit.GetActor())
    {
        UStaticMeshComponent* MeshComponent = Hit.GetActor()->FindComponentByClass<UStaticMeshComponent>();
        if (MeshComponent)
        {
            UMaterialInterface* Material = MeshComponent->GetMaterial(0);
            if (Material)
            {
                Material->GetVectorParameterValue(FHashedMaterialParameterInfo("BaseColor"), Albedo);
            }
        }
    }
    return Albedo;
}