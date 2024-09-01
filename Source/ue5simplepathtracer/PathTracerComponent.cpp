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
        _PixelVariance.SetNum(Size.X * Size.Y);
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

const UPathTracerMaterialComponent* UPathTracerComponent::GetMaterialProperties(const FHitResult& Hit)
{
    if (Hit.GetActor())
    {
        const UPathTracerMaterialComponent* MaterialComponent = Hit.GetActor()->FindComponentByClass<UPathTracerMaterialComponent>();
        if (MaterialComponent)
        {
            return MaterialComponent;
        }
    }
    return nullptr;
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

            if (_SampleCounts[_CurrentY * Size.X + _CurrentX] < SamplesPerPixel)
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

                int32 SamplesToTake = FMath::Max(1, FMath::RoundToInt(_PixelVariance[PixelIndex] * 10));
                FLinearColor PixelColor = FLinearColor::Black;
                for (int32 s = 0; s < 1; ++s)
                {
                    float U = (Pixel.X + FMath::FRand()) / (float)Size.X;
                    float V = (Pixel.Y + FMath::FRand()) / (float)Size.Y;
                    PathTracingRay CameraRay = GetCameraRay(U, V);
                    float outHitDistance;
                    PixelColor += TracePixel(CameraRay, 0, outHitDistance);
                }
                PixelColor /= SamplesToTake;

                FCriticalSection CriticalSection;
                {
                    FScopeLock Lock(&CriticalSection);
                    FLinearColor OldMean = _AccumulatedLinearColor[PixelIndex] / _SampleCounts[PixelIndex];
                    _AccumulatedLinearColor[PixelIndex] += PixelColor;
                    _SampleCounts[PixelIndex] += SamplesToTake;
                    FLinearColor NewMean = _AccumulatedLinearColor[PixelIndex] / _SampleCounts[PixelIndex];

                    _PixelVariance[PixelIndex] = FVector::DistSquared((FVector)OldMean, (FVector)NewMean);
                    _AccumulatedColor[PixelIndex] = NewMean.ToFColor(false);
                }

                FPlatformAtomics::InterlockedAdd(&_RayCount, SamplesToTake);
            });
    }
    double EndTime = FPlatformTime::Seconds();

    double PathTracingTime = EndTime - StartTime;
    UpdateBatchSize(PathTracingTime);

    if (RT)
    {
        
        if (ShowSampleCount)
        {
            TArray<uint32>* ColorDataCopy = new TArray<uint32>(_SampleCounts);
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
        else
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
    QueryParams.bReturnFaceIndex = true;

    GetWorld()->LineTraceSingleByChannel(HitResult, Ray.Origin, Ray.Origin + Ray.Direction * 100000.0f, ECC_Visibility, QueryParams);

    return HitResult;
}

FLinearColor UPathTracerComponent::TracePixel(const PathTracingRay& Ray, int32 Depth, float& OutHitDistance)
{
    TRACE_CPUPROFILER_EVENT_SCOPE(TEXT("PathTracer_TracePixel"));

    if (Depth >= MaxBounces)
        return FLinearColor::Black;

    if (Depth >= RussianRouletteStartDepth)
    {
        if (FMath::FRand() > RussianRouletteProbability)
            return FLinearColor::Black;
    }

    FHitResult Hit = TraceRay(Ray);
    OutHitDistance = Hit.Distance;

    if (Hit.bBlockingHit)
    {
        FVector HitNormal = Hit.Normal;
        const UPathTracerMaterialComponent* Material = GetMaterialProperties(Hit);

        if (!Material)
            return FLinearColor::Black;

        if (Material->InterpolateNormals)
        {
            UStaticMeshComponent* MeshComponent = Cast<UStaticMeshComponent>(Hit.GetComponent());
            if (MeshComponent)
            {
                HitNormal = CalculateInterpolatedNormal(MeshComponent, Hit);
            }
        }

        if (ShowNormals)
            return FLinearColor(HitNormal * 0.5f + 0.5f);

        if (ShowDepth)
        {
            float NormalizedDepth = FMath::Clamp(Hit.Distance / 1000.0f, 0.0f, 1.0f);
            return FLinearColor(1.0f - NormalizedDepth, 1.0f - NormalizedDepth, 1.0f - NormalizedDepth);
        }

        if (ShowAlbedo)
            return Material->Albedo;

        FVector ViewDirection = -Ray.Direction;

        FLinearColor DirectLighting = CalculateDirectLighting(Hit.Location, HitNormal, ViewDirection, Material);

        if (ShowLightContribution)
            return DirectLighting;

        FLinearColor IndirectLighting = CalculateIndirectLighting(Hit, HitNormal, Ray, Material, Depth);

        if (Depth >= RussianRouletteStartDepth)
        {
            IndirectLighting /= RussianRouletteProbability;
        }

        FLinearColor FinalColor = DirectLighting + IndirectLighting;
        const float MaxLuminance = 100.0f;
        float Luminance = FinalColor.GetLuminance();
        if (Luminance > MaxLuminance)
        {
            FinalColor *= MaxLuminance / Luminance;
        }

        return FinalColor;
    }

    return CalculateEnvironmentLighting(Ray);
}

FLinearColor UPathTracerComponent::CalculateDirectLighting(const FVector& Position, const FVector& Normal, const FVector& ViewDirection, const UPathTracerMaterialComponent* Material)
{
    FLinearColor TotalDirectLighting = FLinearColor::Black;

    for (const UPointLightComponent* Light : _PointLights)
    {
        TotalDirectLighting += CalculatePointLightContribution(Position, Normal, ViewDirection, Material, Light);
    }

    return TotalDirectLighting;
}

FLinearColor UPathTracerComponent::CalculatePointLightContribution(const FVector& Position, const FVector& Normal, const FVector& ViewDirection, const UPathTracerMaterialComponent* Material, const UPointLightComponent* Light)
{
    FVector LightDirection = (Light->GetLightPosition() - Position).GetSafeNormal();
    float NdotL = FMath::Max(0.0f, FVector::DotProduct(Normal, LightDirection));

    if (IsInShadow(Position, Light))
        return FLinearColor::Black;

    float Distance = FVector::Distance(Light->GetLightPosition(), Position);
    float Attenuation = CalculateAttenuation(Distance);

    FLinearColor DiffuseContribution = CalculateDiffuseContribution(Material, NdotL);
    FLinearColor SpecularContribution = CalculateSpecularContribution(Normal, ViewDirection, LightDirection, Material);

    return (DiffuseContribution + SpecularContribution) * Light->GetLightColor() * Light->Intensity * Attenuation;
}

bool UPathTracerComponent::IsInShadow(const FVector& Position, const UPointLightComponent* Light)
{
    FHitResult ShadowHit;
    FVector ShadowRayOrigin = Position + FVector::UpVector * 0.001f; // Offset to avoid self-intersection
    FVector ShadowRayEnd = Light->GetLightPosition();
    GetWorld()->LineTraceSingleByChannel(ShadowHit, ShadowRayOrigin, ShadowRayEnd, ECC_Visibility);

    return ShadowHit.bBlockingHit && ShadowHit.GetActor() != Light->GetOwner();
}

float UPathTracerComponent::CalculateAttenuation(float Distance)
{
    return 1.0f / (1.0f + 0.1f * Distance + 0.01f * Distance * Distance);
}

FLinearColor UPathTracerComponent::CalculateDiffuseContribution(const UPathTracerMaterialComponent* Material, float NdotL)
{
    return Material->Albedo * NdotL;
}

FLinearColor UPathTracerComponent::CalculateSpecularContribution(const FVector& Normal, const FVector& ViewDirection, const FVector& LightDirection, const UPathTracerMaterialComponent* Material)
{
    FVector HalfVector = (ViewDirection + LightDirection).GetSafeNormal();
    float NdotH = FMath::Max(0.0f, FVector::DotProduct(Normal, HalfVector));
    float SpecularPower = FMath::Pow(2.0f, (1.0f - Material->Roughness) * 11.0f + 1.0f);
    return FLinearColor::White * FMath::Pow(NdotH, SpecularPower) * Material->Specular;
}

FLinearColor UPathTracerComponent::CalculateIndirectLighting(const FHitResult& Hit, const FVector& Normal, const PathTracingRay& Ray, const UPathTracerMaterialComponent* Material, int32 Depth)
{
    FLinearColor IndirectLighting = FLinearColor::Black;
    FVector ViewDirection = -Ray.Direction;

    float CosTheta = FMath::Max(FVector::DotProduct(Normal, ViewDirection), 0.0f);
    float FresnelFactor = FresnelSchlick(CosTheta, Material->Specular);

    FLinearColor ReflectionColor = CalculateReflection(Hit, Normal, Ray, Depth);
    FLinearColor RefractionColor = CalculateRefraction(Hit, Normal, Ray, Material, Depth);
    FLinearColor DiffuseColor = CalculateDiffuseReflection(Hit, Normal, Material, Depth);

    float MetallicFactor = Material->Metallic;
    float DiffuseFactor = 1.0f - MetallicFactor;

    IndirectLighting = (ReflectionColor * FresnelFactor * MetallicFactor) +
        (RefractionColor * Material->Translucency) +
        (DiffuseColor * Material->Albedo * DiffuseFactor * (1.0f - Material->Translucency));

    // Apply emission
    IndirectLighting += Material->Albedo * Material->Emission;

    return IndirectLighting;
}

FLinearColor UPathTracerComponent::CalculateReflection(const FHitResult& Hit, const FVector& Normal, const PathTracingRay& Ray, int32 Depth)
{
    FVector ReflectionDirection = Ray.Direction - 2 * FVector::DotProduct(Ray.Direction, Normal) * Normal;
    PathTracingRay ReflectionRay = { Hit.Location + Normal * 0.001f, ReflectionDirection };
    float ReflectionHitDistance;
    return TracePixel(ReflectionRay, Depth + 1, ReflectionHitDistance);
}

FLinearColor UPathTracerComponent::CalculateRefraction(const FHitResult& Hit, const FVector& Normal, const PathTracingRay& Ray, const UPathTracerMaterialComponent* Material, int32 Depth)
{
    if (Material->Translucency <= 0.0f)
        return FLinearColor::Black;

    float EtaI = 1.0f; // Air
    float EtaT = Material->IndexOfRefraction;
    FVector TranslucencyNormal = Normal;

    if (FVector::DotProduct(Ray.Direction, Normal) > 0.0f)
    {
        // Ray is exiting the material
        Swap(EtaI, EtaT);
        TranslucencyNormal = -Normal;
    }

    if (!TotalInternalReflection(Ray.Direction, TranslucencyNormal, EtaI, EtaT))
    {
        FVector RefractionDirection = Refract(Ray.Direction, TranslucencyNormal, EtaI, EtaT);
        PathTracingRay RefractionRay = { Hit.Location - TranslucencyNormal * 0.001f, RefractionDirection };
        float RefractionHitDistance;
        return TracePixel(RefractionRay, Depth + 1, RefractionHitDistance);
    }
    else
    {
        // Total internal reflection, use reflection
        return CalculateReflection(Hit, Normal, Ray, Depth);
    }
}

FLinearColor UPathTracerComponent::CalculateDiffuseReflection(const FHitResult& Hit, const FVector& Normal, const UPathTracerMaterialComponent* Material, int32 Depth)
{
    FVector DiffuseDirection = SampleHemisphere(Normal, Material->Roughness);
    PathTracingRay DiffuseRay = { Hit.Location + Normal * 0.001f, DiffuseDirection };
    float DiffuseHitDistance;
    return TracePixel(DiffuseRay, Depth + 1, DiffuseHitDistance);
}

FLinearColor UPathTracerComponent::CalculateEnvironmentLighting(const PathTracingRay& Ray)
{
    // Simple sky color
    float T = 0.5f * (Ray.Direction.Z + 1.0f);
    return FLinearColor::LerpUsingHSV(FLinearColor(1.0f, 1.0f, 1.0f), FLinearColor(0.5f, 0.7f, 1.0f), T);
}

FVector UPathTracerComponent::Refract(const FVector& Incident, const FVector& Normal, float EtaI, float EtaT)
{
    float Eta = EtaI / EtaT;
    float CosI = -FVector::DotProduct(Normal, Incident);
    float SinT2 = Eta * Eta * (1.0f - CosI * CosI);

    if (SinT2 > 1.0f)
        return Incident - 2.0f * FVector::DotProduct(Incident, Normal) * Normal; // Total internal reflection

    float CosT = FMath::Sqrt(1.0f - SinT2);
    return Eta * Incident + (Eta * CosI - CosT) * Normal;
}

bool UPathTracerComponent::TotalInternalReflection(const FVector& Incident, const FVector& Normal, float EtaI, float EtaT)
{
    float Eta = EtaI / EtaT;
    float CosI = -FVector::DotProduct(Normal, Incident);
    float SinT2 = Eta * Eta * (1.0f - CosI * CosI);
    return SinT2 > 1.0f;
}

FVector UPathTracerComponent::SampleHemisphere(const FVector& Normal, float Roughness)
{
    float Phi = 2.0f * PI * FMath::FRand();
    float CosTheta = FMath::Pow(FMath::FRand(), 1.0f / (Roughness + 1.0f));
    float SinTheta = FMath::Sqrt(1.0f - CosTheta * CosTheta);

    FVector TangentSpace = FVector(SinTheta * FMath::Cos(Phi), SinTheta * FMath::Sin(Phi), CosTheta);
    FVector Tangent, Bitangent;
    Normal.FindBestAxisVectors(Tangent, Bitangent);

    return TangentSpace.X * Tangent + TangentSpace.Y * Bitangent + TangentSpace.Z * Normal;
}

float UPathTracerComponent::FresnelSchlick(float Cosine, float F0)
{
    return F0 + (1.0f - F0) * FMath::Pow(1.0f - Cosine, 5.0f);
}

FVector UPathTracerComponent::CalculateInterpolatedNormal(UStaticMeshComponent* MeshComponent, const FHitResult& Hit)
{
    if (!MeshComponent || !MeshComponent->GetStaticMesh())
        return Hit.Normal;

    UStaticMesh* Mesh = MeshComponent->GetStaticMesh();
    FStaticMeshLODResources& LODModel = Mesh->GetRenderData()->LODResources[0];

    int32 FaceIndex = Hit.FaceIndex;

    if (FaceIndex < 0)
        return Hit.Normal;

    FIndexArrayView Indices = LODModel.IndexBuffer.GetArrayView();

    // Get the triangle vertices
    int32 TriangleIndex = Hit.FaceIndex * 3;
    FVector Vertices[3];
    FVector Normals[3];
    for (int32 i = 0; i < 3; ++i)
    {
        int32 VertexIndex = Indices[TriangleIndex + i];
        Vertices[i] = (FVector)LODModel.VertexBuffers.PositionVertexBuffer.VertexPosition(VertexIndex);
        Normals[i] = (FVector)LODModel.VertexBuffers.StaticMeshVertexBuffer.VertexTangentZ(VertexIndex);
    }

    // Calculate barycentric coordinates
    FVector LocalHitLocation = MeshComponent->GetComponentTransform().InverseTransformPosition(Hit.Location);
    FVector BarycentricCoord = FMath::ComputeBaryCentric2D(LocalHitLocation, Vertices[0], Vertices[1], Vertices[2]);

    // Interpolate normals using barycentric coordinates
    FVector InterpolatedNormal =
        Normals[0] * BarycentricCoord.X +
        Normals[1] * BarycentricCoord.Y +
        Normals[2] * BarycentricCoord.Z;

    // Transform the normal to world space
    InterpolatedNormal = MeshComponent->GetComponentTransform().TransformVector(InterpolatedNormal);
    InterpolatedNormal.Normalize();

    return InterpolatedNormal;
}