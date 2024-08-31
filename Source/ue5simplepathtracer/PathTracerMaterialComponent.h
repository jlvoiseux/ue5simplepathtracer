// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "PathTracerMaterialComponent.generated.h"


UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class UE5SIMPLEPATHTRACER_API UPathTracerMaterialComponent : public UActorComponent
{
	GENERATED_BODY()

public:	
	UPathTracerMaterialComponent();		

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Path Tracing Material")
    FLinearColor Albedo = FLinearColor::White;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Path Tracing Material")
    float Metallic = 0.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Path Tracing Material")
    float Roughness = 0.5f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Path Tracing Material")
    float Specular = 0.5f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Path Tracing Material")
    float Emission = 0.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Path Tracing Material")
    bool InterpolateNormals = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Path Tracing Material|Translucency")
    float Translucency = 0.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Path Tracing Material|Translucency")
    float IndexOfRefraction = 1.0f;
};
