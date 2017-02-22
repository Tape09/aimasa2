// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "GameFramework/Actor.h"
#include "ProceduralMeshComponent.h"
#include "MyMath.h"
#include "Polygon.generated.h"

UCLASS()
class AIMASA2_API APolygon : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	APolygon();

	// Called when the game starts or when spawned
	virtual void BeginPlay() override;
	
	// Called every frame
	virtual void Tick( float DeltaSeconds ) override;

	void init(TArray<FVector> groundVertices, FVector newPosition);

	UPrimitiveComponent* getPrimComponent();

private:
	UPROPERTY(VisibleAnywhere, Category = Materials)
	UProceduralMeshComponent * mesh;
};
