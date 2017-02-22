// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "GameFramework/Actor.h"
#include "Car.generated.h"

UCLASS()
class AIMASA2_API ACar : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	ACar();

	// Called when the game starts or when spawned
	virtual void BeginPlay() override;
	
	// Called every frame
	virtual void Tick( float DeltaSeconds ) override;


	UPROPERTY(EditAnywhere)
	UParticleSystemComponent *OurParticleSystem1;
	UPROPERTY(EditAnywhere)
	UParticleSystemComponent *OurParticleSystem2;


	FVector position;
	FVector velocity;
	FVector acceleration;

};
