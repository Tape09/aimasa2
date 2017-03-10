// Fill out your copyright notice in the Description page of Project Settings.

#include "aimasa2.h"
#include "Item.h"


// Sets default values
AItem::AItem()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
	USphereComponent* SphereComponent = CreateDefaultSubobject<USphereComponent>(TEXT("RootComponent"));
	RootComponent = SphereComponent;
	SphereComponent->InitSphereRadius(10.0f);
	SphereComponent->SetCollisionProfileName(TEXT("Pawn"));
	SphereComponent->SetCollisionResponseToChannel(ECollisionChannel::ECC_GameTraceChannel1, ECollisionResponse::ECR_Ignore);
	SphereComponent->SetCollisionResponseToAllChannels(ECollisionResponse::ECR_Ignore);

	UStaticMeshComponent* SphereVisual = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("VisualRepresentation"));
	SphereVisual->SetupAttachment(RootComponent);
	static ConstructorHelpers::FObjectFinder<UStaticMesh> SphereVisualAsset(TEXT("/Game/StarterContent/Props/SM_Bush.SM_Bush"));
	if (SphereVisualAsset.Succeeded()) {
		SphereVisual->SetStaticMesh(SphereVisualAsset.Object);
		SphereVisual->SetRelativeRotation(FRotator(0, 180, 0));
		SphereVisual->SetRelativeLocation(FVector(0.0f, 0.0f, -10.0f));
		SphereVisual->SetWorldScale3D(FVector(0.5f));
	}
	SphereVisual->SetCollisionResponseToChannel(ECollisionChannel::ECC_GameTraceChannel1, ECollisionResponse::ECR_Ignore);
	SphereVisual->SetCollisionResponseToAllChannels(ECollisionResponse::ECR_Ignore);

	//// Create a particle system that we can activate or deactivate
	//OurParticleSystem1 = CreateDefaultSubobject<UParticleSystemComponent>(TEXT("MovementParticles"));
	//OurParticleSystem1->SetupAttachment(SphereVisual);
	//OurParticleSystem1->bAutoActivate = true;
	//OurParticleSystem1->SetRelativeLocation(FVector(-0.0f, 0.0f, 50.0f));
	//static ConstructorHelpers::FObjectFinder<UParticleSystem> ParticleAsset(TEXT("/Game/StarterContent/Particles/P_Fire.P_Fire"));
	//if (ParticleAsset.Succeeded()) {
	//	OurParticleSystem1->SetTemplate(ParticleAsset.Object);
	//}

}

// Called when the game starts or when spawned
void AItem::BeginPlay()
{
	Super::BeginPlay();
	
}

// Called every frame
void AItem::Tick( float DeltaTime )
{
	Super::Tick( DeltaTime );

}

