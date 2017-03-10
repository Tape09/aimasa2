// Fill out your copyright notice in the Description page of Project Settings.

#include "aimasa2.h"
#include "Car.h"


// Sets default values
ACar::ACar()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	USphereComponent* SphereComponent = CreateDefaultSubobject<USphereComponent>(TEXT("RootComponent"));
	RootComponent = SphereComponent;
	SphereComponent->InitSphereRadius(10.0f);
	SphereComponent->SetCollisionProfileName(TEXT("Pawn"));
	SphereComponent->SetCollisionResponseToChannel(ECollisionChannel::ECC_GameTraceChannel1, ECollisionResponse::ECR_Ignore);

	UStaticMeshComponent* SphereVisual = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("VisualRepresentation"));
	SphereVisual->SetupAttachment(RootComponent);
	static ConstructorHelpers::FObjectFinder<UStaticMesh> SphereVisualAsset(TEXT("/Game/StarterContent/Props/SM_Chair.SM_Chair"));
	if (SphereVisualAsset.Succeeded()) {
		SphereVisual->SetStaticMesh(SphereVisualAsset.Object);
		SphereVisual->SetRelativeRotation(FRotator(0, 0,0));
		SphereVisual->SetRelativeLocation(FVector(0.0f, 0.0f, 0.0f));
		SphereVisual->SetWorldScale3D(FVector(0.5f));
	}
	SphereVisual->SetCollisionResponseToChannel(ECollisionChannel::ECC_GameTraceChannel1, ECollisionResponse::ECR_Ignore);

	//// Create a particle system that we can activate or deactivate
	//OurParticleSystem1 = CreateDefaultSubobject<UParticleSystemComponent>(TEXT("MovementParticles"));
	//OurParticleSystem1->SetupAttachment(SphereVisual);
	//OurParticleSystem1->bAutoActivate = true;
	//OurParticleSystem1->SetRelativeLocation(FVector(-0.0f, 0.0f, 0.0f));
	//static ConstructorHelpers::FObjectFinder<UParticleSystem> ParticleAsset(TEXT("/Game/StarterContent/Particles/P_Fire.P_Fire"));
	//if (ParticleAsset.Succeeded()) {

	//	//ParticleAsset.Object->Emitters[2]->ParticleSize = 1;

	//	OurParticleSystem1->SetTemplate(ParticleAsset.Object);

	//}

	// Create a particle system that we can activate or deactivate
	OurParticleSystem2 = CreateDefaultSubobject<UParticleSystemComponent>(TEXT("MovementParticles2"));
	OurParticleSystem2->SetupAttachment(SphereVisual);
	OurParticleSystem2->bAutoActivate = true;
	OurParticleSystem2->SetRelativeLocation(FVector(-0.0f, 0.0f, 0.0f));
	static ConstructorHelpers::FObjectFinder<UParticleSystem> ParticleAsset2(TEXT("/Game/StarterContent/Particles/P_Sparks.P_Sparks"));
	if (ParticleAsset2.Succeeded()) {
		OurParticleSystem2->SetTemplate(ParticleAsset2.Object);
	}
	OurParticleSystem2->SetCollisionResponseToChannel(ECollisionChannel::ECC_GameTraceChannel1, ECollisionResponse::ECR_Ignore);
	

	



}

// Called when the game starts or when spawned
void ACar::BeginPlay()
{
	Super::BeginPlay();
	
}

// Called every frame
void ACar::Tick( float DeltaTime )
{
	Super::Tick( DeltaTime );

	FVector CurrentLocation = this->GetActorLocation();

	//SetActorLocation ( CurrentLocation + FVector(-10,0,0) );



}

