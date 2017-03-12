// Fill out your copyright notice in the Description page of Project Settings.

#include "aimasa2.h"
#include "Car2.h"


// Sets default values
ACar2::ACar2()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.bCanEverTick = true;

	USphereComponent* SphereComponent = CreateDefaultSubobject<USphereComponent>(TEXT("RootComponent"));
	RootComponent = SphereComponent;
	SphereComponent->InitSphereRadius(10.0f);
	SphereComponent->SetCollisionProfileName(TEXT("Pawn"));
	SphereComponent->SetCollisionResponseToChannel(ECollisionChannel::ECC_GameTraceChannel1, ECollisionResponse::ECR_Ignore);

	UStaticMeshComponent* SphereVisual = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("VisualRepresentation"));
	SphereVisual->SetupAttachment(RootComponent);
	static ConstructorHelpers::FObjectFinder<UStaticMesh> SphereVisualAsset(TEXT("StaticMesh'/Game/StarterContent/Shapes/Shape_Torus.Shape_Torus'"));
	if (SphereVisualAsset.Succeeded()) {
		SphereVisual->SetStaticMesh(SphereVisualAsset.Object);
		SphereVisual->SetRelativeRotation(FRotator(0, 0, 0));
		SphereVisual->SetRelativeLocation(FVector(0.0f, 0.0f, 0.0f));
		SphereVisual->SetWorldScale3D(FVector(1.0f));
	}
	static ConstructorHelpers::FObjectFinder<UMaterial> mat(TEXT("Material'/Game/StarterContent/Materials/M_Basic_Wall4.M_Basic_Wall4'"));
	if (mat.Succeeded()) {
		SphereVisual->SetMaterial(0, mat.Object);
	}
	
	
	SphereVisual->SetCollisionResponseToChannel(ECollisionChannel::ECC_GameTraceChannel1, ECollisionResponse::ECR_Ignore);



	UStaticMeshComponent* SphereVisual2 = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("VisualRepresentation2"));
	SphereVisual2->SetupAttachment(RootComponent);
	static ConstructorHelpers::FObjectFinder<UStaticMesh> SphereVisualAsset2(TEXT("StaticMesh'/Game/StarterContent/Shapes/Shape_NarrowCapsule.Shape_NarrowCapsule'"));
	if (SphereVisualAsset2.Succeeded()) {
		SphereVisual2->SetStaticMesh(SphereVisualAsset2.Object);
		SphereVisual2->SetRelativeRotation(FRotator(0, 0, 0));
		SphereVisual2->SetRelativeLocation(FVector(0.0f, 0.0f, 0.0f));
		SphereVisual2->SetWorldScale3D(FVector(1.0f,1.0f,0.5f));
	}

	static ConstructorHelpers::FObjectFinder<UMaterial> mat2(TEXT("Material'/Game/StarterContent/Materials/M_Basic_Wall3.M_Basic_Wall3'"));
	if (mat2.Succeeded()) {
		SphereVisual2->SetMaterial(0, mat2.Object);
	}
	SphereVisual2->SetCollisionResponseToChannel(ECollisionChannel::ECC_GameTraceChannel1, ECollisionResponse::ECR_Ignore);



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
	OurParticleSystem2->SetWorldScale3D(FVector(3.0f, 3.0f, -3.0f));
	static ConstructorHelpers::FObjectFinder<UParticleSystem> ParticleAsset2(TEXT("/Game/StarterContent/Particles/P_Sparks.P_Sparks"));
	if (ParticleAsset2.Succeeded()) {
		OurParticleSystem2->SetTemplate(ParticleAsset2.Object);
	}
	OurParticleSystem2->SetCollisionResponseToChannel(ECollisionChannel::ECC_GameTraceChannel1, ECollisionResponse::ECR_Ignore);


}

// Called when the game starts or when spawned
void ACar2::BeginPlay()
{
	Super::BeginPlay();
	
}

// Called every frame
void ACar2::Tick( float DeltaTime )
{
	Super::Tick( DeltaTime );

}

