// Fill out your copyright notice in the Description page of Project Settings.

#include "aimasa2.h"
#include "Polygon.h"



// Sets default values
APolygon::APolygon()
{
	//GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Cyan, "new polygon");
	mesh = CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("GeneratedMesh"));
	RootComponent = mesh;

	static ConstructorHelpers::FObjectFinder<UMaterial> mat(TEXT("Material'/Game/StarterContent/Materials/M_MWhite.M_MWhite'"));
	if (mat.Succeeded()) {
		mesh->SetMaterial(1, mat.Object);
	}
}

// Called when the game starts or when spawned
void APolygon::BeginPlay()
{
	Super::BeginPlay();
	
}

// Called every frame
void APolygon::Tick( float DeltaTime )
{
	Super::Tick( DeltaTime );

}

void APolygon::init(TArray<FVector> groundVertices, FVector newPosition) {
	TArray<FVector> vertices;
	TArray<FColor> vertexColors;

	for (int i = 0; i < groundVertices.Num(); ++i) {
		FVector temp = groundVertices[i];
		temp.Z = 0;

		vertices.Add(groundVertices[i]);
		vertexColors.Add(FColor(255.0, 0.00, 0.0, 1.0));

		
		temp.Z = 100;
		vertices.Add(temp);
		vertexColors.Add(FColor(255.0, 0.00, 0.0, 1.0)); // does nothing?
	}

	TArray<int32> Triangles;

	// floor
	for (int i = 1; i < vertices.Num()/2 - 1; ++i) {
		Triangles.Add(0);
		Triangles.Add(i*2);
		Triangles.Add(i*2 + 2);

		Triangles.Add(i*2 + 2);
		Triangles.Add(i*2);
		Triangles.Add(0);		
	}

	// ceiling
	for (int i = 1; i < vertices.Num()/2 - 1; ++i) {
		Triangles.Add(1);
		Triangles.Add(i*2+1);
		Triangles.Add(i*2 + 1 + 2);

		Triangles.Add(i*2 + 1 + 2);
		Triangles.Add(i*2 + 1);
		Triangles.Add(1);
	}

	int n = vertices.Num();
	// walls
	for (int i = 0; i < n - 2; ++i) {
		Triangles.Add(i);
		Triangles.Add(i + 1);
		Triangles.Add(i + 2);

		Triangles.Add(i + 2);
		Triangles.Add(i + 1);
		Triangles.Add(i);
	}

	Triangles.Add(n - 2);
	Triangles.Add(n - 1);
	Triangles.Add(0);

	Triangles.Add(0);
	Triangles.Add(n - 1);
	Triangles.Add(n - 2);

	Triangles.Add(n - 1);
	Triangles.Add(0);
	Triangles.Add(1);

	Triangles.Add(1);
	Triangles.Add(0);
	Triangles.Add(n - 1);

	TArray<FVector> normals;
	normals.AddUninitialized(vertices.Num());

	TArray<FVector2D> UV0;
	UV0.AddUninitialized(vertices.Num());

	for (int i = 0; i < UV0.Num(); ++i) {
		if (i % 3 == 0) {
			UV0[i].X = 0.0;
			UV0[i].Y = 0.0;
		} else if (i % 3 == 1) {
			UV0[i].X = 1.0;
			UV0[i].Y = 0.0;
		} else {
			UV0[i].X = 0.5;
			UV0[i].Y = 0.5;
		}
		
	}

	TArray<FProcMeshTangent> tangents;
	tangents.AddUninitialized(vertices.Num());


	mesh->CreateMeshSection(1, vertices, Triangles, normals, UV0, vertexColors, tangents, true);
	mesh->SetCollisionResponseToChannel(ECollisionChannel::ECC_GameTraceChannel1,ECollisionResponse::ECR_Overlap);
	//mesh->SetCollisionResponseToAllChannels(ECollisionResponse::ECR_Overlap);
	mesh->CastShadow = false;
	
	SetActorLocation(newPosition, false);
}

UPrimitiveComponent* APolygon::getPrimComponent() { return mesh; }