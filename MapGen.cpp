// Fill out your copyright notice in the Description page of Project Settings.

#include "aimasa2.h"
#include "MapGen.h"



// Sets default values
AMapGen::AMapGen()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	FString problem = FString("problem_A3");
	readJson(problem);
	initFakeGroundPoints();
}

// Called when the game starts or when spawned
void AMapGen::BeginPlay()
{
	print("MapGen!", 20, FColor::Red);

	Super::BeginPlay();

	

	const UWorld * world = GetWorld();

	if (world) {
		FActorSpawnParameters spawnParams;
		spawnParams.Owner = this;
		spawnParams.Instigator = Instigator;

		for (int j = 0; j < allGroundPoints.Num(); ++j) {

			APolygon * newActor = GetWorld()->SpawnActor<APolygon>(allGroundPoints[j][0], FRotator::ZeroRotator, spawnParams);
			allPolygons.Add(newActor);
			newActor->init(allGroundPoints[j], FVector(0, 0, 0));

			newActor = GetWorld()->SpawnActor<APolygon>(allFakeGroundPoints[j][0], FRotator::ZeroRotator, spawnParams);
			allFakePolygons.Add(newActor);
			newActor->init(allFakeGroundPoints[j], FVector(0, 0, 0));

		}

		for (int j = 0; j < allWallPoints.Num(); ++j) {
			APolygon * newActor = GetWorld()->SpawnActor<APolygon>(allWallPoints[j][0], FRotator::ZeroRotator, spawnParams);
			newActor->init(allWallPoints[j], FVector(0, 0, 0));
			allWalls.Add(newActor);
		}

		for (int j = 0; j < startPoints.Num(); ++j) {
			cars.Add(GetWorld()->SpawnActor<ACar>(startPoints[j], FVector(1,0,0).Rotation(), spawnParams));
		}

		for (int j = 0; j < itemPoints.Num(); ++j) {
			items.Add(GetWorld()->SpawnActor<AItem>(itemPoints[j], FVector(1, 0, 0).Rotation(), spawnParams));
		}


		//car = GetWorld()->SpawnActor<ACar>(start_pos, start_vel.Rotation(), spawnParams);
		//goal = GetWorld()->SpawnActor<AGoal>(goal_pos, goal_vel.Rotation(), spawnParams);


		



	}

	
	

}

// Called every frame
void AMapGen::Tick( float DeltaTime )
{
	Super::Tick( DeltaTime );
}

void AMapGen::readJson(FString fileName)
{
	Nvertices = 0;

	FString FullPath = FPaths::GameDir() + "Data/" + fileName + ".json";
	FString JsonStr;
	FFileHelper::LoadFileToString(JsonStr, *FullPath);
	TSharedRef<TJsonReader<TCHAR>> JsonReader = FJsonStringReader::Create(JsonStr);
	TSharedPtr<FJsonObject> JsonObject = MakeShareable(new FJsonObject());
	bool serialized = FJsonSerializer::Deserialize(JsonReader, JsonObject);

	if (!serialized) {
		return;
	}

	int i = 0;
	while (true) {
		FString fieldName = FString("polygon") + FString::FromInt(i);
		if (!JsonObject->HasField(fieldName)) {
			break;
		}	

		TArray < TSharedPtr < FJsonValue > > coords = JsonObject->GetArrayField(fieldName);
		TArray<FVector> groundPoints;

		for (int j = 0; j < coords.Num(); ++j) {

			TSharedPtr < FJsonValue > test = coords[j];
			TArray <TSharedPtr < FJsonValue >> test2 = test->AsArray();
			TSharedPtr < FJsonValue> testX = test2[0];
			float x = (float)testX->AsNumber();

			TSharedPtr < FJsonValue> testY = test2[1];
			float y = (float)testY->AsNumber();

			groundPoints.Add(FVector(-x*scale, y*scale, default_Z));
			allPoints.Add(FVector(-x*scale, y*scale, default_Z));
			cornerPoints.Add(FVector(-x*scale, y*scale, default_Z));
			//GEngine->AddOnScreenDebugMessage(-1, 50.f, FColor::Cyan, FVector(-x*scale, y*scale, 0).ToString());//FString::SanitizeFloat(Hit.Distance));
			
			Nvertices++;
		}

		allGroundPoints.Add(groundPoints);

		++i;
	}

	//TArray<FVector> endPolygon;
	//endPolygon.Add(goal_pos);
	//allGroundPoints.Add(endPolygon);


	// wall points
	FString fieldName = FString("boundary_polygon");
	TArray < TSharedPtr < FJsonValue > > coords = JsonObject->GetArrayField(fieldName);

	for (int j = 0; j < coords.Num(); ++j) {
		TSharedPtr < FJsonValue > test = coords[j];
		TArray <TSharedPtr < FJsonValue >> test2 = test->AsArray();
		TSharedPtr < FJsonValue> testX = test2[0];
		float x = (float)testX->AsNumber();

		TSharedPtr < FJsonValue> testY = test2[1];
		float y = (float)testY->AsNumber();

		wallPoints.Add(FVector(-x*scale, y*scale, default_Z));
		allPoints.Add(FVector(-x*scale, y*scale, default_Z));

		Nvertices++;
	}

	for (int j = 0; j < wallPoints.Num() - 1; j++) {
		TArray<FVector> temp;
		temp.Add(wallPoints[j]);
		temp.Add(wallPoints[j + 1]);
		allWallPoints.Add(temp);
	}

	TArray<FVector> temp;
	temp.Add(wallPoints[wallPoints.Num() - 1]);
	temp.Add(wallPoints[0]);
	allWallPoints.Add(temp);

	//floats
	L_car = JsonObject->GetNumberField("L_car") * scale;
	a_max = JsonObject->GetNumberField("a_max") * scale;
	k_friction = JsonObject->GetNumberField("k_friction");
	omega_max = JsonObject->GetNumberField("omega_max");
	phi_max = JsonObject->GetNumberField("phi_max");
	v_max = JsonObject->GetNumberField("v_max") * scale;
	sensor_range = JsonObject->GetNumberField("sensor_range") * scale;

	// ASSIGNMENT 2

	//pos / velocity
	TArray <TSharedPtr < FJsonValue >> vals = JsonObject->GetArrayField("goal_pos");
	goalPoints.Add(FVector(-(float)vals[0]->AsNumber()*scale, (float)vals[1]->AsNumber()*scale, default_Z));

	vals = JsonObject->GetArrayField("start_pos");
	startPoints.Add(FVector(-(float)vals[0]->AsNumber()*scale, (float)vals[1]->AsNumber()*scale, default_Z));

	i = 1;
	while (true) {
		FString fieldNameStart = FString("start_pos") + FString::FromInt(i);
		FString fieldNameGoal = FString("goal_pos") + FString::FromInt(i);

		//print_log(fieldNameStart);
		//print_log(fieldNameGoal);
		//print_log("===");

		if (!JsonObject->HasField(fieldNameStart) || !JsonObject->HasField(fieldNameGoal)) {
			break;
		}

		vals = JsonObject->GetArrayField(fieldNameStart);
		startPoints.Add(FVector(-(float)vals[0]->AsNumber()*scale, (float)vals[1]->AsNumber()*scale, default_Z));

		vals = JsonObject->GetArrayField(fieldNameGoal);
		goalPoints.Add(FVector(-(float)vals[0]->AsNumber()*scale, (float)vals[1]->AsNumber()*scale, default_Z));

		++i;
	}
	//allPoints.Add(start_pos);

	TArray<FString> allKeys;
	JsonObject->Values.GetKeys(allKeys);

	for (int j = 0; j < allKeys.Num(); ++j) {
		if(!allKeys[j].Contains("item")) continue;
		vals = JsonObject->GetArrayField(allKeys[j]);
		itemPoints.Add(FVector(-(float)vals[0]->AsNumber()*scale, (float)vals[1]->AsNumber()*scale, default_Z));
		//print_log(itemPoints.Last());
	}

	//print_log(startPoints.Num());




}




bool AMapGen::Trace(FVector start, FVector end, int polyNum) {

	FCollisionQueryParams QParams = FCollisionQueryParams();
	FCollisionResponseParams RParams = FCollisionResponseParams();
	QParams.bTraceComplex = true;

	TArray<FHitResult> Hits;

	if (polyNum != -1) {
		QParams.AddIgnoredActor(allPolygons[polyNum]);
	}

	GetWorld()->LineTraceMultiByChannel(Hits, start+trace_offset, end+trace_offset, ECollisionChannel::ECC_GameTraceChannel1, QParams, RParams);
	
	
	float expected_dist = FVector::Dist(start, end);
	float first_hit_dist = 0;

	if (Hits.Num() == 0) {
		return true;
	} else {
		for (int i = 0; i < Hits.Num(); ++i) {
			float dist = Hits[i].Distance;
			if(dist == 0.0) continue;
			first_hit_dist = dist;
			break;
		}
	}

	if(first_hit_dist == 0.0) first_hit_dist = expected_dist;

	float dist_error = abs(first_hit_dist - expected_dist) / expected_dist;

	return dist_error < 0.1; // kanske fel


	
}

TArray<FVector> AMapGen::getPath(std::vector<PolyPoint> &path) {
	TArray<FVector> pathCoordinates;

	for (int i = 0; i < path.size(); i++) {
		pathCoordinates.Add(getPoint(path[i]));
	}

	return pathCoordinates;
}


void AMapGen::print(FString msg, float time, FColor color) {
	GEngine->AddOnScreenDebugMessage(-1, time, color, msg);
}



FVector AMapGen::getPoint(PolyPoint pp) {
	return allGroundPoints[pp.polygon_index][pp.point_index];	
}

void AMapGen::initFakeGroundPoints() {
	allFakeGroundPoints = allGroundPoints;

	for (int i = 0; i < allGroundPoints.Num(); ++i) {

		FVector midpoint(0,0,0);
		for (int j = 0; j < allGroundPoints[i].Num(); ++j) {
			midpoint = midpoint + allGroundPoints[i][j];
		}
		midpoint = midpoint / allGroundPoints[i].Num();

		for (int j = 0; j < allGroundPoints[i].Num(); ++j) {
			allFakeGroundPoints[i][j] = allGroundPoints[i][j] + (midpoint - allGroundPoints[i][j]) * 0.01;
		}
	}
}


void AMapGen::drawLine(FVector from, FVector to, FColor color, FVector z_offset) {
	DrawDebugLine(GetWorld(), from + z_offset, to + z_offset, color, true, -1, 0, 10);
}














