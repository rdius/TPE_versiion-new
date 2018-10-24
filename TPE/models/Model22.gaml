 /**
 *  Model22
 *  Author: rdius
 *  Description: pollutants dispersion simulation  by applying a directed wind. 
 */

model Model22

global {
//	file shape_file_roads <- file("../includes/SIG_simu/roads_gama.shp");
//	file shape_file_buildings <- file("../includes/SIG_simu/buildings_gama.shp");
//	file shape_file_bound <- file("../includes/SIG_simu/reperes.shp");
//	file shape_file_roads <- file("../includes/carrefour/road.shp");
//	file shape_file_buildings <- file("../includes/carrefour/building.shp");
//	
//	file shape_file_roads <- file("../includes/shape/road.shp");
//	file shape_file_buildings <- file("../includes/shape/building.shp");
//	file shape_file_node <- file("../includes/shape/noeuds.shp");
	
	
//	file shape_file_roads <- file("../includes/shape/road.shp");
//	file shape_file_buildings <- file("../includes/shape/building.shp");
//	file shape_file_node <- file("../includes/shape/noeuds.shp");
	
//	
//	file shape_file_roads <- file("../includes/data/road.shp");
//	file shape_file_buildings <- file("../includes/data/building.shp");
//	//file shape_file_bound <- file("../includes/data/reperes.shp");

//Chargement des fichiers shapes
	file shape_file_roads <- file("../includes/orig/road.shp");
	file shape_file_buildings <- file("../includes/orig/buildings.shp");
	file shape_file_node <- file("../includes/orig/node.shp");
	//definition des parametres globaux
	graph the_graph; 
	geometry shape <- envelope(shape_file_roads)-100;
	list<Wind> wind <- [];
	list<Wind> winds <- [];
	float a1;
	float a2;
	float a3;
	float a4;
	
	float b1;
	float b2;
	float b3;
	float b4;
	
	float c1;
	float c2;
	float c3;
	float c4;
	
	float d1;
	float d2;
	float d3;
	float d4;
	point goal <- {1 ,(rnd(z_max - 2) + 1)}; 
	int z_max parameter: 'Z max of the Environment' <- 400;  
	float maximal_speed parameter: 'Maximal speed' <- 15.0 min: 0.1 max: 15.0;
	float minimal_distance parameter: 'Minimal Distance' <- 10.0; 
	bool apply_separation <- true parameter: 'Apply Separation ?';   
	bool apply_goal <- true parameter: 'Follow Goal ?'; 
	bool apply_wind <- true parameter: 'Apply Wind ?';     
	point wind_vector <- {0,0,0}  parameter: 'Direction of the wind'; 
	int cohesion_factor parameter: 'Cohesion Factor' <- 100; 
	//float time;
	int nbGoalsAchived <- 0;
	int simultime <- 50;
	//initialisation des agents du systeme
	init {
		create noeud from:shape_file_node;
		create road from: shape_file_roads with:[type::string(get("type"))]{
        
		}	
		the_graph <- as_edge_graph(road) ;
		//the_graph<-  (as_driving_graph(road,noeud));
		
		
		create building from: shape_file_buildings /*with: [type:: string(read('building'))]*/ {

		}
//		create vehicule number: 300 {
//			speed <- 100 #km /#h ;
//			target <- any_location_in(one_of (road)) ;
//			location <- any_location_in(one_of(noeud));
//			//location <- any_location_in (one_of(building));
//			source <- location;
//			
//		if (flip((0.1)))
//			{
//				type <- "CAMION";
//				energy <- rnd(100.0)+2.0;
//			} else if (flip((0.3)))
//			{
//				type <- "CAR";
//				energy <- rnd(70.0)+3.0;
//			} else if(flip(0.2))
//			{
//				type <- "BUS";
//				energy <- rnd(80.0)+2.0;
//			} else if(flip(0.4)){
//				type <- "MOTOBIKE";
//				energy <- rnd(50.0)+2.0;
//			}
//		} 
	}
//fonction permettant de generer les vehicules
//arret de creation de vehicules lorsque le nobre sup a 350 
//ou le temps de simulation devint sup a 500 cycles
	reflex VehiculeCreator when:(length(vehicule)>=350 or time <=500)  {
		create vehicule number: 5 { //creer 5 vehicules par cycle
		//un vehicule creee est place sur la route
			location <- any_location_in(one_of(road));
			max_speed <- 160 °km/°h;
			vehicle_length <- 5.0 °m;
			right_side_driving <- true;
			proba_lane_change_up <- 0.1 + (rnd(500) / 500);
			proba_lane_change_down <- 0.5+ (rnd(500) / 500);
			//location <- one_of(intersection where empty(each.stop)).location;
			security_distance_coeff <- 5/9 * 3.6 * (1.5 - rnd(1000) / 1000);  
			proba_respect_priorities <- 1.0 - rnd(200/1000);
			proba_respect_stops <- [1.0];
			proba_block_node <- 0.0;
			proba_use_linked_road <- 0.0;
			max_acceleration <- 5/3.6;
			speed_coeff <- 1.2 - (rnd(400) / 1000);
			threshold_stucked <-int ( (1 + rnd(5))°mn);			
			//affectation aleatoire du type de vehicule suivant le pourcentage 
			//que represente chaque vehicule dans la ville
			if (flip((0.1))) //10% de cammions
			{
				type <- "CAMION";
			//un vehicule creee possede une energie initiale
				energy <- rnd(100.0)+2.0;
			} else if (flip((0.3)))
			{
				type <- "CAR";
				energy <- rnd(70.0)+3.0;
			} else if(flip(0.2))
			{
				type <- "BUS";
				energy <- rnd(80.0)+2.0;
			} else if(flip(0.4)){
				type <- "MOTOBIKE";
				energy <- rnd(50.0)+2.0;
			}
			
			
		}	
	write (length(vehicule));
	//do stop;
    }

//fonction createur de vent
reflex WindGenerator {
    	
		if (flip(0.9))
		{
			create Wind number: 1;
//			float speed <- 100 #km /#h ;
			
		}

	}
	//mise a jour du vent
	reflex WindUpdater	{
		wind <- list(copy(Wind));
	}


}

//grille de polluants
species pollutant_grid skills:[moving] {			
	float direction1;
	string type;
	float speed max: maximal_speed <- maximal_speed;
	float range <- minimal_distance * 2;
	point velocity <- {0,0, 0} ;
	list others update: ((pollutant_grid at_distance range)  - self);
	point mass_center update:  (length(others) > 0) ? (mean (others collect (each.location)) ):location;
	//mouvement de dissipation ou de dispersion
	reflex separation when: apply_separation {
			point orientation<- {0,0,0};
			loop pollutant over: (pollutant_grid at_distance (minimal_distance))  {
				orientation<- orientation- ((location of pollutant) - location);
			}  
			velocity <- velocity + orientation;
		}
		//changement d'altitude (z) en fonction de celle du vent
		action bounding {
			if (location.z) <=0 {
				location <- {location.x,location.y,0};
			} else if (location.z) > z_max {
				location <- {location.x,location.y,z_max};
			}
		}
		//action pour prendre la direction du vent
		reflex follow_goal when: apply_goal {
			velocity <- velocity + ((goal - location) / cohesion_factor);
		}
		//la vitesse de deplacement des polluants sont mise a jour en fonction de celle du vent
		reflex wind when: apply_wind {
			velocity <- velocity + wind_vector;
		}
		//deplacement sur la direction du vent
		action do_move {  
			if (((velocity.x) as int) = 0) and (((velocity.y) as int) = 0) and (((velocity.z) as int) = 0) {
				velocity <- {(rnd(4)) -2, (rnd(4)) - 2,  ((rnd(4)) - 2)} ; 
			}
			point old_location <- location;
			do goto target: location + velocity;
			velocity <- location - old_location;
		}
		
		reflex movement {
			do bounding;
			do do_move;
		}
//		
	//list<float> pollutant <- list_with(6,0.0);
	//float speed<-5.0;
	//aspect visuel ou representation graphique
		aspect base
	{
		if (type = "NOx")
		{
			draw sphere(1) color: # orange;
		}

		if (type = "CO")
		{
			draw sphere(2) color: # gray;
		}

		if (type = "PM")
		{
			draw sphere(3) color: # red;
		}		
		if (type = "SO2")
		{
			draw sphere(4) color: # blue;
		}

	}
	reflex move
	{
		Wind my_wind <- nil;
		float intensity_wind <- 0.0;
		
		if(length(winds where(distance_to(self,each)<=50.0))>0)
		{
			my_wind <- winds where(distance_to(self,each)<=50.0) with_max_of each.intensity;
		}
		
		loop i over: winds
		{
			if (((self distance_to i) < 50.0) and (i.intensity > intensity_wind))
			{
				my_wind <- i;
				intensity_wind <- i.intensity;
				direction1 <- i.direction;
			}

		}
		//stabilite en absence de vent
		if (my_wind != nil)
		{
			self.location <- { self.location.x + cos(my_wind.direction), self.location.y - sin(my_wind.direction) };
			
			do goto target: self.location speed: speed;
		} else
		{
			do wander;
		}

	}
		

}


//species pollutant_grid skills:[moving] {			
//	float direction1;
//	string type;
//	float speed max: maximal_speed <- maximal_speed;
//	float range <- minimal_distance * 2;
//	point velocity <- {0,0, 0} ;
//	list others update: ((pollutant_grid at_distance range)  - self);
//	point mass_center update:  (length(others) > 0) ? (mean (others collect (each.location)) ):location;
//	
//	reflex separation when: apply_separation {
//			point orientation<- {0,0,0};
//			loop pollutant over: (pollutant_grid at_distance (minimal_distance))  {
//				orientation<- orientation- ((location of pollutant) - location);
//			}  
//			velocity <- velocity + orientation ;
//		}
//		
//		action bounding {
//			if (location.z) <= 0 {
//				location <- {location.x,location.y,0};
//			} else if (location.z) > z_max {
//				location <- {location.x,location.y,z_max};
//			}
//		}
//		
//		reflex follow_goal when: apply_goal {
//			velocity <- velocity + ((goal - location) / cohesion_factor);
//		}
//		
//		reflex wind when: apply_wind {
//			velocity <- velocity + wind_vector;
//		}
//		  
//		action do_move {  
//			if (((velocity.x) as int) = 0) and (((velocity.y) as int) = 0) and (((velocity.z) as int) = 0) {
//				velocity <- {(rnd(4)) -2, (rnd(4)) - 2,  ((rnd(4)) - 2)} ; 
//			}
//			point old_location <- location;
//			do goto target: location + velocity;
//			velocity <- location - old_location;
//		}
//		
//		reflex movement {
//			do bounding;
//			do do_move;
//		}
////		
//	//list<float> pollutant <- list_with(6,0.0);
//	//float speed<-5.0;
//	aspect base
//	{
//		if (type = "NOx")
//		{
//			draw sphere(1) color: # orange;
//		}
//
//		if (type = "CO")
//		{
//			draw sphere(2) color: # gray;
//		}
//
//		if (type = "PM")
//		{
//			draw sphere(3) color: # red;
//		}		
//		if (type = "SO2")
//		{
//			draw sphere(4) color: # blue;
//		}
//
//	}
//
//	reflex move
//	{
//		Wind my_wind <- nil;
//		float intensity_wind <- 0.0;
//		
////		if(length(winds where(distance_to(self,each)<=20.0))>0)
////		{
////			my_wind <- winds where(distance_to(self,each)<=20.0) with_max_of each.intensity;
////		}
//		
//		loop i over: winds
//		{
//			if (((self distance_to i) < 50.0) and (i.intensity > intensity_wind))
//			{
//				my_wind <- i;
//				intensity_wind <- i.intensity;
//				direction1 <- i.direction;
//			}
//
//		}
//
//		if (my_wind != nil)
//		{
//			self.location <- { self.location.x + cos(my_wind.direction), self.location.y - sin(my_wind.direction) };
//			
//			do goto target: self.location speed: speed;
//		} else
//		{
//			do wander;
//		}
//
//	}
//		
//
//}


//fonctions et proprietes du vent
species Wind skills: [moving] {
	float direction;
	float intensity;
	point target;
	//float speed ; 
	float speed max: maximal_speed <- maximal_speed;
	//initialisation de la position du vent (x,y)
	init {
		float x_cord <- max([0.0, min([1000.0, gauss({ 500, 50 })])]);
		float y_cord <- max([0.0, min([1000.0, gauss({ 500, 50 })])]);	
		location <- { x_cord, y_cord };
		intensity <- max([0.0, min([100.0, gauss({ 50, 15 })])]);
		
	}
	
	const range type: float init: 20.0;
	const size type: float init: 10.0;
		

	aspect basewind {
		
		draw triangle(10) color: # yellow;
	}

//	reflex move when: intensity > 0 {
//		point cible  <- { self.location.x + cos(self.direction), self.location.y - sin(self.direction) };
//		//self.location <- { self.location.x + cos(self.direction), self.location.y - sin(self.direction),self.location.y - rnd(sin(self.direction)) };
//		intensity <- intensity - 0.5;
//		do goto target: cible speed: speed;
//	}
	//le vent se deplace tant que sa vitesse est sup a 0
	reflex move when: self.intensity > 0 {
		do goto target: target speed: speed;
		self.location <- { self.location.x + cos(self.direction), self.location.y - sin(self.direction) };
		//self.location <- { self.location.x + cos(self.direction), self.location.y - sin(self.direction),self.location.y - rnd(sin(self.direction)) };
		if (location.z) <= 0 {
				location <- {location.x,location.y,0};
			} else if (location.z) > z_max {
				location <- {location.x,location.y,z_max};
			}
		//la force du vent diminue au fur et a mesure qu'il se deplace
		intensity <- intensity - 0.1;
		goal <- location;
		
	}

	//le vent disparait lorsque son intensité(force) devient nulle
	reflex die when: intensity < 0 {
		do die;
	}

}


//species Wind skills: [moving] {
//	float direction;
//	float intensity;
//	init {
////		direction <- max([rnd(3000.0), min([360.0, gauss({ 180, 35 })])]);
////		float x_cord <- max([0.0, min([3000.0, gauss({ 1000, 50 })])]);
////		float y_cord <- max([0.0, min([3000.0, gauss({ 1000, 50 })])]);
////		location <- { x_cord, y_cord };
//		location <- any_location_in(shape);
//		
//		direction <- max([0.0, min([360.0, gauss({ 180, 35 })])]);
//		intensity <- max([0.0, min([100.0, gauss({ 50, 15 })])]);
//		float x_cord <- max([0.0, min([1000.0, gauss({ 500, 50 })])]);
//		float y_cord <- max([0.0, min([1000.0, gauss({ 500, 50 })])]);
//		//location <- { x_cord, y_cord };
//		
//	}
//
////		reflex wander { 
////			//do  wander_3D amplitude: 45 speed: 10;
////			do goto target: target; 
////			if (location.z) < 0 {
////				location <- {location.x,location.y,0};
////			} else if (location.z) > z_max {
////				location <- {location.x,location.y,z_max};
////			}
////			//goal <- location;
////			intensity <- intensity - 0.010;
////		}
//	aspect basewind {
//		
//		draw triangle(10) color: # yellow;
//	}
//
//	reflex move when: intensity > 0 {
//		point cible  <- { self.location.x + cos(self.direction), self.location.y - sin(self.direction) };
//		//self.location <- { self.location.x + cos(self.direction), self.location.y - sin(self.direction),self.location.y - rnd(sin(self.direction)) };
//		intensity <- intensity - 0.5;
//		do goto target: cible speed: speed;
//	}
//
//	reflex die when: intensity < 0 {
//		do die;
//	}
//
//}

//species pollutant_grid skills:[moving] {			
//	float direction1;
//	string type;
//	//list<float> pollutant <- list_with(6,0.0);
//	
//	
//	float speed<-5.0;
//	aspect base
//	{
//		if (type = "NOx")
//		{
//			draw sphere(1) color: # orange;
//		}
//
//		if (type = "CO")
//		{
//			draw sphere(2) color: # gray;
//		}
//
//		if (type = "PM")
//		{
//			draw sphere(3) color: # red;
//		}		
//		if (type = "SO2")
//		{
//			draw sphere(4) color: # blue;
//		}
//
//	}
//
//	reflex move
//	{
//		Wind my_wind <- nil;
//		float intensity_wind <- 0.0;
////		if(length(winds where(distance_to(self,each)<=50.0))>0)
////		{
////			my_wind <- winds where(distance_to(self,each)<=50.0) with_max_of each.intensity;
////		}
//		loop i over: winds
//		{
//			if (((self distance_to i) < 50.0) and (i.intensity > intensity_wind))
//			{
//				my_wind <- i;
//				intensity_wind <- i.intensity;
//				direction1 <- i.direction;
//				point cible  <- { self.location.x + cos(self.direction1), self.location.y - sin(self.direction1) };
//				do goto target: cible speed: speed;
//			}
// //			
//
//		}
//
//		if (my_wind != nil)
//		{
//			point target <- { self.location.x + cos(my_wind.direction), self.location.y - sin(my_wind.direction) };
//			
//			do goto target: target speed: speed;
//		} else
//		{
//			do wander;
//		}
//
//	}
//		
//
//	
//}

grid cell width: 10 height: 10 use_neighbours_cache: false use_individual_shapes: true neighbours: 4{
		init {
		write "my row index is:" + grid_y;
		}
}

//functions et proprietes des routes
species road {
    string type;
    rgb color<- #black;
    
    int nbLanes;
	float coeff_traffic <- 1.0 update: 1 + (float(length(vehicule at_distance 1.0)) / shape.perimeter * 200 /50);
	geometry visu_geom;
	
	//suivi du trafic en fonction du nombre de vehicules sul les axes 
	aspect traffic_jam {  
		if (coeff_traffic > 0.025) {
			draw shape + (coeff_traffic / 4.0) color: rgb("red") ;
		}
	} 
    
	aspect base {
		draw shape+5 color: color;
	}
}

species building {
	string type;
	rgb color <- rgb('gray');
	int height;
	aspect base {
		draw shape+5 color: color depth:3;
	}
}


species noeud {
	string type;
	rgb color <- rgb('gray');
	int height;
	aspect base {
		draw shape color: color;
	}
}

//foctions et proprietes de vehicule
species vehicule skills: [advanced_driving] {
	point target;
	path my_path; 
	float max_speed;
	point source;
	point the_target <- nil;
	int counter_stucked <- 0;
	int threshold_stucked;
	string type;
	float energy;
	bool normalMove <- true;
	point previousLoc <- nil;
	point targetBis <- nil ;
	int evadeDist <-200;
	//float speed <- vitesse_min + rnd(vitesse_max - vitesse_min);
	//deplacement des vehicules sur les routes
	reflex move when: normalMove{
		previousLoc <- copy(location);
		do goto target: target on: the_graph speed: speed ; 
		switch location { 
			match target {
				target <- any_location_in (one_of(road));
				nbGoalsAchived <- nbGoalsAchived +1;
			}
//			match previousLoc {
//				targetBis <- last((one_of(road where (each distance_to self < evadeDist)).shape).points);
//				normalMove <- false;
//			}
		}
	}
	//cas de changement d'itineraire en cas d e congestion
	reflex EvadeMove when: !(normalMove){
		//previousLoc <- copy(location);
		do goto target: targetBis on: the_graph speed: speed; 
		switch location { 
			match targetBis {
				normalMove <- true;
			}
			match previousLoc {
				targetBis <- last((one_of(road where (each distance_to self < evadeDist)).shape).points);
			}
		}
	}	
	
//	
//	reflex time_to_go when: final_target = nil {
//		current_road <- compute_path(graph: the_graph, target: one_of(noeud));
//	}
//	
//	
		reflex find_target 
	{
		the_target <- any_point_in(one_of(road));
		do goto target:target on:the_graph;
		//do drive;
		//write the_target;
		//energy <- energy -0.1;
	}
//	
////	reflex move when: final_target != nil {   
////		do drive;
////	}
//	
		//liberation de polluants par types de vehicule
		//chaque polluant est liberer en tenant compte du facteur d'emission du vehicule
		//par rapport au polluant concerné
		// a1, b1,c3,d4 representent le cumule de chacun des pollluants
		action CamEmmission  {
		//int number_pollutant_grid <- int(size * 2 + speed / 10);
		create pollutant_grid   {
			location<- myself.location;// point ([rnd (500), rnd (500), 350]);
			//0.25 est la probabilite pour qu'un vehicule libere le polluant
			
			if (flip(0.25))//0.8+2.7+0.4+11=14.9
			{
				type <- "PM";
				//0.05 represente le facteur d'emission des pm pour les camions
				 a1 <- pollutant_grid count (each.type = "PM")*0.05;
			} 
			 else if(flip(0.25))
			{
				type <- "CO";
				 b1 <- pollutant_grid count (each.type = "CO")*2.75;
			} 
			 else if(flip(0.25))
			{
				type <- "SO2";
				 c1 <- pollutant_grid count (each.type = "SO2")*0.4;
			} 
			 else if(flip(0.25)){
				type <- "NOx";
				 d1 <- pollutant_grid count (each.type = "NOx")*11.0;
			}

		}

	
	}
	
			action CarEmmission { 
		//int number_pollutant_grid <- int(size * 2 + speed / 10);
		create pollutant_grid  {
			location<- myself.location;// point ([rnd (500), rnd (500), 350]);
//			//write "my row index is:" + grid_y;
//			neighboor_vehicule <- vehicule at_distance 20#m;
//			activeVehicule <- activeVehicule accumulate(neighboor_vehicule); 
			if (flip(0.25))
			{
				type <- "PM";
				 a2 <- pollutant_grid count (each.type = "PM")*0.1;
				 
			}  else if (flip(0.25))
			{
				type <- "CO";
				 b2 <- pollutant_grid count (each.type = "CO")*3.62;
			}  else if(flip(0.25))
			{
				type <- "SO2";
				 c2 <- pollutant_grid count (each.type = "SO2")*0.17;
			}  else if(flip(0.25)){
				type <- "NOx";
				 d2 <- pollutant_grid count (each.type = "NOx")*11.0;
			}

		}

	
	}
	
			action BusEmmission { 
		//int number_pollutant_grid <- int(size * 2 + speed / 10);
		create pollutant_grid  {
			location<- myself.location;// point ([rnd (500), rnd (500), 350]);
			
			if (flip(0.25))
			{
				type <- "PM";
				 a3 <- pollutant_grid count (each.type = "PM")*1.5;
			}  else if (flip(0.25))
			{
				type <- "CO";
				 b3 <- pollutant_grid count (each.type = "CO")*3.1;
			}  else if(flip(0.25))
			{
				type <- "SO2";
				 c3 <- pollutant_grid count (each.type = "SO2")*0.64;
			}  else if(flip(0.25)){
				type <- "NOx";
				 d3 <- pollutant_grid count (each.type = "NOx")*7.6;
			}
			
			
		}

	
	}
	
			action MotEmmission { 
		//int number_pollutant_grid <- int(size * 2 + speed / 10);
		create pollutant_grid  {
			location<- myself.location;// point ([rnd (500), rnd (500), 350]);
//			//write "my row index is:" + grid_y;
//			neighboor_vehicule <- vehicule at_distance 20#m;
//			activeVehicule <- activeVehicule accumulate(neighboor_vehicule); 
//			if (flip(0.1))
//			{
//				type <- "PM";
//				float a <- pollutant_grid count (each.type = "PM")*0.1;
//			}  if (flip(3.62))
//			{
//				type <- "CO";
//			}  if(flip(0.03))
//			{
//				type <- "SO2";
//			}  if(flip(0.3)){
//				type <- "NOx";
//			}
			
			if (flip(0.25))
			{
				type <- "PM";
				 a4 <- pollutant_grid count (each.type = "PM")*0.1;
			}  else if (flip(0.25))
			{
				type <- "CO";
				 b4 <- pollutant_grid count (each.type = "CO")*3.62;
			}  else if(flip(0.25))
			{
				type <- "SO2";
				 c4 <- pollutant_grid count (each.type = "SO2")*0.03;
			}  else if(flip(0.25)){
				type <- "NOx";
				 d4 <- pollutant_grid count (each.type = "NOx")*0.3;
			}

		}
	
	
	}
//	reflex som {
//		
//		set pollutant_grid count (each.type = "NOx")<- d1+d2+d3+d4;
//	}
	
	
	reflex EmitAction when: (flip(0.2) ){
	if (type = "CAMION" and energy>0 ){
			do  CamEmmission;
			set energy <- energy -2.5;	
		if (energy<0)	{
			do die;
		}	
		}
	
	if (type = "CAR" and energy>0){
			do CarEmmission;
			set energy <- energy -1.55;
		if (energy<0)	{
			do die;
		}	
			
		}
	
		
	if (type = "BUS" and energy>0){
			do BusEmmission;
			set energy <- energy -1.6;
		if (energy<0)	{
			do die;
		}	
		}
	
	if (type = "MOTOBIKE" and energy>0){
			do MotEmmission;
			//une fois que le vehicule libere de polluants, sont energie diminue
			set energy <- energy -3.6;	
			//si son energie finie, alors le vehicule s'arrete
		if (energy<0)	{
			do die;
		}			
						
		}			
		}
		
		reflex stop when:energy <=0{
			do die;
		}
		
			aspect base
	{
		if (type = "CAMION"){
		//do emmission;		
			draw circle(15) color: # orange;
		}

		if (type = "CAR") {
			//do emmission;
			draw circle(10) color: # brown;
		}

		if (type = "BUS"){
			//do emmission;
			draw square(15) color: # blue;
		}

		if (type = "MOTOBIKE"){
			//do emmission;
			draw triangle(8) color: # black;
		}

	}
		
//	aspect base {
//		draw circle(10) color: #green;
//	}
	
//	reflex movement {
//		do goto target:target on:the_graph;
//	}
}

experiment Air_Pollution_2 type: gui {
	output {
		display city_display type:opengl refresh_every:1 {
			image '../includes/images/org.png' refresh: false;
			//species building aspect:base;
			
			//species noeud aspect:base;
			
			//species Wind aspect:basewind;
			species pollutant_grid aspect:base;
		}
		
//		display polluant type:opengl  {
//			image '../includes/images/org.png' refresh: false;
//			//species Wind aspect:basewind;
//			species pollutant_grid aspect:base;
//		}
		
		display trafficjmp refresh_every: 1 {
//			species road aspect: base ;
			species road aspect: traffic_jam ;
			//species road aspect:base;			
			species vehicule aspect:base;
			//grid cell lines: rgb("red") ;
		}
		
		//visualisation du nombre total de polluants par type
		monitor "PM pollutant" value: pollutant_grid count (each.type = "NOx");
		monitor "CO pollutant" value: pollutant_grid count (each.type = "CO");
		monitor "SO2 pollutant" value: pollutant_grid count (each.type = "SO2");
		monitor "NOx pollutant" value: pollutant_grid count (each.type = "PM");
		
		
		display chart_display refresh_every: 15 {		
			
			chart "Gas Status" type: series 
			{
				
				data "SO2" value:a1+a2+a3+a4 style: line color: # green;
				data "CO" value: b1+b2+b3+b4 style: line color: # orange;
				data "PM" value: c1+c2+c3+c4 style: line color: # red;
				data "NOx" value: d1+d2+d3+d4  style: line color: # blue;

			}

		}
		
	}
}


