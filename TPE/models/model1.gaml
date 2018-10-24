/**
 *  new
 *  Author: rdius
 *  Description: 
 */

model Marrasketch

global {
//	file shape_file_roads <- file("../includes/SIG_simu/roads_gama.shp");
//	file shape_file_buildings <- file("../includes/SIG_simu/buildings_gama.shp");
//	file shape_file_bound <- file("../includes/SIG_simu/reperes.shp");
//	file shape_file_roads <- file("../includes/carrefour/road.shp");
//	file shape_file_buildings <- file("../includes/carrefour/building.shp");
//	
//	file shape_file_roads <- file("../includes/shape/road.shp");
//	file shape_file_buildings <- file("../includes/shape/building.shp");
//	
//	file shape_file_roads <- file("../includes/data/road.shp");
//	file shape_file_buildings <- file("../includes/data/building.shp");
//	//file shape_file_bound <- file("../includes/data/reperes.shp");
	
	file shape_file_roads <- file("../includes/orig/road.shp");
	file shape_file_buildings <- file("../includes/orig/buildings.shp");
	file shape_file_node <- file("../includes/orig/node.shp");
	
	graph the_graph; 
	geometry shape <- envelope(shape_file_roads)-10;
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
	
	init {
		create noeud from:shape_file_node;
		create road from: shape_file_roads with:[type::string(get("type"))]{
         if (type = 'primary') {
				color <- #red;
	     }
	     if (type = 'secondary') {
				color <- #blue;
	     }
	     if (type = 'tertiary') {
				color <- #black;
	     }
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

	reflex VehiculeCreator when:length(vehicule)<=300  {
		create vehicule number: 5 { 
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
			//proba_breakdown <- 0.00001;
			
			if (flip((0.1)))
			{
				type <- "CAMION";
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
    }


reflex WindGenerator {
    	
		if (flip(0.9))
		{
			create Wind number: 5;
			
		}

	}
	
	reflex WindUpdater	{
		wind <- list(copy(Wind));
	}


}

species Wind skills: [moving] {
	float direction;
	float intensity;
	init {
//		direction <- max([rnd(3000.0), min([360.0, gauss({ 180, 35 })])]);
//		float x_cord <- max([0.0, min([3000.0, gauss({ 1000, 50 })])]);
//		float y_cord <- max([0.0, min([3000.0, gauss({ 1000, 50 })])]);
//		location <- { x_cord, y_cord };
		location <- any_location_in(shape);
		
		direction <- max([0.0, min([360.0, gauss({ 180, 35 })])]);
		intensity <- max([0.0, min([100.0, gauss({ 50, 15 })])]);
		float x_cord <- max([0.0, min([1000.0, gauss({ 500, 50 })])]);
		float y_cord <- max([0.0, min([1000.0, gauss({ 500, 50 })])]);
		//location <- { x_cord, y_cord };
		
	}

//		reflex wander { 
//			//do  wander_3D amplitude: 45 speed: 10;
//			do goto target: target; 
//			if (location.z) < 0 {
//				location <- {location.x,location.y,0};
//			} else if (location.z) > z_max {
//				location <- {location.x,location.y,z_max};
//			}
//			//goal <- location;
//			intensity <- intensity - 0.010;
//		}
	aspect basewind {
		
		draw triangle(10) color: # yellow;
	}

	reflex move when: intensity > 0 {
		point cible  <- { self.location.x + cos(self.direction), self.location.y - sin(self.direction) };
		//self.location <- { self.location.x + cos(self.direction), self.location.y - sin(self.direction),self.location.y - rnd(sin(self.direction)) };
		intensity <- intensity - 0.5;
		do goto target: cible speed: speed;
	}

	reflex die when: intensity < 0 {
		do die;
	}

}

species pollutant_grid skills:[moving] {			
	float direction1;
	string type;
	//list<float> pollutant <- list_with(6,0.0);
	
	
	float speed<-5.0;
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
//		if(length(winds where(distance_to(self,each)<=50.0))>0)
//		{
//			my_wind <- winds where(distance_to(self,each)<=50.0) with_max_of each.intensity;
//		}
		loop i over: winds
		{
			if (((self distance_to i) < 50.0) and (i.intensity > intensity_wind))
			{
				my_wind <- i;
				intensity_wind <- i.intensity;
				direction1 <- i.direction;
				point cible  <- { self.location.x + cos(self.direction1), self.location.y - sin(self.direction1) };
				do goto target: cible speed: speed;
			}

			
		}

		if (my_wind != nil)
		{
			point target <- { self.location.x + cos(my_wind.direction), self.location.y - sin(my_wind.direction) };
			
			do goto target: target speed: speed;
		} else
		{
			do wander;
		}

	}
		

	
}


species road {
    string type;
    rgb color<- #black;
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
	//float speed <- vitesse_min + rnd(vitesse_max - vitesse_min);
	
	
	
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
	
		action CamEmmission  { 
		//int number_pollutant_grid <- int(size * 2 + speed / 10);
		create pollutant_grid   {
			location<- myself.location;// point ([rnd (500), rnd (500), 350]);
//			//write "my row index is:" + grid_y;
//			neighboor_vehicule <- vehicule at_distance 20#m;
//			activeVehicule <- activeVehicule accumulate(neighboor_vehicule); 
			if (flip(0.25))//0.8+2.7+0.4+11=14.9
			{
				type <- "PM";
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
//			//write "my row index is:" + grid_y;
//			neighboor_vehicule <- vehicule at_distance 20#m;
//			activeVehicule <- activeVehicule accumulate(neighboor_vehicule); 
//			if (flip(1.5))
//			{
//				type <- "PM";
//				float a <- pollutant_grid count (each.type = "PM")*0.1;
//			}  if (flip(3.1))
//			{
//				type <- "CO";
//			}  if(flip(0.64))
//			{
//				type <- "SO2";
//			}  if(flip(7.6)){
//				type <- "NOx";
//			}
			
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
	
	
	reflex EmitAction when: flip(0.2) {
	if (type = "CAMION" ){
			do  CamEmmission;
			set energy <- energy -2.5;
						
			
		}
	if (type = "CAR"){
			do CarEmmission;
			set energy <- energy -2;
			
		}
		
	if (type = "BUS"){
			do BusEmmission;
			set energy <- energy -1.5;		
			
		}
		
	if (type = "MOTOBIKE"){
			do MotEmmission;
			set energy <- energy -1.6;
				
			
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

experiment GIS_agentification type: gui {
	output {
		display city_display type:opengl refresh_every:1 {
			//image '../includes/images/org.png' refresh: false;
			//species building aspect:base;
			species road aspect:base;
			
			species vehicule aspect:base;
			//species noeud aspect:base;
			
//			species Wind aspect:basewind;
//			species pollutant_grid aspect:base;
		}
		
//		display polluant type:opengl  {
//			image '../includes/images/org.png' refresh: false;
//			//species Wind aspect:basewind;
//			species pollutant_grid aspect:base;
//		}
		
//		display chart_display refresh_every: 5 {		
//			
//			chart "Gas Status" type: series {
//				
//				data "PM" value:a1+a2+a3+a4 style: line color: # green;
//				data "CO" value: b1+b2+b3+b4 style: line color: # orange;
//				data "SO2" value: c1+c2+c3+c4 style: line color: # red;
//				data "NOX" value: d1+d2+d3+d4  style: line color: # blue;
//
//			}
//
//		}
		
	}
}


