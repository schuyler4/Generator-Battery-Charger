module ShuntResistor()
{
    cylinder_resolution = 1000;
    
    resistor_rotation = [90, 0, 0];
    resistor_length = 23.24;
    
    resistor_color = "SaddleBrown";
    
    lead_length = 10;
    lead_radius = 1.5;
    
    lead_color = "LightGrey";
    
    rotate(resistor_rotation)
    {
        color(resistor_color, 1.0)
        cylinder(
            h=resistor_length,
            r=4.17,
            center=true,
            $fn=cylinder_resolution );
        
        color(lead_color, 1.0)
        cylinder(
            h=resistor_length + 2*lead_length, 
            r=lead_radius, 
            center=true, 
            $fn=cylinder_resolution );
    }
}

     
