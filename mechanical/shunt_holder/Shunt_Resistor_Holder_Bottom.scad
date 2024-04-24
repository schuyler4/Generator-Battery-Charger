module ShuntResistorHolderBottom()
{
    base_thickness = 4;
    base_length = 65;
    base_size = [base_length,14,base_thickness];
    
    mounting_hole_diameter = 3.2;
    
    cylinder_resolution = 1000;
    tolerance = 0.25;
    
    resistor_diameter = 8.35;
    holder_height = base_thickness+resistor_diameter/2;
    
    // Create a base and mounting holes.
    difference()
    {
        cube(base_size, center=true);
        for(i=[-1:1])
        {
            if(i != 0)
            {
                offset = (base_length/2)-mounting_hole_diameter-1;
                translate([offset*i,0,0])
                cylinder(
                    d=mounting_hole_diameter, 
                    h=base_thickness+2,
                    center=true,
                    $fn=cylinder_resolution);
            }
        }
    }
    
    cavity_vertical_translation = base_thickness+resistor_diameter/2;
    
    // Create the part that holds the resistor.
    difference()
    {
        translate([0, 0, holder_height/2])
        cube([32, 14, holder_height], center=true);
        rotate([90, 0, 90])
        
        translate([0, cavity_vertical_translation, 0])
        cylinder(
            d=resistor_diameter+tolerance, 
            h=100, 
            center=true,
            $fn=cylinder_resolution);
    }
}

ShuntResistorHolderBottom();