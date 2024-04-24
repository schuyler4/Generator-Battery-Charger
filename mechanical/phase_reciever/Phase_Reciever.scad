module PhaseReciever()
{
    thickness = 4;
    width = 68;
    height = 30;
    
    hole_distance = 20;
    
    plug_hole_size = 7.9;
    
    curve_resolution=1000;
    
    fastener_hole_size = 3.5;
    
    foot_size = 20;
    
    gantrey_offset = 30;
    
    difference()
    {
        cube([width, thickness, height], center=true);
        
        for(i = [-1:1])
        {
            translate([i*hole_distance, 0, 0])
            rotate([90, 0, 0])
            cylinder(
                h=thickness+1, 
                center=true,
                d=plug_hole_size,
                $fn=curve_resolution);
        }
    }
    
    difference()
    {
        for(i = [-1:1])
        {
            if(i != 0)
            {
                translate([i*width/2, 0, -height/2 + 2])
                cube(
                    [foot_size, foot_size, thickness], 
                    center=true);
            }   
        }
        for(i = [-1:1])
        {
            if(i != 0)
            {
                translate([i*((width/2)+5), 0, -height/2 + 2])
                cylinder(
                    h=thickness+1, 
                    d=fastener_hole_size,
                    $fn=1000,
                    center=true);
            }
        }
    }  
    
    for(i = [-1:1])
    {
        if(i != 0)
        {
            translate([i*gantrey_offset,0,-2])
            rotate([-90,0,90])
            linear_extrude(height=thickness, center=true)
            polygon(points=[[0,0], [10,10], [0,10]]);
            
            translate([i*gantrey_offset,0,-2])
            rotate([-90,0,-90])
            linear_extrude(height=thickness, center=true)
            polygon(points=[[0,0], [10,10], [0,10]]);
        }
    }
}

PhaseReciever();