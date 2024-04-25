<h1> Generator Battery Charger </h1>
<p>
This is exactly what it sounds like, it is a generator that charges a lithium-ion battery pack. The difference
between this and a just plugging a battery charger into a generator (good luck finding a good HV battery charger off the
shelf) is the regulation is full system. That is, the constant current delivery to the battery is controlled via the
throttle on the engine.  
</p>
<h3>To What Extent Does this Actually Work?</h3>
<p>
A ~150V battery pack was charged for several minutes with the system, and it appeared to regulate current fairly well. A longer test
was not done because the battery pack was not fully protected with BMS (Cowboy Charging).  
There was not excessive current into the battery pack, but there was some ripple in the current (~0.5A), so the control system did not work
perfectly, but it did seem to behave more like a current source than a voltage source. Additionally, the battery pack was made out of
reused 18650 cells which may have had a higher than normal series resistance. This may have had an effect on the behavior of the control
system. 
</p>

See [Here](https://hackaday.io/project/195765-generator-battery-charger "here") for additional documentation
<h3> Directory Structure </h3>
<ul>
<li><b>circuit_board_designs:</b> This includes all the custom circuit board designs that were made for the project.</li>
<li><b>control_program:</b> This has several revisions of the firmware that controls the whole system. This firmware is run on an Arduino Nano.</li>
<li><b>mechanical:</b> Some 3D printed mechanical interface stuff.</li>
<li><b>pictures:</b> Pictures of the setup.</li>
<li><b>system:</b> This has schematic diagrams for both the HV powertrain and the low voltage system.</li>
<li><b>test:</b> Documentation of all the testing that was done with the system and various other component testing.</li>
</ul>