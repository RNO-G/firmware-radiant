This Directory contains the Verilog source code for the RADIANT board.


radiant_top.v: This is the top level file for the Radiant board project.

boardman_interface.v:This communicates over external lines to to allow firmware to be controlled by external software.

wbc_intercon.v:Connects and configures the Wishbone interface used to give addresses to various firmware features accessed by the software, and connect the addressing system to various firmware parts.

rad_id_ctrl.v:

lab4d_controller.sv:Processes outgoing commands to the Lab4Ds, communicating them to the picoblaze processor for more complex commands. Takes generated commands both from firmware and picoblaze and sends the actual signals out to the lab4Ds (via the CPLDs).
	radiant_trigger_control_v2.v: the code internal to the controller that handles telling the lab4ds what data storage bank to use to hold the current live data. When a trigger is recieved it also handles processing the exact addresses of data to be read out from the lab4ds, and handles retriggering to achieve larger sample sizes, as well as any variable delays between blocks of lab4ds. 

par_lab4d_readout.v:reads out the raw data coming back from the Lab4ds and passes it to the calram.

wb_calram_v2.v:temporary data storage and processing, passes the data to the par_lab4d_fifo with some processing.

par_lab4d_fifo.sv: Stores data until it is read out by software.

radiant_trig_top.v: monitors the 24 raw LAb4d trigger inputs, and uses them to determine if an event should be triggered. If it is, it handles generating event header data that can be combined with the raw data at a later point, as well as directly sending triggers to the lab4d controller so the raw data can be requested from the lab4ds.

radiant_scalers.sv:

spidma.v:
