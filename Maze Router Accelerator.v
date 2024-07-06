
module MRA(
	// CHIP IO
	clk            	,	
	rst_n          	,	
	in_valid       	,	
	frame_id        ,	
	net_id         	,	  
	loc_x          	,	  
    loc_y         	,
	cost	 		,		
	busy         	,

    // AXI4 IO
	     arid_m_inf,
	   araddr_m_inf,
	    arlen_m_inf,
	   arsize_m_inf,
	  arburst_m_inf,
	  arvalid_m_inf,
	  arready_m_inf,
	
	      rid_m_inf,
	    rdata_m_inf,
	    rresp_m_inf,
	    rlast_m_inf,
	   rvalid_m_inf,
	   rready_m_inf,
	
	     awid_m_inf,
	   awaddr_m_inf,
	   awsize_m_inf,
	  awburst_m_inf,
	    awlen_m_inf,
	  awvalid_m_inf,
	  awready_m_inf,
	
	    wdata_m_inf,
	    wlast_m_inf,
	   wvalid_m_inf,
	   wready_m_inf,
	
	      bid_m_inf,
	    bresp_m_inf,
	   bvalid_m_inf,
	   bready_m_inf 
);

// ===============================================================
//  					Input / Output 
// ===============================================================

// << CHIP io port with system >>
input 			  	clk,rst_n;
input 			   	in_valid;
input  [4:0] 		frame_id;
input  [3:0]       	net_id;     
input  [5:0]       	loc_x; 
input  [5:0]       	loc_y; 
output reg [13:0] 	cost;
output reg          busy;       
parameter ID_WIDTH=4, DATA_WIDTH=128, ADDR_WIDTH=32;
// ------------------------
// <<<<< AXI READ >>>>>
// ------------------------
// (1)	axi read address channel 
output wire [ID_WIDTH-1:0]      arid_m_inf;
output wire [1:0]            arburst_m_inf;
output wire [2:0]             arsize_m_inf;
output wire [7:0]              arlen_m_inf;
output reg                  arvalid_m_inf;
input  wire                  arready_m_inf;
output reg [ADDR_WIDTH-1:0]  araddr_m_inf;
// ------------------------
// (2)	axi read data channel 
input  wire [ID_WIDTH-1:0]       rid_m_inf;
input  wire                   rvalid_m_inf;
output reg                   rready_m_inf;
input  wire [DATA_WIDTH-1:0]   rdata_m_inf;
input  wire                    rlast_m_inf;
input  wire [1:0]              rresp_m_inf;
// ------------------------
// <<<<< AXI WRITE >>>>>
// ------------------------
// (1) 	axi write address channel 
output wire [ID_WIDTH-1:0]      awid_m_inf;
output wire [1:0]            awburst_m_inf;
output wire [2:0]             awsize_m_inf;
output wire [7:0]              awlen_m_inf;
output reg                  awvalid_m_inf;
input  wire                  awready_m_inf;
output reg [ADDR_WIDTH-1:0]  awaddr_m_inf;
// -------------------------
// (2)	axi write data channel 
output reg                   wvalid_m_inf;
input  wire                   wready_m_inf;
output wire [DATA_WIDTH-1:0]   wdata_m_inf;
output reg                    wlast_m_inf;
// -------------------------
// (3)	axi write response channel 
input  wire  [ID_WIDTH-1:0]      bid_m_inf;
input  wire                   bvalid_m_inf;
output reg                   bready_m_inf;
input  wire  [1:0]             bresp_m_inf;

// ===============================================================
//  					FSM
// ===============================================================
parameter IDLE = 0, DRAM_READ = 1, FILLING = 2, RETRACE = 3, DRAM_WRITE = 4;
reg [2:0] c_state, n_state;
reg filling_over;
reg retrace_over;
reg [3:0] total_net_num, cur_net;

always @(posedge clk, negedge rst_n)begin
	if(!rst_n) c_state <= IDLE;
	else	   c_state <= n_state;
end

always @* begin
	case(c_state)
		IDLE: begin
			if(in_valid)     								  n_state = DRAM_READ;
			else		     								  n_state = IDLE;
		end
		DRAM_READ: begin
			if(rlast_m_inf)  								  n_state = FILLING;
			else			 								  n_state = DRAM_READ;
		end 
		FILLING: begin
			if(filling_over) 								  n_state = RETRACE;
			else			 							 	  n_state = FILLING;
		end
		RETRACE: begin
			if(cur_net == total_net_num && retrace_over)      n_state = DRAM_WRITE;
			else if(cur_net != total_net_num && retrace_over) n_state = FILLING;
			else											  n_state = RETRACE;
  		end
  		DRAM_WRITE: begin
          	if (wlast_m_inf) 								  n_state = IDLE;
          	else             								  n_state = DRAM_WRITE;
  		end
  		default: 		     								  n_state = IDLE;
	endcase
end


// ===============================================================
//  					Input parameter
// ===============================================================
assign arid_m_inf    = 4'd0; 
assign awid_m_inf    = 4'd0; 	  
assign arburst_m_inf = 2'b01;
assign awburst_m_inf = 2'b01;		
assign arsize_m_inf  = 3'b100;	
assign awsize_m_inf  = 3'b100;
assign arlen_m_inf   = 8'd127;     
assign awlen_m_inf   = 8'd127;
reg in_valid_reg;
reg [4:0]  frame_id_fixed;
reg [6:0]  read_DRAM_cnt;
wire [6:0] address_tmp;

assign address_tmp = read_DRAM_cnt + wready_m_inf + 1'b1;
always @(posedge clk, negedge rst_n)begin
	if(!rst_n)			frame_id_fixed <= 0;
	else begin
		if(in_valid)	frame_id_fixed <= frame_id;
		else			frame_id_fixed <= frame_id_fixed;
	end		
end

always @(posedge clk, negedge rst_n) begin
	if(!rst_n)	busy <= 0;
	else		busy <= (c_state != IDLE) & !(in_valid);
end

wire RETRACE_reg = (c_state == RETRACE);
// ===============================================================
//  				location SRAM
// ===============================================================
wire [6:0]  address_loc;
wire [127:0] loc_input;
reg [127:0]  loc_output;
reg [127:0] data_back;


reg [1:0] DRAM_WRITE_cnt;
always @(posedge clk, negedge rst_n)begin
	if(!rst_n) DRAM_WRITE_cnt <= 0;
	else begin
		if(c_state == DRAM_WRITE) begin
			if(DRAM_WRITE_cnt < 3) DRAM_WRITE_cnt <= DRAM_WRITE_cnt + 1;
			else				   DRAM_WRITE_cnt <= DRAM_WRITE_cnt;
		end 
		else					   DRAM_WRITE_cnt <= 0;
	end
end

always @(posedge clk, negedge rst_n)begin
	if(!rst_n)	read_DRAM_cnt <= 7'd0;
	else begin
		if(rvalid_m_inf || wready_m_inf) read_DRAM_cnt <= read_DRAM_cnt + 1;
		else			 				 read_DRAM_cnt <= read_DRAM_cnt;
	end
end

wire [6:0] retrace_addr;
assign address_loc = (RETRACE_reg) ? retrace_addr : (c_state == DRAM_WRITE && DRAM_WRITE_cnt > 1) ? address_tmp : read_DRAM_cnt;
assign loc_input = (c_state == DRAM_READ && rvalid_m_inf) ? rdata_m_inf : data_back;

reg [5:0] next_y_retrace, next_x_retrace;
reg WEB_retrace;
wire WEB_loc;
assign WEB_loc = !(c_state == DRAM_READ && rvalid_m_inf) && WEB_retrace;
assign retrace_addr = (next_y_retrace << 1) + next_x_retrace[5];

// ===============================================================
//  				weight SRAM
// ===============================================================
wire [6:0]  address_weight;
wire [127:0] weight_input;
reg [127:0] weight_output;
wire WEB_weight;
reg weight_read_done;

always @(posedge clk, negedge rst_n)begin
	if(!rst_n) weight_read_done <= 0;
	else begin
		if(rlast_m_inf && c_state != DRAM_READ)   weight_read_done <= 1;
		else if(c_state == IDLE)				  weight_read_done <= 0;
		else							          weight_read_done <= weight_read_done;
	end
end

assign WEB_weight = !(rvalid_m_inf && c_state != DRAM_READ);
assign address_weight = (weight_read_done) ? retrace_addr : read_DRAM_cnt;
assign weight_input   = (c_state == IDLE) ? 128'd0 : rdata_m_inf;

// ===============================================================
//  					DRAM address
// ===============================================================
reg [31:0] dram_address;
always @* begin
	if(c_state == DRAM_READ || c_state == DRAM_WRITE)   dram_address = {16'h0001, frame_id_fixed, 11'd0};
	else					         				    dram_address = {16'h0002, frame_id_fixed, 11'd0};
end

reg [127:0] wdata, Zero, loc_output_reg;
always @(posedge clk, negedge rst_n) begin
	if(!rst_n)										      Zero <= 0;
	else begin
		if (c_state == DRAM_WRITE && DRAM_WRITE_cnt == 2) Zero <= loc_output;
		else 											  Zero <= Zero;
	end
	
end

always @(posedge clk, negedge rst_n) begin
    if (!rst_n) loc_output_reg <= 128'd0;
    else        loc_output_reg <= loc_output;
end

always @* begin
    if (read_DRAM_cnt != 0) wdata = loc_output_reg;
    else                    wdata = Zero;  /// zero data of wdata
end


// ===============================================================
//  					bridge
// ===============================================================
reg bridge_on;
wire mode;
reg fill_label; // 第一次filling，需要從dram讀weight

always @(posedge clk, negedge rst_n)begin
	if(!rst_n) 		    fill_label <= 1;
	else begin
		if(RETRACE_reg) fill_label <= 0;
		else 		    fill_label <= fill_label;
	end
end

assign mode = c_state != DRAM_WRITE;
always @(posedge clk, negedge rst_n)begin
	if(!rst_n) bridge_on <= 1'd0;
	else begin
		if(c_state == DRAM_READ)	                   bridge_on <= 1'd1;
		else if(!(&read_DRAM_cnt))begin
			if(c_state == DRAM_WRITE)				   bridge_on <= 1'd1;
			else if(c_state == FILLING && fill_label)  bridge_on <= 1'd1;
		end
		else									       bridge_on <= 1'd0;
	end
end


always @(posedge clk, negedge rst_n) begin
    if(!rst_n) wlast_m_inf <= 1'b0;
    else begin
        if (c_state == DRAM_WRITE && read_DRAM_cnt == 7'd126)	wlast_m_inf <= 1'b1;
		else													wlast_m_inf <= 1'b0;
	end
end

parameter IDLE_BRIDGE = 0, DRAM_READ_WAIT_ARREADY = 1, DRAM_READ_WAIT_RLAST = 2, DRAM_WRITE_WAIT_AWREADY = 3, DRAM_WRITE_WAIT_WLAST = 4, DRAM_WRITE_WAIT_BVALID = 5;
reg [2:0] c_state_b, n_state_b;

always @(posedge clk, negedge rst_n)begin
	if(!rst_n) c_state_b <= IDLE_BRIDGE;
	else	   c_state_b <= n_state_b;
end

always @* begin
	case(c_state_b)
		IDLE_BRIDGE: begin
			if(bridge_on && mode) 				 n_state_b = DRAM_READ_WAIT_ARREADY;
			else if(bridge_on && !mode)			 n_state_b = DRAM_WRITE_WAIT_AWREADY;
			else		        				 n_state_b = IDLE_BRIDGE;
		end
		DRAM_READ_WAIT_ARREADY: begin
			if(arready_m_inf && arvalid_m_inf)   n_state_b = DRAM_READ_WAIT_RLAST;
			else			                     n_state_b = DRAM_READ_WAIT_ARREADY;
		end
		DRAM_READ_WAIT_RLAST: begin
			if(rvalid_m_inf && rlast_m_inf)  	 n_state_b = IDLE_BRIDGE;
			else								 n_state_b = DRAM_READ_WAIT_RLAST;
		end
		DRAM_WRITE_WAIT_AWREADY: begin
			if(awready_m_inf)   				 n_state_b = DRAM_WRITE_WAIT_WLAST;
			else								 n_state_b = DRAM_WRITE_WAIT_AWREADY;
		end
		DRAM_WRITE_WAIT_WLAST: begin
			if(wlast_m_inf)     				 n_state_b = DRAM_WRITE_WAIT_BVALID;
			else								 n_state_b = DRAM_WRITE_WAIT_WLAST;
		end
		DRAM_WRITE_WAIT_BVALID: begin
			if(bvalid_m_inf)    				 n_state_b = IDLE_BRIDGE;
			else								 n_state_b = DRAM_WRITE_WAIT_BVALID;
		end
		default:								 n_state_b = IDLE_BRIDGE;
	endcase
end


// ==============================================================
//  					DRAM READ
// ===============================================================
always @(posedge clk, negedge rst_n)begin
	if(!rst_n) begin
		arvalid_m_inf <= 1'd0;
		araddr_m_inf  <= 32'd0;
	end
	else begin
		if(c_state_b == DRAM_READ_WAIT_ARREADY) begin
			if(!arready_m_inf) begin
				arvalid_m_inf <= 1'd1;
				araddr_m_inf  <= dram_address;
			end
			else begin
				arvalid_m_inf <= 1'd0;
				araddr_m_inf  <= 32'd0;
			end
		end
	end
end

always @(posedge clk, negedge rst_n)begin
	if(!rst_n) 												  rready_m_inf <= 1'd0;
	else begin
		if(c_state_b == DRAM_READ_WAIT_RLAST && !rlast_m_inf) rready_m_inf <= 1'd1;
		else												  rready_m_inf <= 1'd0;
	end
end


// ==============================================================
//  					DRAM WRITE
// ===============================================================
assign	wdata_m_inf = wdata;
always @(posedge clk, negedge rst_n)begin
	if(!rst_n) begin
		awvalid_m_inf <= 1'd0;
		awaddr_m_inf  <= 32'd0;
	end
	else begin
		if(c_state_b == DRAM_WRITE_WAIT_AWREADY) begin
			if(!awready_m_inf) begin
				awvalid_m_inf <= 1'd1;
				awaddr_m_inf  <= dram_address;
			end
			else begin
				awvalid_m_inf <= 1'd0;
				awaddr_m_inf  <= 32'd0;
			end
		end
		else begin
			awvalid_m_inf <= 1'd0;
			awaddr_m_inf  <= 32'd0;
		end
	end
end

reg [7:0 ]debug_cnt;
always @(posedge clk, negedge rst_n)begin
	if(!rst_n) begin
		wvalid_m_inf  <= 1'd0;
		debug_cnt     <= 8'd0;
	end
	else begin
		if(c_state_b == DRAM_WRITE_WAIT_WLAST) begin
			if(!wlast_m_inf) begin
				debug_cnt 	  <= debug_cnt + 1'd1;
				wvalid_m_inf  <= 1'd1;
			end
			else begin
				debug_cnt 	  <= debug_cnt;
				wvalid_m_inf  <= 1'd0;
			end
		end
	end
end

always @(posedge clk, negedge rst_n)begin
	if(!rst_n) 				  									 bready_m_inf <= 1'd0;
	else begin
		if(c_state_b == DRAM_WRITE_WAIT_BVALID && !bvalid_m_inf) bready_m_inf <= 1'd1;
		else				  									 bready_m_inf <= 1'd0;
	end
end

// ===============================================================
//  					Path Map
// ===============================================================
reg [5:0] x_retrace, y_retrace;
reg [3:0] num_cnt;
reg flag;
reg [5:0] x_source [0:14];
reg [5:0] y_source [0:14];
reg [5:0] x_sink [0:14];
reg [5:0] y_sink [0:14];
reg [3:0] net_num [0:14];
reg [1:0] path_map [0:63] [0:63];
wire retrace_start;

// ===============================================================
//  					calculate cost
// ===============================================================
wire nxt_retrace_over =((x_retrace == x_source[cur_net]) && (y_retrace == y_source[cur_net]));
reg [1:0] retrace_cnt;
always @(posedge clk, negedge rst_n) begin
  	if(!rst_n) begin
  		  cost <= 14'b0;
    end
  	else begin
    	if(in_valid) 						   										    cost <= 14'b0;
    	else if(retrace_start && retrace_cnt[1] && !nxt_retrace_over && !WEB_retrace)   cost <= cost + weight_output[x_retrace[4:0] * 4 +:4]; 
		else								                       						cost <= cost;
  	end
end


///////////////////  		FILLING       ////////////////////


////////////////   source, sink, net id parameter //////////////////////////
always @(posedge clk, negedge rst_n)begin
	if(!rst_n)       flag <= 0;
	else begin
		if(in_valid) flag <= flag + 1;
		else		 flag <= 0;
	end
end

always @(posedge clk, negedge rst_n)begin
	if(!rst_n) num_cnt <= 0;
	else begin
		if(in_valid) begin
			if(flag) num_cnt <= num_cnt + 1;
			else	 num_cnt <= num_cnt;
		end
		else		 num_cnt <= 0;
	end
end

integer i, j;
always @(posedge clk, negedge rst_n)begin
	if(!rst_n) begin
		for(i = 0 ;i < 15; i = i + 1)begin
					 net_num[i] <= 4'd0;
		end
	end
	else begin
		if(in_valid) net_num[num_cnt] <= net_id;
		else		 net_num[num_cnt] <= net_num[num_cnt];
	end
end



always @(posedge clk, negedge rst_n)begin
	if(!rst_n) 		 total_net_num <= 0;
	else begin
		if(in_valid) total_net_num <= num_cnt;
		else		 total_net_num <= total_net_num;
	end
end

always @(posedge clk, negedge rst_n)begin
	if(!rst_n) begin
		for(i = 0 ;i < 15; i = i + 1)begin
			x_source[i] <= 6'd0;
			y_source[i] <= 6'd0;
		end
	end
	else begin
		if(in_valid && !flag) begin
			x_source[num_cnt] <= loc_x;
			y_source[num_cnt] <= loc_y;
		end
	end
end

always @(posedge clk, negedge rst_n)begin
	if(!rst_n) begin
		for(i = 0 ;i < 15; i = i + 1)begin
			x_sink[i] <= 6'd0;
			y_sink[i] <= 6'd0;
		end
	end
	else begin
		if(in_valid && flag) begin
			x_sink[num_cnt] <= loc_x;
			y_sink[num_cnt] <= loc_y;
		end
	end
end

always @(posedge clk, negedge rst_n)begin
	if(!rst_n) cur_net <= 0;
	else begin
		if(RETRACE_reg && retrace_over)        cur_net <= cur_net + 1;
		else if(c_state == DRAM_READ)          cur_net <= 0;
		else								   cur_net <= cur_net;
	end
end
///////////////     filling path_map ////////////
///////////////     0 : empty        ////////////
///////////////     1 : block        ////////////
///////////////     2 : first fill   ////////////
///////////////     3 : second fill  ////////////
reg [1:0] fill_cnt; /// decide first and second cycle
wire [5:0] y_axis = read_DRAM_cnt[6:1];
wire [5:0] x_axis = (read_DRAM_cnt[0] << 5);
always @(posedge clk, negedge rst_n)begin
	if(!rst_n) fill_cnt <= 0;
	else begin
		if(c_state == FILLING)begin
			if(fill_cnt < 2) fill_cnt <= fill_cnt + 1;
			else             fill_cnt <= fill_cnt;
		end
		else                 fill_cnt <= 0;
	end
end

wire FILLING_reg = (c_state == FILLING);
wire DRAM_READ_reg = (c_state == DRAM_READ);

reg [1:0] filling_num;
always @(posedge clk, negedge rst_n)begin
	if(!rst_n) filling_over <= 0;
	else	   filling_over <= (path_map[y_sink[cur_net]][x_sink[cur_net]][1] && c_state == FILLING); /// filling至sink有值了
end

//assign filling_over = (path_map[y_sink[cur_net]][x_sink[cur_net]][1]); /// filling至sink有值了
always @(posedge clk, negedge rst_n)begin
	if(!rst_n) 			                      					 filling_num <= 0;
	else begin
		if(RETRACE_reg) begin
			if(retrace_start && WEB_retrace && retrace_cnt > 0)  filling_num <= filling_num - 1;
			else			          		  					 filling_num <= filling_num;
		end
		else if(c_state == FILLING)begin
			if(filling_over)                  					 filling_num <= filling_num - 3;
			else			                  					 filling_num <= filling_num + 1;
		end 
		
		else			              		  					 filling_num <= 0;
	end
end


reg flip_flag;
always @(posedge clk, negedge rst_n)begin
	if(!rst_n) 				   flip_flag <= 0;
	else begin
		if(c_state == FILLING) flip_flag <= ~flip_flag;
		else			       flip_flag <= flip_flag;
	end
end



always @(posedge clk, negedge rst_n)begin
	if(!rst_n)begin
		for(i = 0; i < 64; i = i + 1)begin
			for(j = 0; j < 64 ; j = j + 1)begin
				path_map[i][j] <= 0;
			end
		end
	end
	else begin
		if(DRAM_READ_reg)begin
			for(i = 0; i < 32; i = i + 1)begin
				path_map[y_axis][x_axis + i] <= {1'd0, |rdata_m_inf[i * 4 +: 4] }; /// rdata_m_inf[i * 4 + 3: i * 4]過不了??
			end
		end
		else if(FILLING_reg)begin
			if(fill_cnt == 2'd0)        path_map[y_source[cur_net]][x_source[cur_net]] <= {1, filling_num[1]}; /// source 設為2
			else if(fill_cnt == 2'd1)   path_map[y_sink[cur_net]][x_sink[cur_net]]     <= 2'd0; /// sink 設為  0
			else begin
				for (i = 1; i < 63; i = i + 1) begin
					for (j = 1; j < 63; j = j + 1) begin 
						if((path_map[i][j] == 2'd0) && (path_map[i-1][j][1] | path_map[i+1][j][1] | path_map[i][j-1][1] | path_map[i][j+1][1])) path_map[i][j] <= {1'b1, filling_num[1]};  ////332233223322
						else 																													path_map[i][j] <= path_map[i][j];		   ////上下左右同時找
					end
				end
				////////////////////    Edge //////////////////////////////////////
				/////////////////// cur_xminus and cur_xplus ////////////////////////////////
				for (i = 1; i < 63; i = i + 1) begin
					if ((path_map[0][i] == 2'd0) && (path_map[0][i-1][1] | path_map[0][i+1][1] | path_map[1][i][1]))     path_map[0][i] <= {1'b1, filling_num[1]};
					else																							     path_map[0][i] <= path_map[0][i];
					
					if ((path_map[63][i] == 2'd0) && (path_map[63][i-1][1] | path_map[63][i+1][1] | path_map[62][i][1])) path_map[63][i] <= {1'b1, filling_num[1]};
					else																								 path_map[63][i] <= path_map[63][i];
				end
				////////////////// cur_yplus and cur_yminus ///////////////////////////////////
				for (i = 1; i < 63; i = i + 1) begin
					if((path_map[i][0] == 2'd0) && (path_map[i-1][0][1] | path_map[i+1][0][1] | path_map[i][1][1]))	     path_map[i][0] <= {1'b1, filling_num[1]};
					else																								 path_map[i][0] <= path_map[i][0];
			
					if((path_map[i][63] == 2'd0) && (path_map[i-1][63][1] | path_map[i+1][63][1] | path_map[i][62][1]))  path_map[i][63] <= {1'b1, filling_num[1]};
					else																								 path_map[i][63] <= path_map[i][63];
				end
				
				/////////////////   cur_xminus and cur_xplus corner of cur_yplus ////////////////////
				if((path_map[0][0] == 2'd0) && ( path_map[0][1][1] | path_map[1][0][1]))        						 path_map[0][0] <= {1'b1, filling_num[1]};
				else																	        						 path_map[0][0] <= path_map[0][0];
					
				if((path_map[0][63] == 2'd0) && ( path_map[0][62][1] | path_map[1][63][1]))     						 path_map[0][63] <= {1'b1, filling_num[1]};
				else 																	        						 path_map[0][63] <= path_map[0][63];
					
				/////////////////   cur_xminus and cur_xplus corner of cur_yminus ////////////////////
				if((path_map[63][0] == 2'd0) && ( path_map[63][1][1] | path_map[62][0][1]))     						 path_map[63][0] <= {1'b1, filling_num[1]};
				else 																	        						 path_map[63][0] <= path_map[63][0];
					
				if((path_map[63][63] == 2'd0) && ( path_map[63][62][1] | path_map[62][63][1]))  						 path_map[63][63] <= {1'b1, filling_num[1]};
				else 																									 path_map[63][63] <= path_map[63][63];
			end
		end
		else if (retrace_over) begin
            for(i = 0;i < 64;i = i + 1) begin
                for(j = 0;j < 64;j = j + 1) begin
					if(path_map[i][j] == 2 || path_map[i][j] == 3)  path_map[i][j] <= 2'd0;   /// turn 23 to 0
                end
       	    end
        end
		else if(retrace_start && retrace_cnt == 2) begin
			path_map[y_retrace][x_retrace] <= 2'd1; ///turn retrace output to block
		end
	end
end


// ===============================================================
//  					RETRACE
// ===============================================================
reg cur_yplus, cur_yminus, cur_xminus, cur_xplus;
assign retrace_start = (RETRACE_reg && weight_read_done);

always @(posedge clk, negedge rst_n)begin
    if(!rst_n) retrace_cnt <= 0;
    else begin
        if(retrace_start) begin
            if(retrace_cnt < 2) retrace_cnt <= retrace_cnt + 1;
            else        		retrace_cnt <= retrace_cnt;
        end
        else            		retrace_cnt <= 0;
    end
end

always @(posedge clk, negedge rst_n) begin
    if (!rst_n) begin
        x_retrace   <= 6'd0;
        y_retrace   <= 6'd0;
    end
    else begin
        if (retrace_start && retrace_cnt == 2 && RETRACE_reg && !retrace_over) begin  
            x_retrace <= next_x_retrace;
            y_retrace <= next_y_retrace;
        end 
        else begin
            x_retrace <= x_sink[cur_net];
            y_retrace <= y_sink[cur_net];
        end
    end
end

always @(posedge clk, negedge rst_n) begin
    if (!rst_n) WEB_retrace <= 1'b1;
    else begin
        if (retrace_start && retrace_cnt == 2 && RETRACE_reg && !retrace_over) WEB_retrace <= ~WEB_retrace; ///1cycle讀取 1cycle寫入
		else 														  		   WEB_retrace <= 1'b1; 
    end
end

///////////////// 確認 xy 有沒有在邊界/////////////////////////////////
always @(posedge clk, negedge rst_n) begin //// x = 0則無法往左
	if(!rst_n)				   cur_xminus <= 1'b0; 
	else begin
		if (|next_x_retrace)   cur_xminus <= (path_map[next_y_retrace][next_x_retrace - 1][0] == filling_num[1] && path_map[next_y_retrace][next_x_retrace - 1][1]); 
    	else                   cur_xminus <= 1'b0; 
	end
end

always @(posedge clk, negedge rst_n) begin  /// x = 63則無法往右
	if(!rst_n) 				   cur_xplus <= 1'b0;
	else begin
		if (&next_x_retrace)   cur_xplus <= 1'b0;
    	else                   cur_xplus <= (path_map[next_y_retrace][next_x_retrace + 1][0] == filling_num[1] && path_map[next_y_retrace][next_x_retrace + 1][1]);
	end
end

always @(posedge clk, negedge rst_n) begin //// y = 63則無法往下
	if(!rst_n) 					   cur_yminus <= 1'b0; 
	else begin
		if (&next_y_retrace)  cur_yminus <= 1'b0; 
    	else                  cur_yminus <= (path_map[next_y_retrace + 1][next_x_retrace][0] == filling_num[1] && path_map[next_y_retrace + 1][next_x_retrace][1]);
	end
end

always @(posedge clk, negedge rst_n) begin  //// y = 0則無法往上
	if(!rst_n) 				   cur_yplus <= 1'b0; 
	else begin
		if (|next_y_retrace)   cur_yplus <= (path_map[next_y_retrace - 1][next_x_retrace][0] == filling_num[1] && path_map[next_y_retrace - 1][next_x_retrace][1]); 
    	else                   cur_yplus <= 1'b0; 
	end
end



always @(posedge clk, negedge rst_n)begin
	if(!rst_n)begin
		next_x_retrace <= 0;
        next_y_retrace <= 0;
	end
	else begin
		case({retrace_start, retrace_cnt, WEB_retrace})  /// retrace_cnt == 1 sink retrace, else 其他的retrace, WEB_retrace時不動
			4'b1100, 4'b1010, 4'b1011: begin
				case ({cur_yminus, cur_yplus, cur_xplus, cur_xminus})
					4'b0001: begin                           
						next_x_retrace <= next_x_retrace - 1;
						next_y_retrace <= next_y_retrace;  
					end
					4'b0010, 4'b0011: begin                  
						next_x_retrace <= next_x_retrace + 1;
						next_y_retrace <= next_y_retrace;
					end
					4'b0100, 4'b0101, 4'b0110, 4'b0111: begin 
						next_x_retrace <= next_x_retrace;
						next_y_retrace <= next_y_retrace - 1;
					end
					default: begin                            
						next_x_retrace <= next_x_retrace;
						next_y_retrace <= next_y_retrace + 1;
					end
				endcase
			end
			4'b1101:begin
				next_x_retrace <= next_x_retrace;
				next_y_retrace <= next_y_retrace;
			end
			default: begin
				next_x_retrace <= x_sink[cur_net];
				next_y_retrace <= y_sink[cur_net];
			end
		endcase
	end
end



always @(posedge clk, negedge rst_n) begin
    if (!rst_n) retrace_over <= 1'b0;
    else        retrace_over <= nxt_retrace_over;
end

// ===============================================================
//  					put result into SRAM
// ===============================================================
always @* begin   ///data_back[i * 4 + 3 : i * 4]不能用???
    for (i = 0 ;i < 32; i = i + 1) begin
        if (i == x_retrace[4:0]) data_back[i * 4+: 4] = net_num[cur_net]; ////新寫的路徑
        else                     data_back[i * 4+: 4] = loc_output[i * 4 +: 4];////原始的路徑
    end
end


// ===============================================================
//  					SRAM
// ===============================================================
explore_location_map  L1( .A0(address_loc[0]), .A1(address_loc[1]), .A2(address_loc[2]), .A3(address_loc[3]), .A4(address_loc[4]), 
						  .A5(address_loc[5]), .A6(address_loc[6]),
                          
                          .DO0(loc_output[0]),    .DO1(loc_output[1]),    .DO2(loc_output[2]),    .DO3(loc_output[3]),    .DO4(loc_output[4]),    .DO5(loc_output[5]),     
						  .DO6(loc_output[6]),    .DO7(loc_output[7]),    .DO8(loc_output[8]),    .DO9(loc_output[9]),    .DO10(loc_output[10]),  .DO11(loc_output[11]),   
						  .DO12(loc_output[12]),  .DO13(loc_output[13]),  .DO14(loc_output[14]),  .DO15(loc_output[15]),  .DO16(loc_output[16]),  .DO17(loc_output[17]),   
						  .DO18(loc_output[18]),  .DO19(loc_output[19]),  .DO20(loc_output[20]),  .DO21(loc_output[21]),  .DO22(loc_output[22]),  .DO23(loc_output[23]),   
						  .DO24(loc_output[24]),  .DO25(loc_output[25]),  .DO26(loc_output[26]),  .DO27(loc_output[27]),  .DO28(loc_output[28]),  .DO29(loc_output[29]),
                          .DO30(loc_output[30]),  .DO31(loc_output[31]),  .DO32(loc_output[32]),  .DO33(loc_output[33]),  .DO34(loc_output[34]),  .DO35(loc_output[35]),   
						  .DO36(loc_output[36]),  .DO37(loc_output[37]),  .DO38(loc_output[38]),  .DO39(loc_output[39]),  .DO40(loc_output[40]),  .DO41(loc_output[41]),   
						  .DO42(loc_output[42]),  .DO43(loc_output[43]),  .DO44(loc_output[44]),  .DO45(loc_output[45]),  .DO46(loc_output[46]),  .DO47(loc_output[47]),   
						  .DO48(loc_output[48]),  .DO49(loc_output[49]),  .DO50(loc_output[50]),  .DO51(loc_output[51]),  .DO52(loc_output[52]),  .DO53(loc_output[53]),   
						  .DO54(loc_output[54]),  .DO55(loc_output[55]),  .DO56(loc_output[56]),  .DO57(loc_output[57]),  .DO58(loc_output[58]),  .DO59(loc_output[59]),
                          .DO60(loc_output[60]),  .DO61(loc_output[61]),  .DO62(loc_output[62]),  .DO63(loc_output[63]),  .DO64(loc_output[64]),  .DO65(loc_output[65]),   
						  .DO66(loc_output[66]),  .DO67(loc_output[67]),  .DO68(loc_output[68]),  .DO69(loc_output[69]),  .DO70(loc_output[70]),  .DO71(loc_output[71]),   
						  .DO72(loc_output[72]),  .DO73(loc_output[73]),  .DO74(loc_output[74]),  .DO75(loc_output[75]),  .DO76(loc_output[76]),  .DO77(loc_output[77]),
					      .DO78(loc_output[78]),  .DO79(loc_output[79]),  .DO80(loc_output[80]),  .DO81(loc_output[81]),  .DO82(loc_output[82]),  .DO83(loc_output[83]),   
						  .DO84(loc_output[84]),  .DO85(loc_output[85]),  .DO86(loc_output[86]),  .DO87(loc_output[87]),  .DO88(loc_output[88]),  .DO89(loc_output[89]),
                          .DO90(loc_output[90]),  .DO91(loc_output[91]),  .DO92(loc_output[92]),  .DO93(loc_output[93]),  .DO94(loc_output[94]),  .DO95(loc_output[95]),   
						  .DO96(loc_output[96]),  .DO97(loc_output[97]),  .DO98(loc_output[98]),  .DO99(loc_output[99]),  .DO100(loc_output[100]),.DO101(loc_output[101]), 
						  .DO102(loc_output[102]),  .DO103(loc_output[103]),  .DO104(loc_output[104]),  .DO105(loc_output[105]),  .DO106(loc_output[106]),  .DO107(loc_output[107]), 
						  .DO108(loc_output[108]),  .DO109(loc_output[109]),  .DO110(loc_output[110]),  .DO111(loc_output[111]),  .DO112(loc_output[112]),  .DO113(loc_output[113]), 
						  .DO114(loc_output[114]),  .DO115(loc_output[115]),  .DO116(loc_output[116]),  .DO117(loc_output[117]),  .DO118(loc_output[118]),  .DO119(loc_output[119]),
                          .DO120(loc_output[120]),  .DO121(loc_output[121]),  .DO122(loc_output[122]),  .DO123(loc_output[123]),  .DO124(loc_output[124]),  .DO125(loc_output[125]), 
						  .DO126(loc_output[126]),  .DO127(loc_output[127]), 
                          
                          .DI0(loc_input[0]),    .DI1(loc_input[1]),     .DI2(loc_input[2]),    .DI3(loc_input[3]),     .DI4(loc_input[4]),     .DI5(loc_input[5]),     
						  .DI6(loc_input[6]),    .DI7(loc_input[7]),     .DI8(loc_input[8]),    .DI9(loc_input[9]),     .DI10(loc_input[10]),   .DI11(loc_input[11]),   
						  .DI12(loc_input[12]),  .DI13(loc_input[13]),   .DI14(loc_input[14]),  .DI15(loc_input[15]),   .DI16(loc_input[16]),   .DI17(loc_input[17]),   
						  .DI18(loc_input[18]),  .DI19(loc_input[19]),   .DI20(loc_input[20]),  .DI21(loc_input[21]),   .DI22(loc_input[22]),   .DI23(loc_input[23]),   
						  .DI24(loc_input[24]),  .DI25(loc_input[25]),   .DI26(loc_input[26]),  .DI27(loc_input[27]),   .DI28(loc_input[28]),   .DI29(loc_input[29]),
                          .DI30(loc_input[30]),  .DI31(loc_input[31]),   .DI32(loc_input[32]),  .DI33(loc_input[33]),   .DI34(loc_input[34]),   .DI35(loc_input[35]),   
						  .DI36(loc_input[36]),  .DI37(loc_input[37]),   .DI38(loc_input[38]),  .DI39(loc_input[39]),   .DI40(loc_input[40]),   .DI41(loc_input[41]),   
						  .DI42(loc_input[42]),  .DI43(loc_input[43]),   .DI44(loc_input[44]),  .DI45(loc_input[45]),   .DI46(loc_input[46]),   .DI47(loc_input[47]),   
						  .DI48(loc_input[48]),  .DI49(loc_input[49]),   .DI50(loc_input[50]),  .DI51(loc_input[51]),   .DI52(loc_input[52]),   .DI53(loc_input[53]),   
						  .DI54(loc_input[54]),  .DI55(loc_input[55]),   .DI56(loc_input[56]),  .DI57(loc_input[57]),   .DI58(loc_input[58]),   .DI59(loc_input[59]),
                          .DI60(loc_input[60]),  .DI61(loc_input[61]),   .DI62(loc_input[62]),  .DI63(loc_input[63]),   .DI64(loc_input[64]),   .DI65(loc_input[65]),   
						  .DI66(loc_input[66]),  .DI67(loc_input[67]),   .DI68(loc_input[68]),  .DI69(loc_input[69]),   .DI70(loc_input[70]),   .DI71(loc_input[71]),   
						  .DI72(loc_input[72]),  .DI73(loc_input[73]),   .DI74(loc_input[74]),  .DI75(loc_input[75]),   .DI76(loc_input[76]),   .DI77(loc_input[77]),   
						  .DI78(loc_input[78]),  .DI79(loc_input[79]),   .DI80(loc_input[80]),  .DI81(loc_input[81]),   .DI82(loc_input[82]),   .DI83(loc_input[83]),   
						  .DI84(loc_input[84]),  .DI85(loc_input[85]),   .DI86(loc_input[86]),  .DI87(loc_input[87]),   .DI88(loc_input[88]),   .DI89(loc_input[89]),
                          .DI90(loc_input[90]),  .DI91(loc_input[91]),   .DI92(loc_input[92]),  .DI93(loc_input[93]),   .DI94(loc_input[94]),   .DI95(loc_input[95]),   
						  .DI96(loc_input[96]),  .DI97(loc_input[97]),   .DI98(loc_input[98]),  .DI99(loc_input[99]),   .DI100(loc_input[100]), .DI101(loc_input[101]), 
						  .DI102(loc_input[102]),  .DI103(loc_input[103]),  .DI104(loc_input[104]),  .DI105(loc_input[105]),  .DI106(loc_input[106]),  
						  .DI107(loc_input[107]),  .DI108(loc_input[108]),  .DI109(loc_input[109]),  .DI110(loc_input[110]),  .DI111(loc_input[111]),  
						  .DI112(loc_input[112]),  .DI113(loc_input[113]),  .DI114(loc_input[114]),  .DI115(loc_input[115]),  .DI116(loc_input[116]),  
						  .DI117(loc_input[117]),  .DI118(loc_input[118]),  .DI119(loc_input[119]),  .DI120(loc_input[120]),  .DI121(loc_input[121]),  
						  .DI122(loc_input[122]),  .DI123(loc_input[123]),  .DI124(loc_input[124]),  .DI125(loc_input[125]),  .DI126(loc_input[126]),  .DI127(loc_input[127]), 

                          .CK(clk), .WEB(WEB_loc), .OE(1'b1), .CS(1'b1));




explore_weight_map W1( .A0(address_weight[0]), .A1(address_weight[1]), .A2(address_weight[2]), .A3(address_weight[3]), .A4(address_weight[4]), 
					   .A5(address_weight[5]), .A6(address_weight[6]),
                          
                       .DO0(weight_output[0]),    .DO1(weight_output[1]),    .DO2(weight_output[2]),    .DO3(weight_output[3]),    .DO4(weight_output[4]),    .DO5(weight_output[5]),     
					   .DO6(weight_output[6]),    .DO7(weight_output[7]),    .DO8(weight_output[8]),    .DO9(weight_output[9]),    .DO10(weight_output[10]),  .DO11(weight_output[11]),   
					   .DO12(weight_output[12]),  .DO13(weight_output[13]),  .DO14(weight_output[14]),  .DO15(weight_output[15]),  .DO16(weight_output[16]),  .DO17(weight_output[17]),   
					   .DO18(weight_output[18]),  .DO19(weight_output[19]),  .DO20(weight_output[20]),  .DO21(weight_output[21]),  .DO22(weight_output[22]),  .DO23(weight_output[23]),   
					   .DO24(weight_output[24]),  .DO25(weight_output[25]),  .DO26(weight_output[26]),  .DO27(weight_output[27]),  .DO28(weight_output[28]),  .DO29(weight_output[29]),
                       .DO30(weight_output[30]),  .DO31(weight_output[31]),  .DO32(weight_output[32]),  .DO33(weight_output[33]),  .DO34(weight_output[34]),  .DO35(weight_output[35]),   
				       .DO36(weight_output[36]),  .DO37(weight_output[37]),  .DO38(weight_output[38]),  .DO39(weight_output[39]),  .DO40(weight_output[40]),  .DO41(weight_output[41]),   
					   .DO42(weight_output[42]),  .DO43(weight_output[43]),  .DO44(weight_output[44]),  .DO45(weight_output[45]),  .DO46(weight_output[46]),  .DO47(weight_output[47]),   
					   .DO48(weight_output[48]),  .DO49(weight_output[49]),  .DO50(weight_output[50]),  .DO51(weight_output[51]),  .DO52(weight_output[52]),  .DO53(weight_output[53]),   
					   .DO54(weight_output[54]),  .DO55(weight_output[55]),  .DO56(weight_output[56]),  .DO57(weight_output[57]),  .DO58(weight_output[58]),  .DO59(weight_output[59]),
                       .DO60(weight_output[60]),  .DO61(weight_output[61]),  .DO62(weight_output[62]),  .DO63(weight_output[63]),  .DO64(weight_output[64]),  .DO65(weight_output[65]),   
			           .DO66(weight_output[66]),  .DO67(weight_output[67]),  .DO68(weight_output[68]),  .DO69(weight_output[69]),  .DO70(weight_output[70]),  .DO71(weight_output[71]),   
					   .DO72(weight_output[72]),  .DO73(weight_output[73]),  .DO74(weight_output[74]),  .DO75(weight_output[75]),  .DO76(weight_output[76]),  .DO77(weight_output[77]),
					   .DO78(weight_output[78]),  .DO79(weight_output[79]),  .DO80(weight_output[80]),  .DO81(weight_output[81]),  .DO82(weight_output[82]),  .DO83(weight_output[83]),   
					   .DO84(weight_output[84]),  .DO85(weight_output[85]),  .DO86(weight_output[86]),  .DO87(weight_output[87]),  .DO88(weight_output[88]),  .DO89(weight_output[89]),
                       .DO90(weight_output[90]),  .DO91(weight_output[91]),  .DO92(weight_output[92]),  .DO93(weight_output[93]),  .DO94(weight_output[94]),  .DO95(weight_output[95]),   
					   .DO96(weight_output[96]),  .DO97(weight_output[97]),  .DO98(weight_output[98]),  .DO99(weight_output[99]),  .DO100(weight_output[100]),.DO101(weight_output[101]), 
					   .DO102(weight_output[102]),  .DO103(weight_output[103]),  .DO104(weight_output[104]),  .DO105(weight_output[105]),  .DO106(weight_output[106]),  .DO107(weight_output[107]), 
					   .DO108(weight_output[108]),  .DO109(weight_output[109]),  .DO110(weight_output[110]),  .DO111(weight_output[111]),  .DO112(weight_output[112]),  .DO113(weight_output[113]), 
					   .DO114(weight_output[114]),  .DO115(weight_output[115]),  .DO116(weight_output[116]),  .DO117(weight_output[117]),  .DO118(weight_output[118]),  .DO119(weight_output[119]),
                       .DO120(weight_output[120]),  .DO121(weight_output[121]),  .DO122(weight_output[122]),  .DO123(weight_output[123]),  .DO124(weight_output[124]),  .DO125(weight_output[125]), 
					   .DO126(weight_output[126]),  .DO127(weight_output[127]), 
                          
                       .DI0(weight_input[0]),    .DI1(weight_input[1]),     .DI2(weight_input[2]),    .DI3(weight_input[3]),     .DI4(weight_input[4]),     .DI5(weight_input[5]),     
					   .DI6(weight_input[6]),    .DI7(weight_input[7]),     .DI8(weight_input[8]),    .DI9(weight_input[9]),     .DI10(weight_input[10]),   .DI11(weight_input[11]),   
					   .DI12(weight_input[12]),  .DI13(weight_input[13]),   .DI14(weight_input[14]),  .DI15(weight_input[15]),   .DI16(weight_input[16]),   .DI17(weight_input[17]),   
					   .DI18(weight_input[18]),  .DI19(weight_input[19]),   .DI20(weight_input[20]),  .DI21(weight_input[21]),   .DI22(weight_input[22]),   .DI23(weight_input[23]),   
					   .DI24(weight_input[24]),  .DI25(weight_input[25]),   .DI26(weight_input[26]),  .DI27(weight_input[27]),   .DI28(weight_input[28]),   .DI29(weight_input[29]),
                       .DI30(weight_input[30]),  .DI31(weight_input[31]),   .DI32(weight_input[32]),  .DI33(weight_input[33]),   .DI34(weight_input[34]),   .DI35(weight_input[35]),   
					   .DI36(weight_input[36]),  .DI37(weight_input[37]),   .DI38(weight_input[38]),  .DI39(weight_input[39]),   .DI40(weight_input[40]),   .DI41(weight_input[41]),   
					   .DI42(weight_input[42]),  .DI43(weight_input[43]),   .DI44(weight_input[44]),  .DI45(weight_input[45]),   .DI46(weight_input[46]),   .DI47(weight_input[47]),   
					   .DI48(weight_input[48]),  .DI49(weight_input[49]),   .DI50(weight_input[50]),  .DI51(weight_input[51]),   .DI52(weight_input[52]),   .DI53(weight_input[53]),   
					   .DI54(weight_input[54]),  .DI55(weight_input[55]),   .DI56(weight_input[56]),  .DI57(weight_input[57]),   .DI58(weight_input[58]),   .DI59(weight_input[59]),
                       .DI60(weight_input[60]),  .DI61(weight_input[61]),   .DI62(weight_input[62]),  .DI63(weight_input[63]),   .DI64(weight_input[64]),   .DI65(weight_input[65]),   
					   .DI66(weight_input[66]),  .DI67(weight_input[67]),   .DI68(weight_input[68]),  .DI69(weight_input[69]),   .DI70(weight_input[70]),   .DI71(weight_input[71]),   
					   .DI72(weight_input[72]),  .DI73(weight_input[73]),   .DI74(weight_input[74]),  .DI75(weight_input[75]),   .DI76(weight_input[76]),   .DI77(weight_input[77]),   
					   .DI78(weight_input[78]),  .DI79(weight_input[79]),   .DI80(weight_input[80]),  .DI81(weight_input[81]),   .DI82(weight_input[82]),   .DI83(weight_input[83]),   
					   .DI84(weight_input[84]),  .DI85(weight_input[85]),   .DI86(weight_input[86]),  .DI87(weight_input[87]),   .DI88(weight_input[88]),   .DI89(weight_input[89]),
                       .DI90(weight_input[90]),  .DI91(weight_input[91]),   .DI92(weight_input[92]),  .DI93(weight_input[93]),   .DI94(weight_input[94]),   .DI95(weight_input[95]),   
					   .DI96(weight_input[96]),  .DI97(weight_input[97]),   .DI98(weight_input[98]),  .DI99(weight_input[99]),   .DI100(weight_input[100]), .DI101(weight_input[101]), 
					   .DI102(weight_input[102]),  .DI103(weight_input[103]),  .DI104(weight_input[104]),  .DI105(weight_input[105]),  .DI106(weight_input[106]),  .DI107(weight_input[107]), 
					   .DI108(weight_input[108]),  .DI109(weight_input[109]),  .DI110(weight_input[110]),  .DI111(weight_input[111]),  .DI112(weight_input[112]),  .DI113(weight_input[113]), 
				       .DI114(weight_input[114]),  .DI115(weight_input[115]),  .DI116(weight_input[116]),  .DI117(weight_input[117]),  .DI118(weight_input[118]),  .DI119(weight_input[119]),
                       .DI120(weight_input[120]),  .DI121(weight_input[121]),  .DI122(weight_input[122]),  .DI123(weight_input[123]),  .DI124(weight_input[124]),  .DI125(weight_input[125]), 
					   .DI126(weight_input[126]),  .DI127(weight_input[127]), 

                       .CK(clk), .WEB(WEB_weight), .OE(1'b1), .CS(1'b1));

					   
endmodule





